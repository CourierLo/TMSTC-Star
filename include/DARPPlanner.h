#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <string>
#include <fstream>
#include <ros/package.h>
#include <random>
#include <thread>
#include <ctime>
#include <time.h>
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include <algorithm>
#include <mutex>
#include <iomanip>
#include <DARP.h>
#include <STC.h>
#include <MaximumSubRectDivision.h>
#include <ACO_STC.h>
#include <Dinic.h>

typedef std::vector<std::vector<int>> Mat;
using std::string;

class DARPPlanner
{
public:
  int number_of_robots;
  int ob = 0;

  // Solve time variables
  double tic, toc;
  double solve_time;

  // Variable to hold the best paths
  Mat src, Map, Region, MST;
  std::vector<std::pair<int, int>> robotOriginCoor;
  nav_msgs::OccupancyGrid *coverage_map;

  DARP *darp;

  int smallcols, smallrows, bigrows, bigcols;

  DARPPlanner(Mat& _map, Mat& _region, std::vector<std::pair<double, double> >& robot_states, int robot_num, nav_msgs::OccupancyGrid *cov_map) : Region(_region), Map(_map), number_of_robots(robot_num), coverage_map(cov_map) {
    // construct src matrix for DARP,coverage map(2x) transfer to small matrix src(1x)
    // robot : 2 / obstacle : 1  / free cells : 0
    // x y 别弄反了！
    bigcols = Region[0].size();   bigrows = Region.size();
    smallcols = Map[0].size();    smallrows = Map.size();

    src.resize(Map.size(), std::vector<int>(Map[0].size(), 0));
    for(int i = 0; i < Map.size(); ++i)
      for(int j = 0; j < Map[0].size(); ++j)
        src[i][j] = !Map[i][j];
      
    for(int i = 0; i < robot_states.size(); ++i){
      int cmap_x = (robot_states[i].first - coverage_map->info.origin.position.x) / coverage_map->info.resolution;
      int cmap_y = (robot_states[i].second - coverage_map->info.origin.position.y) / coverage_map->info.resolution;
      src[cmap_y / 2][cmap_x / 2] = 2;
      //save robots' initial coordinate for STC
      robotOriginCoor.push_back({ cmap_x, cmap_y });
      std::cout << "robot 2D coor: " << cmap_x << " " << cmap_y << std::endl;
    }

    std::cout << "src matrix : \n";
    DARP::printIntMat(src);

    double variateWeight = 0.01;
    int discr = 30, iters = 80000;
    double randomLevel = 0.0001;
    darp = new DARP(src, iters, variateWeight, randomLevel, discr, false);
  }

  Mat plan(string shape){
    tic = ros::Time::now().toSec();
    std::cout << "start planning....\n";

    (*darp).constructAssignmentMatrix();
    Mat allPaths;
    
    std::cout << "finish constructing A matrix.\n";
    for(int i = 0; i < number_of_robots; ++i){
      std::cout << "\ncheck robot region " << i << ": \n";
      DARP::printIntMat((*darp).robotRegion[i]);
    }

    // 机器人顺序和覆盖区域下标对应才行，所以先调整一下robotRegion的顺序先
    vector<Mat> robotRegion_tmp((*darp).robotRegion.size());
    for(int i = 0; i < robotOriginCoor.size(); ++i){
      int sy = robotOriginCoor[i].first / 2;
      int sx = robotOriginCoor[i].second / 2;
      cout << "No." << i << "'s robot's 2d coor: "<< sx << " " << sy << endl;
      for(int j = 0; j < (*darp).robotRegion.size(); ++j){
        if((*darp).robotRegion[j][sx][sy])  robotRegion_tmp[i] = (*darp).robotRegion[j];
      }
    }
    (*darp).robotRegion = robotRegion_tmp;

    cout << "After adjusting...\n";
    for(int i = 0; i < number_of_robots; ++i){
      std::cout << "\ncheck robot region " << i << ": \n";
      DARP::printIntMat((*darp).robotRegion[i]);
    }

    // create path using different shape of spanning trees
    for(int i = 0; i < number_of_robots; ++i){
      Division div((*darp).robotRegion[i]);
      int index = robotOriginCoor[i].first + robotOriginCoor[i].second * bigcols;
      if(shape == "DFS_HORIZONTAL"){
        // Normal DFS spanning tree with path
        //STC stc(coverage_map->info.height, coverage_map->info.width, (*darp).robotRegion[i], robotOriginCoor[i]);
        MST = div.dfsWithStackSolver(HORIZONTAL);
      } else if(shape =="DFS_VERTICAL"){
        MST = div.dfsWithStackSolver(VERTICAL);
      } else if(shape == "RECT_DIV"){
        // RECT_DIV
        MST = div.rectDivisionSolver();
      } else if(shape == "ACO_OPT"){
        ACO_STC aco(1, 1, 1, 0.15, 30, 150, (*darp).robotRegion[i], MST);
        MST = aco.aco_stc_solver();
      } else if(shape == "KRUSKAL"){
        MST = div.kruskalSolver();
      } else if(shape == "DINIC"){
        MST = dinic.dinic_solver((*darp).robotRegion[i], true);
      }

      allPaths.push_back(MST2Path(index));

      std::cout << i << "th path size: " << allPaths.back().size() << std::endl;
    }

    toc = ros::Time::now().toSec();
    return allPaths;
  }  

  void get2DCoordinate(int index, int& x, int& y) {
    x = index / Map[0].size();
    y = index % Map[0].size();
  }

  // MST is undirected graph, so it contains (i, j) and (j, i)
	vector<int> MST2Path(int robot_index) {
    Mat pathEdge;
    vector<int> pathSequence;

		vector<unordered_set<int>> vis(Map.size() * Map[0].size(), std::unordered_set<int>{});
		pathEdge.resize(bigcols * bigrows, vector<int>{});

		// ROS中是横x竖y，所以求路径编号的时候得注意一下, bigcols对应的是mapServer地图的x
		for (int from = 0; from < MST.size(); ++from) {
			for (auto to : MST[from]) {
				if ((!vis[from].empty() && vis[from].find(to) != vis[from].end()) || 
					(!vis[to].empty()   && vis[to].find(from) != vis[to].end()) )	continue;

				vis[from].insert(to);  vis[to].insert(from);

				int x1, x2, y1, y2;
				get2DCoordinate(from, x1, y1);  get2DCoordinate(to, x2, y2);
				int p3 = 2 * x1 * bigcols + 2 * y1 + 1;
				int p4 = 2 * x2 * bigcols + 2 * y2;
				int p1 = (2 * x1 + 1) * bigcols + (2 * y1 + 1);
				int p2 = (2 * x2 + 1) * bigcols + 2 * y2;
				int p6 = 2 * x1 * bigcols + 2 * y1;
				int p5 = (2 * x1 + 1) * bigcols + 2 * y1;
				int p8 = 2 * x2 * bigcols + 2 * y2 + 1;
				int p7 = (2 * x2 + 1) * bigcols + 2 * y2 + 1;

				if (abs(from - to) == 1) {
					//horizontal edges
					pathEdge[p1].push_back(p2);		pathEdge[p3].push_back(p4);
					pathEdge[p2].push_back(p1);		pathEdge[p4].push_back(p3);
				}
				else {
					//vertical edges (x, y) creates (2*x + col, 2*y) and (2*x+col+1, 2*y + 1)
					pathEdge[p4].push_back(p5);		pathEdge[p1].push_back(p8);
					pathEdge[p5].push_back(p4); 	pathEdge[p8].push_back(p1);
				}
			}
		}

		for (int i = 0; i < Map.size(); ++i) {
			for (int j = 0; j < Map[0].size(); ++j) {
				if (!Map[i][j])	continue;
				int x = 2 * i, y = 2 * j;
				int cur = i * smallcols + j;
				int p1 = x * bigcols + y;
				int p2 = x * bigcols + y + 1;
				int p3 = (x + 1) * bigcols + y;
				int p4 = (x + 1) * bigcols + y + 1;

				if (j == 0 || vis[cur].find(cur - 1) == vis[cur].end()) {
					pathEdge[p1].push_back(p3);  
					pathEdge[p3].push_back(p1);
				}
				if (j == smallcols - 1 || vis[cur].find(cur + 1) == vis[cur].end()) {
					pathEdge[p2].push_back(p4);
					pathEdge[p4].push_back(p2);
				}
				if (i == 0 || vis[cur].find(cur - smallcols) == vis[cur].end()) {
					pathEdge[p1].push_back(p2);
					pathEdge[p2].push_back(p1);
				}
				if (i == smallrows - 1 || vis[cur].find(cur + smallcols) == vis[cur].end()) {
					pathEdge[p3].push_back(p4);
					pathEdge[p4].push_back(p3);
				}
			}
		}

		// 得到pathEdge后，进一步得到路径序列pathSequence（及其反序列）
		// pathSequnce从第一个机器人的位置开始
		// pathSequence的长度就是圆的长度，invSequence不是
		vector<bool> inPath(bigrows * bigcols, false);
		int cur = robot_index;
		while (!inPath[robot_index] || cur != robot_index) {
			inPath[cur] = true;
			pathSequence.push_back(cur);

			cur = inPath[pathEdge[cur][0]] ? pathEdge[cur][1] : pathEdge[cur][0];
			if(inPath[cur])	break;
		}

		cout << "Finish Constructing Path from ideal spanning tree.\n";
    return pathSequence;

	}

  
};