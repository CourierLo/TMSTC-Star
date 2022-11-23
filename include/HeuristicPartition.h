#ifndef _HEURISTIC_PARTION_H
#define _HEURISTIC_PARTION_H

#include <iostream>
#include <cstdio>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <cstring>
#include <map>
#include <set>
#include <UnionFind.h>
#include <random>

using std::queue; 
using std::vector;
using std::priority_queue;
using std::cout;
using std::endl;
using std::pair;
using std::set;

namespace HeuristicSolver{

#define HORIZONTAL 0
#define VERTICAL 1
#define RESHAPE(i, j) (int)((i) * Map[0].size() + (j))

typedef vector<vector<int>> Mat;
typedef pair<int, int> P;
typedef struct Rectangle {
    bool empty;
    P coordinate;
    bool dir;   // 0 horizontal 1 vertical
} rect;

typedef struct Edge {
	int from;
	int to;
	int cost;
	bool operator < (const Edge& e) const {
		return cost > e.cost;
	}
} edge;

class HeuristicPartition{
    Mat Map, ranks;   // what about MSTC with heuristic partition ?
    vector<int> X, Y, transX, transY;	
    vector<int> height, width;
    int dir[4][2] = { 0, 1, -1, 0, 0, -1, 1, 0 };
    int max_iter;
    vector<vector<rect>> minRanksRec;
    UnionFind uf;

public:
    HeuristicPartition(Mat& _map, int _max_iter) : Map(_map), max_iter(_max_iter){
        uf.init(Map.size() * Map[0].size() + 5);
		ranks.resize(Map.size() * Map[0].size() + 5, vector<int>{});
    }
    vector<vector<rect>> chessboardPartition();
    vector<vector<rect>> orientRectangle(vector<vector<rect>>& rec);
    void minrects2ranks();
    void mergeRanks(Mat& graph);
    Mat hpSolver(bool merge);
    
    bool isSameLine(int a, int b, int c) {
        return a + c == 2 * b;
	}

    int getEdgeVal(Mat& graph, int v1, int v2) {
		int cost = 0;
		if (graph[v1].size() == 1) {
			if (isSameLine(graph[v1][0], v1, v2))	cost -= 2;
		}
		else if (graph[v1].size() == 2) {
			if (isSameLine(graph[v1][0], v1, graph[v1][1]))	cost += 2;
		}
		else if (graph[v1].size() == 3) {
			cost += 2;
		}

		if (graph[v2].size() == 1) {
			if (isSameLine(v1, v2, graph[v2][0]))	cost -= 2;
		}
		else if (graph[v2].size() == 2) {
			if (isSameLine(graph[v2][0], v2, graph[v2][1]))	cost += 2;
		}
		else if (graph[v1].size() == 3) {
			cost += 2;
		}

		return cost;
	}

	int getVertexVal(Mat& graph, int v) {
		if (graph[v].size() == 0)	return 0;
		else if (graph[v].size() == 1)	return 2;
		else if (graph[v].size() == 2) {
			return isSameLine(graph[v][0], v, graph[v][1]) ? 0 : 2;
		}
		else if (graph[v].size() == 3) return 2;
		else  return 4;
	}

	void showRanks(){
		for(int i = 0; i < ranks.size(); ++i){
			if(ranks[i].size() == 0) continue;
			for(int j = 0; j < ranks[i].size(); ++j)
				cout << "(" << i << ", " << ranks[i][j] << ") "; 
			cout << "\n";
		}
	}

	void checkConnectivity(){
		vector<vector<bool> > vis(Map.size(), vector<bool>(Map[0].size(), false));
		int freecells = 0;
		P start{ -1, -1 };
		for(int i = 0; i < Map.size(); ++i){
			for(int j = 0; j < Map[0].size(); ++j){
				if(Map[i][j]){
					freecells++;
					if(start.first == -1) start = { i, j };
				}
			}
		}

		int connectcell = 0;
		queue<P> que;
		que.push(start);
		vis[start.first][start.second] = true;
		while(!que.empty()){
			P p = que.front();   que.pop();
			connectcell++;
			for(int i = 0; i < 4; ++i){
				int dx = p.first + dir[i][0];
				int dy = p.second + dir[i][1];
				if(dx >= 0 && dy >= 0 && dx < Map.size() && dy < Map[0].size() && Map[dx][dy] && !vis[dx][dy]){
					que.push({ dx, dy });
					vis[dx][dy] = true;
				}
			}
		}

		cout << "freecells and connect-cells: " << freecells << " " << connectcell << "\n";
	}

	bool checkMST(Mat& graph, Mat& Map) {
        int vertexNum = 0;
        for (int i = 0; i < Map.size(); ++i)
            for (int j = 0; j < Map[0].size(); ++j)
                if (Map[i][j])	vertexNum++;

        int edgeNum = 0;
        for (int i = 0; i < graph.size(); ++i)	edgeNum += graph[i].size();
        edgeNum /= 2;

        cout << "The number of edges: " << edgeNum << endl;
        cout << "The number of vertexes: " << vertexNum << endl;
        if (edgeNum != vertexNum - 1)	return false;
        return true;
    }
};

}

#endif