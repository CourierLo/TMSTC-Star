#ifndef _MAXIMUM_SUB_RECT_DIVISION_H
#define _MAXIMUM_SUB_RECT_DIVISION_H

#include <vector>
#include <stack>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <queue>
#include <time.h>
#include <unordered_set>

using std::vector;
using std::stack;
using std::cout;
using std::endl;
using std::pair;
using std::queue;
using std::set;
using std::priority_queue;
using std::string;

typedef vector<vector<int>> Mat;
typedef pair<int, int> P;
typedef struct Rectangle {
	P corner;		// 左上角
	int height;
	int width;
	unsigned char dir;
	// 从大到小排序
	bool operator <(const Rectangle& x)const {
		return height * width > x.height * x.width;
	}
} rect;

typedef struct Edge {
	int from;
	int to;
	int cost;
	bool operator < (const Edge& e) const {
		return cost > e.cost;
	}
} edge;

#define HORIZONTAL 0
#define VERTICAL   1
#define BOTH_ORI   2

#define HALF_CONNECTED  0
#define FULL_CONNECTED -1
#define DISCONNECTED    1

#define RECT_SIZE(i) (rectVec[i].height * rectVec[i].width)
#define reshape(i, j) (int)((i) * Map[0].size() + (j))

class Division {
	//int bigrows, bigcols;
	//int smallrows, smallcols;
	Mat MST, Map;
	vector<int> fa;
	//Mat pathEdge;
public:
	int dir[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };
	int dir2[4][2] = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };

	// bigcols对应的是mapServer地图的x，切记切记
	Division(Mat& map) : Map(map){
		//smallrows = Map.size();  smallcols = Map[0].size();
	}

	vector<rect> findMaximumSubRect(Mat& Map);

	vector<rect> maximumSubRectDivision(Mat& Map);

	void orientRect(vector<rect>& rectVec);

	void constructMST(vector<rect>& rectVec);

	void mergeMST(Mat& graph);

	//void MST2Path();

	Mat rectDivisionSolver();

	Mat dfsWithStackSolver(unsigned char dfsDir);

	Mat kruskalSolver();

	bool checkMST(Mat& graph, Mat& Map);

	//void get2DCoordinate(int index, int& x, int& y);

	// utility function
	int find(int x) {
		return fa[x] == x ? x : fa[x] = find(fa[x]);
	}

	bool same(int x, int y) {
		return find(x) == find(y);
	}

	void unite(int x, int y) {
		x = find(x);  y = find(y);
		if (x == y)	return;
		fa[x] = y;		// 不用rank作平衡了
	}

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
};

#endif