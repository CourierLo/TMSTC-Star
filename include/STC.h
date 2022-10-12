#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_set>

typedef std::vector<std::vector<int>> intMatrix;
typedef std::vector<std::vector<double>> doubleMatrix;
typedef std::vector<std::vector<float>> floatMatrix;
typedef std::vector<std::vector<bool>> boolMatrix;

class STC {
public:
	int MAX_NODES;
	intMatrix robotRegion;
	std::vector<int> parent;
	std::vector<std::pair<int, int>> edgeSetSTC;
	std::pair<int, int> robotInitPos;		
	int bigrows, bigcols;
	int smallrows, smallcols;
	int dir[4][2] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
	STC(int r, int c, intMatrix& region, std::pair<int, int> initPos) : bigrows(r), bigcols(c),
		robotRegion(region), robotInitPos(initPos) {
		MAX_NODES = bigrows * bigcols + 1;
		smallrows = robotRegion.size();
		smallcols = robotRegion[0].size();

		for (int i = 0; i < MAX_NODES; ++i)	parent.push_back(i);
	};

	//bugs
	void dfs(int x, int y) {
		// std::cout << "dfs: " << x << " " << y << std::endl;
		for (int i = 0; i < 4; ++i) {
			int dx = x + dir[i][0];
			int dy = y + dir[i][1];
			if (dx >= 0 && dy >= 0 && dx < smallcols && dy < smallrows && robotRegion[dy][dx]) {
				int from = y * smallcols + x, to = dy * smallcols + dx;
				if (!same(from, to)) {
					unite(from, to);
					// edge x-y : x always smaller than y
					if (from > to)	std::swap(from, to);
					edgeSetSTC.push_back({ from, to });
					dfs(dx, dy);
				}
			}
		}
	}

	int find(int x) {
		return parent[x] == x ? x : parent[x] = find(parent[x]);
	}
	
	void unite(int x, int y) {
		x = find(x);  y = find(y);
		parent[x] = y;
	}

	bool same(int x, int y) {
		return find(x) == find(y);
	}

	// recap (i, j) =>  (2*i, 2*j) /  (2*i, 2*j+1)  /  (2*i+1, 2*j) / (2*i+1, 2*j+1) 
	// cols => 2 * cols
	// let's assume edges (x, y) in which x > y
	// because each edge in path will be only visited once, so just follow it.
	// return path point sequence
	void get2DCoordinate(int index, int& x, int& y) {
		y = index / smallcols;
		x = index % smallcols;
	}

	std::vector<int> getPath() {
		// recap
		dfs(robotInitPos.first / 2, robotInitPos.second / 2);

		std::vector<std::unordered_set<int>> tree(smallcols * smallrows, std::unordered_set<int>{});
		for (auto&& p : edgeSetSTC) {
			tree[p.first].insert(p.second);
			tree[p.second].insert(p.first);
		}

		//construct path graph(bidirectional)
		intMatrix pathEdge(MAX_NODES);
		//step 1: connects different region
		for (auto&& p : edgeSetSTC) {
			int ind1 = p.first, ind2 = p.second;
			int x1, y1, x2, y2;
			int newc = bigcols;
			get2DCoordinate(ind1, x1, y1);  get2DCoordinate(ind2, x2, y2);
			//std::cout << "x1 y1 x2 y2: " << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
			int p3 = 2 * y1 * newc + 2 * x1 + 1;
			int p4 = 2 * y2 * newc + 2 * x2;
			int p1 = (2 * y1 + 1) * newc + (2 * x1 + 1);
			int p2 = (2 * y2 + 1) * newc + 2 * x2;
			int p6 = 2 * y1 * newc + 2 * x1;
			int p5 = (2 * y1 + 1) * newc + 2 * x1;
			int p8 = 2 * y2 * newc + 2 * x2 + 1;
			int p7 = (2 * y2 + 1) * newc + 2 * x2 + 1;
			//cout << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 << " " << p6 << " " << p7 << " " << p8 << endl;
			if (abs(ind1 - ind2) == 1) {
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

		// step 2: connects self-region
		// 这里的rc必须是缩图之后的rc，不能是整个
		// 要检查点间的边才行
		for (int i = 0; i < smallrows; ++i) {
			for (int j = 0; j < smallcols; ++j) {
				if (!robotRegion[i][j])	continue;
				int x = 2 * j, y = 2 * i;
				int cur = i * smallcols + j;
				int p1 = y * bigcols + x;
				int p2 = y * bigcols + x + 1;
				int p3 = (y + 1) * bigcols + x;
				int p4 = (y + 1) * bigcols + x + 1;
			
				if (j == 0 || tree[cur].find(cur - 1) == tree[cur].end()) {
					pathEdge[p1].push_back(p3);
					pathEdge[p3].push_back(p1);
				}
				if (j == smallcols - 1 || tree[cur].find(cur + 1) == tree[cur].end()) {
					pathEdge[p2].push_back(p4);
					pathEdge[p4].push_back(p2);
				}
				if (i == 0 || tree[cur].find(cur - smallcols) == tree[cur].end()) {
					pathEdge[p1].push_back(p2);
					pathEdge[p2].push_back(p1);
				}
				if (i == smallrows - 1 || tree[cur].find(cur + smallcols) == tree[cur].end()) {
					pathEdge[p3].push_back(p4);
					pathEdge[p4].push_back(p3);
				}
			}
		}

		// get path's point in order
		int index = robotInitPos.second * bigcols + robotInitPos.first;
		int origin = index;
		//std::cout << "robot init pos : " << index << "\n";
		std::vector<bool> vis(MAX_NODES, false);
		std::vector<int> pathSequence{ index };
		vis[index] = true;
		while (true) {
			bool flag = false;
			for (auto nxt : pathEdge[index]) {
				if (!vis[nxt]) {
					vis[nxt] = true;
					flag = true;
					pathSequence.push_back(nxt);
					index = nxt;
					break;
				}
			}
			if (!flag)	break;
		}

		return pathSequence;
	}

};