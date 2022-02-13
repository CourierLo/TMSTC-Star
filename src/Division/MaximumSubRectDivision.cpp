#include "MaximumSubRectDivision.h"

vector<rect> Division::findMaximumSubRect(Mat& Map) {
	vector<int> lef(Map[0].size(), 0), rig(Map[0].size(), 0);
	stack<P> sta;
	Mat preSum(Map.size(), vector<int>(Map[0].size(), 0));

	// construct prefix sum 2D array
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			if (!i)	preSum[i][j] = Map[i][j] ? 1 : 0;
			else preSum[i][j] = Map[i][j] ? preSum[i - 1][j] + 1 : 0;
		}
	}

	// scan every row and construct left and right height stack
	rect ret{ {0, 0}, 0, 0, 0 };
	vector<rect> retvec;
	for (int i = 0; i < Map.size(); ++i) {
		// right
		for (int j = 0; j < Map[0].size(); ++j) {
			while (!sta.empty() && preSum[i][j] < sta.top().first) {
				rig[sta.top().second] = j - 1;
				sta.pop();
			}

			if (sta.empty() || preSum[i][j] >= sta.top().first)
				sta.push({ preSum[i][j], j });
		}
		if (!sta.empty()) {
			int rightmost = sta.top().second;
			while (!sta.empty()) {
				rig[sta.top().second] = rightmost;
				sta.pop();
			}
		}

		// left
		for (int j = Map[0].size() - 1; j >= 0; --j) {
			while (!sta.empty() && preSum[i][j] < sta.top().first) {
				lef[sta.top().second] = j + 1;
				sta.pop();
			}

			if (sta.empty() || preSum[i][j] >= sta.top().first)
				sta.push({ preSum[i][j], j });
		}
		if (!sta.empty()) {
			int leftmost = sta.top().second;
			while (!sta.empty()) {
				lef[sta.top().second] = leftmost;
				sta.pop();
			}
		}

		// calculate current row's maximum sub rectangle
		for (int j = 0; j < Map[0].size(); ++j) {
			int area = (rig[j] - lef[j] + 1) * preSum[i][j];
			// 需要处理面积最大为1的情况
			if (area > ret.height * ret.width && area != 1) {
				ret.height = preSum[i][j];
				ret.width = rig[j] - lef[j] + 1;
				ret.corner.first = i - preSum[i][j] + 1;
				ret.corner.second = lef[j];
			}
			// 顺便处理掉孤立的点
			if (area == 1) {
				bool flag = (i > 0 && Map[i - 1][j]) | (i < Map.size() - 1 && Map[i + 1][j]) | (j > 0 && Map[i][j - 1]) | (j < Map[0].size() - 1 && Map[i][j + 1]);
				if (!flag) {
					retvec.push_back({ { i, j }, 1, 1, 0 });
					//cout << i << " " << j << endl;
					Map[i][j] = 0;
				}
			}
		}
	}

	if(ret.height * ret.width != 0)
		retvec.push_back(ret);

	return retvec;
}

//int edgesNum = 0;
// 最好在外面复制一份map再传进来
vector<rect> Division::maximumSubRectDivision(Mat& Map) {
	vector<rect> ret;
	int numVertex = 0;
	for (int i = 0; i < Map.size(); ++i)
		for (int j = 0; j < Map[0].size(); ++j)
			numVertex += Map[i][j] ? 1 : 0;
	//cout << numVertex << endl;

	while (numVertex) {
		vector<rect> tmp = findMaximumSubRect(Map);
		//cout << "ret's number: " << tmp.size() << endl;
		for (int i = 0; i < tmp.size(); ++i) {
			numVertex -= tmp[i].height * tmp[i].width;

			if (tmp[i].height > tmp[i].width)	tmp[i].dir = VERTICAL;
			else if (tmp[i].height < tmp[i].width)	tmp[i].dir = HORIZONTAL;
			else  tmp[i].dir = BOTH_ORI;

			//cout << "(" << tmp[i].corner.first << ", " << tmp[i].corner.second << ")  h: " << tmp[i].height << " w: " << tmp[i].width << endl;
			//cout << "numVertex left:" <<  numVertex << endl;

			for (int row = tmp[i].corner.first; row < tmp[i].corner.first + tmp[i].height; ++row)
			for (int col = tmp[i].corner.second; col < tmp[i].corner.second + tmp[i].width; ++col)
				Map[row][col] = 0;

			ret.push_back(tmp[i]);
		}
	}
	//cout << "end division!" << endl;

	return ret;
}

void Division::orientRect(vector<rect>& rectVec) {
	// 记录每个栅格点所属的矩形编号，以便快速查找朝向
	Mat label(Map.size(), vector<int>(Map[0].size(), 0));
	for (int i = 0; i < rectVec.size(); ++i) {
		for (int row = rectVec[i].corner.first; row < rectVec[i].corner.first + rectVec[i].height; ++row) {
			for (int col = rectVec[i].corner.second; col < rectVec[i].corner.second + rectVec[i].width; ++col) {
				label[row][col] = i;
			}
		}
	}

	//TODO : 长宽一致的矩形，继续优化之
	//TODO : 如果所有矩形长宽一致，另行处理
	bool improvement;
	set<int> verticalRectSet, horizontalRectSet;
	while (true) {
		improvement = false;
		for (int i = 0; i < rectVec.size(); ++i) {
			if (rectVec[i].dir == BOTH_ORI)	continue;

			int verticalCost = 0, horizontalCost = 0;
			verticalRectSet.clear();  horizontalRectSet.clear();
			// if rectVec[i] is VERTICAL
			for (int col = rectVec[i].corner.second; col < rectVec[i].corner.second + rectVec[i].width; ++col) {
				// 处理地图边界以及障碍物包围的情况 注意加一减一
				int upper = rectVec[i].corner.first - 1, lower = rectVec[i].corner.first + rectVec[i].height;
				if (upper >= 0 && Map[upper][col] && lower < Map.size() && Map[lower][col]) {
					if (rectVec[label[upper][col]].dir != HORIZONTAL && rectVec[label[lower][col]].dir != HORIZONTAL) {
						verticalCost--;
						if (rectVec[label[upper][col]].dir == BOTH_ORI)	verticalRectSet.insert(label[upper][col]);
						if (rectVec[label[lower][col]].dir == BOTH_ORI)	verticalRectSet.insert(label[lower][col]);
					}
					else if (rectVec[label[upper][col]].dir == HORIZONTAL && rectVec[label[lower][col]].dir == HORIZONTAL)	verticalCost++;
				}
				else if (upper >= 0 && Map[upper][col]) {
					if (rectVec[label[upper][col]].dir == HORIZONTAL)		verticalCost++;
					else if (rectVec[label[upper][col]].dir == BOTH_ORI)	verticalRectSet.insert(label[upper][col]);
				}
				else if (lower < Map.size() && Map[lower][col]) {
					if (rectVec[label[lower][col]].dir == HORIZONTAL)		verticalCost++;
					else if (rectVec[label[lower][col]].dir == BOTH_ORI)	verticalRectSet.insert(label[lower][col]);
				}
				else  verticalCost++;
			}

			// if rectVec[i] is HORIZONTAL
			for (int row = rectVec[i].corner.first; row < rectVec[i].corner.first + rectVec[i].height; ++row) {
				int lef = rectVec[i].corner.second - 1, rig = rectVec[i].corner.second + rectVec[i].width;
				if (lef >= 0 && Map[row][lef] && rig < Map[0].size() && Map[row][rig]) {
					if (rectVec[label[row][lef]].dir != VERTICAL && rectVec[label[row][rig]].dir != VERTICAL) {
						horizontalCost--;
						if (rectVec[label[row][lef]].dir == BOTH_ORI)	horizontalRectSet.insert(label[row][lef]);
						if (rectVec[label[row][rig]].dir == BOTH_ORI)	horizontalRectSet.insert(label[row][rig]);
					}
					else if (rectVec[label[row][lef]].dir == VERTICAL && rectVec[label[row][rig]].dir == VERTICAL)	horizontalCost++;
				}
				else if (lef >= 0 && Map[row][lef]) {
					if (rectVec[label[row][lef]].dir == VERTICAL)		horizontalCost++;
					else if (rectVec[label[row][lef]].dir == BOTH_ORI)	horizontalRectSet.insert(label[row][lef]);
				}
				else if (rig < Map[0].size() && Map[row][rig]) {
					if (rectVec[label[row][rig]].dir == VERTICAL)		horizontalCost++;
					else if (rectVec[label[row][rig]].dir == BOTH_ORI)	horizontalRectSet.insert(label[row][rig]);
				}
				else  horizontalCost++;
			}

			//cout << "No. " << i << " rect's "<< "orientation cost-> vertical: " << verticalCost << " horizontal: " << horizontalCost << endl;

			// 定下来方向后，周围没有方向的也要保持一致
			int newDir = verticalCost < horizontalCost ? VERTICAL : HORIZONTAL;
			if (newDir != rectVec[i].dir) {
				rectVec[i].dir = newDir;
				if (newDir == HORIZONTAL)
					for (auto id : horizontalRectSet)	rectVec[id].dir = HORIZONTAL;
				else 
					for (auto id : verticalRectSet)		rectVec[id].dir = VERTICAL;

				improvement = true;
			}
		}

		if (!improvement)	break;
	}
}

/*
	另一种做法：
	不构造MST了，只生成横的或者竖的边，然后每次用cost最小的边去连接这些条条就可以
	由于在连接过程中边的cost值会发生变化，所以用个堆去维护边集，直到生成一个最大MST，算法结束
*/

// 贪心构造每个矩形对应的最优MST，并且构造对应的并查集
void Division::constructMST(vector<rect>& rectVec) {
	// initialize u&f tree and graph
	int tot = Map.size() * Map[0].size();
	fa.resize(tot + 5);
	for (int i = 0; i < tot; ++i)	fa[i] = i;
	MST.resize(tot, vector<int>{});
	//Mat graph(tot, vector<int>{});

	for (int i = 0; i < rectVec.size(); ++i) {
		// 只有一点无需加边
		if (rectVec[i].width * rectVec[i].height == 1) continue;

		if (rectVec[i].dir == VERTICAL) {
			for (int row = rectVec[i].corner.first; row < rectVec[i].corner.first + rectVec[i].height - 1; ++row) {
				for (int col = rectVec[i].corner.second; col < rectVec[i].corner.second + rectVec[i].width; ++col) {
					//cout << "*** " << "(" << row << ", " << col << ")" << " *** " << reshape(row, col) << " " << reshape(row + 1, col) << endl;
					MST[reshape(row, col)].push_back(reshape(row + 1, col));
					MST[reshape(row + 1, col)].push_back(reshape(row, col));
					unite(reshape(row, col), reshape(row + 1, col));
				}
			}
		}
		else {
			for (int row = rectVec[i].corner.first; row < rectVec[i].corner.first + rectVec[i].height; ++row) {
				for (int col = rectVec[i].corner.second; col < rectVec[i].corner.second + rectVec[i].width - 1; ++col) {
					MST[reshape(row, col)].push_back(reshape(row, col + 1));
					MST[reshape(row, col + 1)].push_back(reshape(row, col));
					unite(reshape(row, col), reshape(row, col + 1));
				}
			}
		}
	}

	//return graph;
}


// (a + b) / 2 = c means three points lie in the same line
// 2n条边，算出每条边的增量，然后从小到大排序，并查集合并一下即可
void Division::mergeMST(Mat& graph) {
	vector<edge> edges;   //双向边
	priority_queue<edge> que;
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			if (j < Map[0].size() - 1) 
				if (!same(reshape(i, j), reshape(i, j + 1)) && Map[i][j] && Map[i][j + 1])	edges.push_back({ reshape(i, j), reshape(i, j + 1), 0 });
			
			if (i < Map.size() - 1) 
				if (!same(reshape(i, j), reshape(i + 1, j)) && Map[i][j] && Map[i + 1][j])	edges.push_back({ reshape(i, j), reshape(i + 1, j), 0 });
		}
	}

	for (int i = 0; i < edges.size(); ++i) {
		// 求每条边带来的增量
		// 度为零，增量为0；度为1，增量-2或0；度为2，增量0或2；度为3，增量2；度为4不可连接。 算边的两端的增量
		edges[i].cost = getEdgeVal(graph, edges[i].from, edges[i].to);
		que.push(edges[i]);
	}

	//sort(edge.begin(), edge.end());
	// 当然可以提前结束
	//int cnt = 0;
	while (!que.empty()) {
		edge curEdge = que.top();  que.pop();
		int v1 = curEdge.from, v2 = curEdge.to;

		if (same(v1, v2))	continue;

		int curVal = getEdgeVal(graph, v1, v2);
		if (curEdge.cost != curVal) {
			curEdge.cost = curVal;
			que.push(curEdge);
			continue;
		}

		// 可以继续优化，随着MST连接，某些边的cost会发生变化，可以多用一个heap维护之
		graph[v1].push_back(v2);
		graph[v2].push_back(v1);
		unite(v1, v2);

		//cnt++;
	}

}

bool Division::checkMST(Mat& graph, Mat& Map) {
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

Mat Division::rectDivisionSolver() {
	clock_t start, finish;
	start = clock();
	Mat copyMap = Map;

	vector<rect> ans = maximumSubRectDivision(copyMap);

	// test
	sort(ans.begin(), ans.end());

	orientRect(ans);
	constructMST(ans);
	mergeMST(MST);

	finish = clock();
	cout << "rect division solver used time: " << finish - start << endl;

	// calculate the number of turns
	int totalTurns = 0;
	for (int i = 0; i < MST.size(); ++i) {
		totalTurns += getVertexVal(MST, i);
	}
	cout << "rect division solver's number of turns: " << totalTurns << endl;

	checkMST(MST, Map);

	cout << "-------------------RectDiv Solver End-------------------" << endl;

	return MST;
}

#define IS_VALID(x, y) (x >= 0 && x < Map.size() && y >= 0 && y < Map[0].size() && Map[x][y] && !vis[reshape(x, y)])
stack<P> stk;  //防止爆栈，别问我为什么不用递归
vector<bool> vis;
Mat Division::dfsWithStackSolver(unsigned char dfsDir) {
	clock_t start, finish;
	start = clock();

	Mat dfsGraph(Map.size() * Map[0].size(), vector<int>{});
	vis.resize(Map.size() * Map[0].size());
	fill(vis.begin(), vis.end(), false);
	P vertex = { -1, -1 };

	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			if (Map[i][j]) {
				vertex.first = i;  vertex.second = j;
				break;
			}
		}
	}

	//cout << (int)dfsDir << endl;
	//cout << stk.size() << endl;
	for (int i = 3; i >= 0; --i) {
		int dx = vertex.first + (dfsDir == VERTICAL ? dir[i][0] : dir2[i][0]);
		int dy = vertex.second + (dfsDir == VERTICAL ? dir[i][1] : dir2[i][1]);
		if (IS_VALID(dx, dy))	stk.push({ reshape(vertex.first, vertex.second), reshape(dx, dy) });
	}

	while (!stk.empty()) {
		P curEdge = stk.top();  stk.pop();
		if (vis[curEdge.second])	continue;

		dfsGraph[curEdge.first].push_back(curEdge.second);
		dfsGraph[curEdge.second].push_back(curEdge.first);
		vis[curEdge.first] = true;
		vis[curEdge.second] = true;

		for (int i = 3; i >= 0; --i) {
			// curEdge.second turn to 2D coordinate
			int x = curEdge.second / Map[0].size();
			int y = curEdge.second % Map[0].size();
			int dx = x + (dfsDir == VERTICAL ? dir[i][0] : dir2[i][0]);
			int dy = y + (dfsDir == VERTICAL ? dir[i][1] : dir2[i][1]);
			if (IS_VALID(dx, dy)) {
				stk.push({ curEdge.second, reshape(dx, dy) });
			}
		}
	}

	finish = clock();
	if (dfsDir == VERTICAL) cout << "vertical ";
	else cout << "horizontal ";
	cout << "dfs used time: " << finish - start << endl;

	int totalTurns = 0;
	for (int i = 0; i < dfsGraph.size(); ++i) {
		totalTurns += getVertexVal(dfsGraph, i);
	}

	if (dfsDir == VERTICAL) cout << "vertical ";
	else cout << "horizontal ";
	cout << "dfs' number of turns: " << totalTurns << endl;

	checkMST(dfsGraph, Map);

	/*for (int i = 0; i < dfsGraph.size(); ++i) {
		if (dfsGraph[i].size() == 0)		continue;
		cout << "No. " << i << ": ";
		for (auto id : dfsGraph[i])	cout << id << " ";
		cout << endl;
	}*/

	cout << "-----------------DFS MST Solver End---------------------" << endl;

	return dfsGraph;
}

Mat Division::kruskalSolver() {
	clock_t start, finish;
	start = clock();

	vector<P> edges;   // costs all equal to 1
	int tot = Map.size() * Map[0].size();
	fa.resize(tot + 5);
	for (int i = 0; i < tot; ++i)  fa[i] = i;

	Mat kruskalGraph(tot, vector<int>{});
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			if (!Map[i][j])  continue;

			if (i + 1 < Map.size() && Map[i + 1][j])	edges.push_back({ reshape(i, j), reshape(i + 1, j) });
			if (j + 1 < Map[0].size() && Map[i][j + 1])	edges.push_back({ reshape(i, j), reshape(i, j + 1) });
		}
	}

	for (int i = 0; i < edges.size(); ++i) {
		int from = edges[i].first, to = edges[i].second;
		if (!same(from, to)) {
			kruskalGraph[from].push_back(to);
			kruskalGraph[to].push_back(from);
			unite(from, to);
		}
	}

	finish = clock();
	cout << "kruskal solver used time: " << finish - start << endl;

	int totalTurns = 0;
	for (int i = 0; i < kruskalGraph.size(); ++i) {
		totalTurns += getVertexVal(kruskalGraph, i);
	}

	cout << "kruskal' number of turns: " << totalTurns << endl;

	checkMST(kruskalGraph, Map);

	cout << "------------------Kruskal Solver End--------------------" << endl;

	return kruskalGraph;
}
