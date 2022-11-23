#include <HeuristicPartition.h>

vector<vector<HeuristicSolver::rect>> HeuristicSolver::HeuristicPartition::chessboardPartition() {
	set<int> lineX{ 0, (int)Map.size() + 1 }, lineY{ 0, (int)Map[0].size() + 1 };
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			// if (Map[i][j])	continue;   //筛掉非ob的点

			// Make Flag
			int flag = 0;
			for (int k = 0; k < 4; ++k) {
				int di = i + dir[k][0], dj = j + dir[k][1];
				if (di < 0 || dj < 0 || di >= Map.size() || dj >= Map[0].size() || Map[di][dj] != Map[i][j])	continue;
 
				flag |= (1 << k);
			}

			if ((flag & 0x03) == 0x03) {
				lineY.insert(j + 1);
				lineX.insert(i);
			}
			if ((flag & 0x06) == 0x06) {
				lineY.insert(j);
				lineX.insert(i);
			}
			if ((flag & 0x0c) == 0x0c) {
				lineY.insert(j);
				lineX.insert(i + 1);
			}
			if ((flag & 0x09) == 0x09) {
				lineY.insert(j + 1);
				lineX.insert(i + 1);
			}
		}
	}

	for (auto x : lineX)	X.push_back(x);
	for (auto y : lineY)	Y.push_back(y);

	// discretize
	vector<vector<rect> > rec(X.size() - 1, vector<rect>(Y.size() - 1));
	for (int i = 1; i < X.size(); ++i)	height.push_back(X[i] - X[i - 1]);
	for (int i = 1; i < Y.size(); ++i)	width.push_back(Y[i] - Y[i - 1]);

	int x = 0, y;
	for (int i = 0; i < X.size() - 1; ++i) {
		y = 0;
		for (int j = 0; j < Y.size() - 1; ++j) {
			//cout << x << " " << y << endl;
			rec[i][j].empty = Map[x][y] == 0 ? true : false;
			rec[i][j].coordinate.first = x;
			rec[i][j].coordinate.second = y;
			y += width[j];
		}
		x += height[i];
	}

#ifdef DEBUG
	// check boundary
	cout << "X: ";
	for(auto row : X)
		cout << row << " ";
	cout << "\nY: ";
	for(auto col : Y)
		cout << col << " ";
	cout << "\n";

	// check partition
	for(int i = 0; i < rec.size(); ++i){
		for(int j = 0; j < rec[0].size(); ++j){
			if(!rec[i][j].empty){
				cout << "(" << rec[i][j].coordinate.first << ", " << rec[i][j].coordinate.second << ")";
			}
		}
	}
#endif

	cout << "\nchessboard partition successfully.\n";

	return rec;
}

vector<vector<HeuristicSolver::rect>> HeuristicSolver::HeuristicPartition::orientRectangle(vector<vector<rect>>& rec) {
	vector<vector<rect>> minRanksRec;
	vector<P> unvisRec;
	vector<vector<bool>> vis(rec.size(), vector<bool>(rec[0].size(), false));
	int totalRanks = 0x3f3f3f3f;
	std::random_device rd;
    std::mt19937 generator(rd());

	for (int iter = 0; iter < max_iter; ++iter) {
		// randomize rec's orientation
		unvisRec = {};
		for (int i = 0; i < rec.size(); ++i) {
			for (int j = 0; j < rec[0].size(); ++j) {
				if (!rec[i][j].empty) {
					rec[i][j].dir = generator() % 2;
					unvisRec.push_back( { i, j });
				}
			}
		}

		//randomly and locally minimize ranks
		while (1) {
			bool bias = generator() % 2;
			bool improvement = false;
			while (unvisRec.size()) {
				int id = generator() % unvisRec.size();
				int x = unvisRec[id].first, y = unvisRec[id].second;
				unvisRec.erase(unvisRec.begin() + id);
				vis[x][y] = true;

				// find best orientation for current rectangle, need to check both two directions
				// hRank : 假设当前矩阵方向是水平的，计算本身及四领域的rank和，vRank与之相反
				int hRank = height[y], vRank = width[x];
				if (x > 0) {
					hRank += rec[x - 1][y].dir == HORIZONTAL ? height[x - 1] : width[y];
					vRank += rec[x - 1][y].dir == VERTICAL ? 0 : height[x - 1];
				}
				if (x < rec.size() - 1) {
					hRank += rec[x + 1][y].dir == HORIZONTAL ? height[x + 1] : width[y];
					vRank += rec[x + 1][y].dir == VERTICAL ? 0 : height[x + 1];
				}
				if (y > 0) {
					hRank += rec[x][y - 1].dir == HORIZONTAL ? 0 : width[y - 1];
					vRank += rec[x][y - 1].dir == VERTICAL ? width[y - 1] : height[x];
				}
				if (y < rec[0].size() - 1) {
					hRank += rec[x][y + 1].dir == HORIZONTAL ? 0 : width[y + 1];
					vRank += rec[x][y + 1].dir == VERTICAL ? width[y + 1] : height[x];
				}

				// 分类讨论两种情况
				if (hRank == vRank && bias != rec[x][y].dir) {
					rec[x][y].dir ^= 1;
					if (x > 0 && vis[x - 1][y]) {
						vis[x - 1][y] = false;
						unvisRec.push_back({ x - 1, y });
					}
					if (x < rec.size() - 1 && vis[x + 1][y]) {
						vis[x + 1][y] = false;
						unvisRec.push_back({ x + 1, y });
					}
					if (y > 0 && vis[x][y - 1]) {
						vis[x][y - 1] = false;
						unvisRec.push_back({ x, y - 1 });
					}
					if (y < rec[0].size() - 1 && vis[x][y + 1]) {
						vis[x][y + 1] = false;
						unvisRec.push_back({ x, y + 1 });
					}
				}
				else if ((hRank < vRank && rec[x][y].dir == VERTICAL) || (vRank < hRank && rec[x][y].dir == HORIZONTAL)) {
					rec[x][y].dir ^= 1;
					if (x > 0 && vis[x - 1][y]) {
						vis[x - 1][y] = false;
						unvisRec.push_back({ x - 1, y });
					}
					if (x < rec.size() - 1 && vis[x + 1][y]) {
						vis[x + 1][y] = false;
						unvisRec.push_back({ x + 1, y });
					}
					if (y > 0 && vis[x][y - 1]) {
						vis[x][y - 1] = false;
						unvisRec.push_back({ x, y - 1 });
					}
					if (y < rec[0].size() - 1 && vis[x][y + 1]) {
						vis[x][y + 1] = false;
						unvisRec.push_back({ x, y + 1 });
					}
					improvement = true;
				}

			} // end while (unvisRec.size())
			if (improvement)	bias ^= 1;
			else	break;
		} // end while(1)

		// calculate final ranks and update current rectangle with minimun ranks
		fill(vis.begin(), vis.end(), vector<bool>(rec[0].size(), false));

		int curRanks = 0;
		for (int i = 0; i < rec.size(); ++i) {
			for (int j = 0; j < rec[0].size(); ++j) {
				if (rec[i][j].empty || vis[i][j])	continue;
				
				if (rec[i][j].dir == HORIZONTAL) {
					curRanks += height[i];
					for (int k = j; k < rec[0].size() && rec[i][k].dir == HORIZONTAL; ++k)	vis[i][k] = true;
				}
				else {
					curRanks += width[j];
					for (int k = i; k < rec.size() && rec[k][j].dir == VERTICAL; ++k)	vis[k][j] = true;
				}
			}
		}
		if (curRanks < totalRanks) {
			totalRanks = curRanks;
			minRanksRec = rec;
		}

#ifdef DEBUG
		if(i % 100 == 0)	cout << "No. " << i << "iter's totalranks: " << totalRanks << "\n";
#endif

	} // end for iter

	cout << "iteration: " << max_iter << endl;
	cout << "minimum ranks: " << totalRanks << endl;

	return minRanksRec;
}

void HeuristicSolver::HeuristicPartition::minrects2ranks(){
    vector<vector<char>> directionMap(Map.size(), vector<char>(Map[0].size(), 'N'));
	
    for(int i = 0; i < minRanksRec.size(); ++i){
        for(int j = 0; j < minRanksRec[0].size(); ++j){
            if(!minRanksRec[i][j].empty){
                int row = minRanksRec[i][j].coordinate.first;
                int col = minRanksRec[i][j].coordinate.second;
                for(int dr = 0; dr < height[i]; ++dr){
                    for(int dc = 0; dc < width[j]; ++dc){
                        directionMap[row + dr][col + dc] = minRanksRec[i][j].dir == HORIZONTAL ? 'H' : 'V';
                    }
                }
            }
        }
    }

	// check direction of each cell
#ifdef DEBUG
	for(int i = 0; i < Map.size(); ++i){
		for(int j = 0; j < Map[0].size(); ++j){
			char c = directionMap[i][j] == 'V' ? '|' : '-';
			if(directionMap[i][j] == 'N')	c = 'N';
			cout << c << " ";
		}
		cout << "\n";
	}
#endif

    for(int i = 0; i < Map.size(); ++i){
        for(int j = 0; j < Map[0].size(); ++j){
            if(directionMap[i][j] == 'H'){
                if(j > 0 && directionMap[i][j - 1] == 'H'){
                    ranks[RESHAPE(i, j)].push_back(RESHAPE(i, j - 1));
                    uf.unite(RESHAPE(i, j), RESHAPE(i, j - 1));
                }
                if(j < Map[0].size() - 1 && directionMap[i][j + 1] == 'H'){
                    ranks[RESHAPE(i, j)].push_back(RESHAPE(i, j + 1));
                    uf.unite(RESHAPE(i, j), RESHAPE(i, j + 1));
                }
            } else if(directionMap[i][j] == 'V'){
                if(i > 0 && directionMap[i - 1][j] == 'V'){
                    ranks[RESHAPE(i, j)].push_back(RESHAPE(i - 1, j));
                    uf.unite(RESHAPE(i, j), RESHAPE(i - 1, j));
                }
                if(i < Map.size() - 1 && directionMap[i + 1][j] == 'V'){
                    ranks[RESHAPE(i, j)].push_back(RESHAPE(i + 1, j));
                    uf.unite(RESHAPE(i, j), RESHAPE(i + 1, j));
                }
            }
        }
    }
}

void HeuristicSolver::HeuristicPartition::mergeRanks(Mat& graph){
    vector<edge> edges;   //双向边
	priority_queue<edge> que;
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			if (j < Map[0].size() - 1) 
				if (Map[i][j] && Map[i][j + 1] && !uf.same(RESHAPE(i, j), RESHAPE(i, j + 1)))
                	edges.push_back({ RESHAPE(i, j), RESHAPE(i, j + 1), 0 });
			
			if (i < Map.size() - 1) 
				if (Map[i][j] && Map[i + 1][j] && !uf.same(RESHAPE(i, j), RESHAPE(i + 1, j)))
                	edges.push_back({ RESHAPE(i, j), RESHAPE(i + 1, j), 0 });
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

		if (uf.same(v1, v2))	continue;

		int curVal = getEdgeVal(graph, v1, v2);
		if (curEdge.cost != curVal) {
			curEdge.cost = curVal;
			que.push(curEdge);
			continue;
		}

		// 可以继续优化，随着MST连接，某些边的cost会发生变化，可以多用一个heap维护之
		graph[v1].push_back(v2);
		graph[v2].push_back(v1);
		uf.unite(v1, v2);

		//cnt++;
	}

}

HeuristicSolver::Mat HeuristicSolver::HeuristicPartition::hpSolver(bool merge){
    vector<vector<rect>> rects = chessboardPartition();
    this->minRanksRec = orientRectangle(rects);
	minrects2ranks();
	if(merge){
		mergeRanks(ranks);
		int totalTurns = 0;
		for (int i = 0; i < ranks.size(); ++i) {
			totalTurns += getVertexVal(ranks, i);
		}
		cout << "rect dinic MST number of turns: " << totalTurns << endl;

		checkMST(ranks, Map);
	}

	return ranks;
}
