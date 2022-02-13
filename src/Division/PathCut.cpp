#include "PathCut.h"

// MST is undirected graph, so it contains (i, j) and (j, i)
void PathCut::MST2Path() {
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
	vector<bool> inPath(Region[0].size() * Region.size(), false);
	int cur = depot[0];
	while (!inPath[depot[0]] || cur != depot[0]) {
		inPath[cur] = true;
		pathSequence.push_back(cur);

		cur = inPath[pathEdge[cur][0]] ? pathEdge[cur][1] : pathEdge[cur][0];
	}
	circleLen = pathSequence.size();

	// 坐标label->顺序label  
	invSequence.resize(Region.size() * Region[0].size(), -1);
	for (int i = 0; i < pathSequence.size(); ++i) {
		invSequence[pathSequence[i]] = i;
	}

	// construct path value vec
	// 假设转一次弯相当于走过2个格子
	// 注意，把第一个和最后一个格子的转弯也纳入了

	pathValue.resize(2 * circleLen, 1.0);
	for (int i = 1; i < 2 * circleLen - 1; ++i) {
		if (!isSameLine(pathSequence[(i - 1 + circleLen) % circleLen], pathSequence[i % circleLen], pathSequence[(i + 1) % circleLen])
			&& i != 2 * circleLen - 1)	 
			pathValue[i] += ONE_TURN_VAL;

		pathValue[i] += pathValue[i - 1];
	}
	pathValue[2 * circleLen - 1] += pathValue[2 * circleLen - 2];

	cout << "Finish Constructing Path from ideal spanning tree.\n";

	// checking path and its value
	/*cout << "display path sequence...\n";
	for (auto i : pathSequence)	cout << i << " ";
	cout << "----------------------------------------------" << endl;
	cout << "display path value...\n";
	for (auto i : pathValue)	cout << i << " ";
	cout << "----------------------------------------------" << endl;*/
}

double PathCut::euclidean_dis(double x1, double y1, double x2, double y2) {
	return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// u和v都是region下的坐标一维编号, 走八邻域时还是得转为二维坐标判断边界
// 八邻域有问题，如果要用必须判断对角线包围情况,当然啦如果以后真用八邻域，dis的数据类型就这样不用改了
// A*可以将转弯数纳入代价
double PathCut::A_star(int u, int v) {
	// 针对原本的图，即region上的A*
	std::priority_queue<node> que;
	int sx = u / bigcols, sy = u % bigcols;
	int ex = u / bigcols, ey = u % bigcols;
	vector<double> dis(bigcols * bigrows, 2e9);
	vector<int> pre(bigcols * bigrows, -1);
	dis[u] = 0;

	que.push({ 0 + euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey), 0, u });
	while (!que.empty()) {
		node cur = que.top();  que.pop();
		if (cur.id == v)	break;
		if (cur.gx - dis[cur.id] > eps)	continue;

		int cx = cur.id / bigcols, cy = cur.id % bigcols;
		for (int i = 0; i < 4; ++i) {
			int dx = cx + dir[i][0], dy = cy + dir[i][1];
			if (dx < 0 || dx >= bigrows || dy < 0 || dy >= bigcols || !Region[dx][dy])	continue;

			int nxt_id = dx * bigcols + dy;
			if (dis[nxt_id] > dis[cur.id] + 1.0) {
				pre[nxt_id] = cur.id;
				dis[nxt_id] = dis[cur.id] + 1.0;
				que.push({ dis[nxt_id] + euclidean_dis(dx, dy, ex, ey), dis[nxt_id], nxt_id });
			}
		}
	}

	int hair = v, head = pre[v], cur;
	double turnVal = 0.0;
	if(head == -1 || head == u)	return dis[v];

	cur = pre[head];
	do {
		if (!isSameLine(hair, head, cur))	turnVal += ONE_TURN_VAL;
		hair = head;
		head = cur;
		cur = pre[cur];
	} while (head != u);

	return dis[v] + turnVal;
}

// 这里可以尝试一下路径压缩A*
vector<int> PathCut::A_star_path(int u, int v) { 
	//cout << "In A star. printing u and v: " << u << " " << v << endl;
	std::priority_queue<node> que;
	int sx = u / bigcols, sy = u % bigcols;
	int ex = v / bigcols, ey = v % bigcols;
	vector<double> dis(bigcols * bigrows, 2e9);
	vector<int> pre(bigcols * bigrows, -1);

	dis[u] = 0;
	//cout << sx << " " << sy << " " << ex << " " << ey << endl;
	//cout << euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey) << endl;
	que.push({ 0 + euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey), 0, u });
	while (!que.empty()) {
		node cur = que.top();  que.pop();
		if (cur.id == v) {
			//cout << "reaching the final point!\n";
			break;
		}
		if (cur.gx - dis[cur.id] > eps)	continue;

		int cx = cur.id / bigcols, cy = cur.id % bigcols;
		//cout << "current coor: " << cx << " " << cy << endl;
		for (int i = 0; i < 4; ++i) {
			int dx = cx + dir[i][0], dy = cy + dir[i][1];
			if (dx < 0 || dx >= bigrows || dy < 0 || dy >= bigcols || !Region[dx][dy])	continue;

			int nxt_id = dx * bigcols + dy;
			if (dis[nxt_id] > dis[cur.id] + 1.0) {
				//cout << "nxt id and nxt coor: " << nxt_id << " " << dx << " " << dy << endl;
				pre[nxt_id] = cur.id;
				dis[nxt_id] = dis[cur.id] + 1.0;
				que.push({ dis[nxt_id] + euclidean_dis(dx, dy, ex, ey), dis[nxt_id], nxt_id });
			}
		}
		//cout << "--------------\n";
	}

	//cout << dis[200] << " " << dis[201] << endl;
	/*for (int i = 0; i < pre.size(); ++i) {
		cout << "No. " << i << " 's point's previous point is " << pre[i] << endl;
	}*/

	//cout << "does it work???" << endl;
	//cout << pre[v] << endl;
	// from tail to head
	vector<int> path;
	int it = v;
	while (it != u) {
		//cout << it << endl;
		path.push_back(it);
		it = pre[it];
	}
	path.push_back(u);
	reverse(path.begin(), path.end());

	// shorten the path ?
	// I don't think it's a good idea. ROS will A-star the path again.
	// over-shorten the path may cause the robot move into inflation layer more often

	return path;
}

double PathCut::getTurnAndLength(int i) {
	// 注意，如果结尾的index小于开头的index，则不是简单尾首相减
	// 另外，严谨一点说，如果cut的最后一个点在全局路径上是转折点的话，这里得-1，当然求前后点的时候注意取模
	int start = cuts[i].start;
	int ending = (cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen;
	double endingTurn = isSameLine(pathSequence[(ending - 1 + circleLen) % circleLen], pathSequence[ending], pathSequence[(ending + 1) % circleLen]) ? 1.0 : 0.0;
	if (start <= ending)
		return pathValue[ending] - pathValue[start] + endingTurn;
	else
		return pathValue[circleLen - 1] - pathValue[start] + pathValue[ending] + endingTurn;
}

double PathCut::updateCutVal(int i) {
	// 计算depot出发，到cut起点 + cut + cut终点到depot的权重
	int cut_start_region_label = pathSequence[cuts[i].start];
	int cut_end_region_label = pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen];

	// return A_star(depot[i], cut_start_region_label) + 1.0 * cuts[i].len + A_star(cut_end_region_label, depot[i]);
	return A_star(depot[cut_depot[i]], cut_start_region_label) + getTurnAndLength(i) + A_star(cut_end_region_label, depot[cut_depot[i]]);
}

// depot是机器人在region下的一维label
void PathCut::MSTC_Star() {
	//cout << "depot's size and cuts' size: " << depot.size() << " " << cuts.size() << "\n";
	// 注意每个机器人的路径初始长度不能简单地根据后减前获取，因为圆上下一个机器人不一定就是下一个序号的机器人
	// 最好是这里在做一层映射，按照圆上的顺序编号，这个编号和robot_init_pos一一对应，最后再映射回去就可以了
	// 最主要需要知道depot label -> circle label(cuts label)
	vector<int> tmp;
	for (auto x : depot) tmp.push_back(invSequence[x]);
	sort(tmp.begin(), tmp.end());
	
	for (int i = 0; i < depot.size(); ++i) {
		for (int j = 0; j < depot.size(); ++j) {
			if (invSequence[depot[i]] == tmp[j]) {
				depot_cut[i] = j;
				cut_depot[j] = i;
			}
		}
	}

	// TODO i改掉
	double opt = 0, wst = 2e9;
	for (int i = 0; i < depot.size(); ++i) {
		cuts[i].start = invSequence[depot[cut_depot[i]]];
		cuts[i].len = (invSequence[depot[cut_depot[(i + 1) % depot.size()]]] - invSequence[depot[cut_depot[i]]] + circleLen) % circleLen; // nr - nl + 1 - 1
		cuts[i].val = updateCutVal(i);
		opt = std::max(opt, cuts[i].val);
		wst = std::min(wst, cuts[i].val);

		//cout << "No." << i << "'s cut's original start, len and val: " << cuts[i].start << " " << cuts[i].len << " " << cuts[i].val << endl;
	}

	// 注意，论文里面这里的跳出条件是最短路径权重大于等于最长路径
	int cur_iter = 0, max_iter = 10;
	//while (cur_iter < max_iter) 
	// infinite loop
	while (opt - wst > 100.0) {
		//cout << "cutting for balancing...\n";     // just a sign
		double minn = 2e9, maxx = -1;
		int min_cut = -1, max_cut = -1;
		for (int i = 0; i < cuts.size(); ++i) {
			if (minn > cuts[i].val) {
				minn = cuts[i].val;
				min_cut = i;
			}
			if (maxx < cuts[i].val) {
				maxx = cuts[i].val;
				max_cut = i;
			}
		}

		/*cout << "min cut and max cut's id: " << min_cut << " " << max_cut << endl;
		cout << "min cut and max cut's length: " << cuts[min_cut].len << " " << cuts[max_cut].len << endl;
		cout << "min cut and max cut's val: " << cuts[min_cut].val << " " << cuts[max_cut].val << endl;*/

		// 判断用逆时针还是顺时针那一段
		vector<int> clw = getHalfCuts(min_cut, max_cut, 1);
		vector<int> ccw = getHalfCuts(min_cut, max_cut, -1);
		clw.size() < ccw.size() ? Balanced_Cut(clw) : Balanced_Cut(ccw);

		// Well, you can't break by simply comparing the longest path.
		/*int cur_opt = std::max_element(cuts.begin(), cuts.end(), [](cut& a, cut& b) { return a.val < b.val; })->val;
		if (cur_opt >= opt + opt / 4) {
			cout << "MSTC_Star Cutoff finished!\n\n\n";
			break;
		}
		else {
			opt = cur_opt;
			cur_iter++;
		}*/
		opt = std::max_element(cuts.begin(), cuts.end(), [](cut& a, cut& b) { return a.val < b.val; })->val;
		wst = std::min_element(cuts.begin(), cuts.end(), [](cut& a, cut& b) { return a.val < b.val; })->val;
	}


}

vector<int> PathCut::getHalfCuts(int cut_min, int cut_max, int dir) {
	vector<int> res;
	int cur_cut = cut_min;
	while (cur_cut != cut_max) {
		res.push_back(cur_cut);
		cur_cut = (cur_cut + dir + depot.size()) % depot.size();
	}
	res.push_back(cut_max);

	if (dir == -1) std::reverse(res.begin(), res.end());   // 作用是最终都是顺时针更新各个段的起点，否则倒着来麻烦

	return res;
}

// 只需要调整cut的起点、长度和权重，类里面的cut vec修改即可
void PathCut::Balanced_Cut(vector<int>& adjustCuts) {
	//cout << "----------------------------------------------\n";
	//cout << "adjusting cut: ";
	/*for (auto i : adjustCuts) cout << i << " ";
	cout << endl;*/

	int r_first = adjustCuts.front(), r_last = adjustCuts.back();
	double old_val_max = -1, old_val_sum = 0;
	for (auto& x : adjustCuts) {
		old_val_max = std::max(old_val_max, cuts[x].val);
		old_val_sum += cuts[x].val;
	}

	pair<double, double> res{ old_val_max, old_val_sum };
	int old_len_r_first = cuts[r_first].len, old_len_r_last = cuts[r_last].len;
	double cur_val_max = -1, cur_val_sum = 0;
	bool update_success = false;
	//int lef = 0, rig = cuts[r_first].len + cuts[r_last].len - 1;  // -1是确保不会出现长度为0的情况
	// MAY BE BUGS!  rig = 1.0 * (cuts[r_first].len + cuts[r_last].len - 1) + getTurnAndLength(r_first) + getTurnAndLength(r_last);
	double lef = 0, rig = getTurnAndLength(r_first) + getTurnAndLength(r_last);
	/*cout << "checking original length for every robot before adjustment...\n";
	for (auto x : cuts) cout << x.len << " ";
	cout << endl;*/

	// 原本只用长度进行二分，但是如果考虑转弯，则是对路径长度+转弯权重进行二分，然后再更新长度
	// 更新长度只能从头开始二分查找路径了
	while (rig - lef > eps) {
		//cout << "haha" << endl;
		double mid = (lef + rig) / 2;

		//cout << "lef and rig and mid: " << lef << " " << rig << " " << mid << endl;
		// 修改len不能和之前一样了，得二分查找下标才行
		// MAY BE BUGS! sth wrong with r_first 这个不是点的下标！！！
		// 这里的二分有问题 毕竟是对一个前缀和数组进行二分
		// mid + getTurnAndLength(r_first)
		// 不能在这个前缀和上二分，因为他不是圆形的，也许要扩大到两倍长度
		// firstCutLen大于两段之和是怎么一回事？
		int firstCutLen = std::lower_bound(pathValue.begin() + cuts[r_first].start, pathValue.end(), mid + pathValue[cuts[r_first].start]) - pathValue.begin() - cuts[r_first].start + 1;
		//cout << "firstCutLen: " << firstCutLen << endl;
		cuts[r_first].len = firstCutLen;
		cuts[r_last].len = old_len_r_first + old_len_r_last - firstCutLen;

		//cout << "adjust len first and last: " << mid << " " << old_len_r_first + old_len_r_last - mid << endl;
		//cout << "lef and rig: " << lef << " " << rig << endl;

		// 首尾的长度更新后，要更新首部以后的段落起始点以及val
		// 这里更新起点是有问题的
		vector<int>::iterator it = adjustCuts.begin();
		while(it != adjustCuts.end()) {
			if(it != adjustCuts.begin())
				cuts[*it].start = (cuts[*(it - 1)].start + cuts[*(it - 1)].len) % circleLen;

			cuts[*it].val = updateCutVal(*it);
			cur_val_max = std::max(cur_val_max, cuts[*it].val);
			cur_val_sum += cuts[*it].val;

			it++;
		}

		if (cur_val_max < old_val_max || (cur_val_max == old_val_max && cur_val_sum < old_val_sum)) {  //
			update_success = true;
			old_val_max = cur_val_max;
			old_val_sum = cur_val_sum;
		}
		
		// checking each cut
		/*for (int i = 0; i < cuts.size(); ++i) {
			cout << "No. " << i << " cut's start, len and val: " << cuts[i].start << " " << cuts[i].len << " " << cuts[i].val << endl;
		}*/


		if (cuts[r_first].val < cuts[r_last].val)	lef = mid + 1;
		else if (cuts[r_first].val > cuts[r_last].val)	rig = mid - 1;
		else break;
	}
	
	/*cout << "checking original length for every robot after adjustment...\n";
	for (auto x : cuts) cout << x.len << " ";
	cout << endl;*/

	if (!update_success) {
		//cout << "Did not find Optimal Cut.\n";
	}
	else {
		//cout << "Found an optimal cut. maximum length = " << old_val_max << " total path length = " << old_val_sum << "\n";
	}
}

void PathCut::get2DCoordinate(int index, int& x, int& y) {
	x = index / smallcols;
	y = index % smallcols;
}

// 根据调整好的cuts，生成最终的路径，每个机器人的路径用的一维坐标，到时候再转换成真实浮点坐标就可以了
// 两段A*生成的路径其实并不必要，因为可以交给ROS的move_base和navigation去做
Mat PathCut::generatePath() {
	Mat path_for_each_robot(depot.size(), vector<int>{});
	for (int i = 0; i < cuts.size(); ++i) {
		//cout << "using A star path...\n";
		//cout << depot[i] << ", " << pathSequence[cuts[i].start] << endl;
		vector<int> p1 = A_star_path(depot[cut_depot[i]], pathSequence[cuts[i].start]);
		//cout << pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen] << ", " << depot[i] << endl;
		vector<int> p2 = A_star_path(pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen], depot[cut_depot[i]]);
		//cout << "A star path ending...\n";
		for (int j = 0; j < cuts[i].len; ++j)	p1.push_back(pathSequence[(cuts[i].start + j) % circleLen]);

		p1.insert(p1.end(), p2.begin(), p2.end());

		path_for_each_robot[i] = p1;
	}

	Mat path_final(depot.size(), vector<int>{});
	for (int i = 0; i < depot.size(); ++i) {
		path_final[i] = path_for_each_robot[cut_depot[i]];
	}

	return path_final;
}

Mat PathCut::cutSolver() {
	cout << "Entering MSTC Cut Solver...\n";
	MST2Path();

	if(depot.size() == 1){
		cout << "Only one robot, no need to balance, return...\n";
		return Mat(1, pathSequence);
	}

	cout << "Begin MSTC_Star Algorithm to find the best cut for each robot...\n";
	MSTC_Star();

	return generatePath();
}

int PathCut::getTurnsNum() {
	int turns = 0;
	for (int i = 1; i < pathSequence.size() - 1; ++i) {
		turns += isSameLine(pathSequence[i - 1], pathSequence[i], pathSequence[i + 1]) ? 0 : 1;
	}

	return turns;
}