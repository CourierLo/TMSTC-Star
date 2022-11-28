#include "PathCut.h"

// MST is undirected graph, so it contains (i, j) and (j, i)
void PathCut::MST2Path() {
	vector<unordered_set<int>> vis(Map.size() * Map[0].size(), std::unordered_set<int>{});
	pathEdge.resize(bigcols * bigrows, vector<int>{});

	// ROS���Ǻ�x��y��������·����ŵ�ʱ���ע��һ��, bigcols��Ӧ����mapServer��ͼ��x
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

	cout << "Generating: Get path edges\n";

	// �õ�pathEdge�󣬽�һ���õ�·������pathSequence�����䷴���У�
	// pathSequnce�ӵ�һ�������˵�λ�ÿ�ʼ
	// pathSequence�ĳ��Ⱦ���Բ�ĳ��ȣ�invSequence����
	vector<bool> inPath(Region[0].size() * Region.size(), false);
	int cur = depot[0];
	while (!inPath[depot[0]] || cur != depot[0]) {
		inPath[cur] = true;
		pathSequence.push_back(cur);

		//cout << cur << " ";
		if(pathEdge[cur].size() == 0)	cout << "Generating: Edge set crash\n";

		cur = inPath[pathEdge[cur][0]] ? pathEdge[cur][1] : pathEdge[cur][0];
		if(inPath[cur])	break;
	}
	circleLen = pathSequence.size();

	cout << "Generating: Trim labels\n";
	// ����label->˳��label  
	invSequence.resize(Region.size() * Region[0].size(), -1);
	for (int i = 0; i < pathSequence.size(); ++i) {
		invSequence[pathSequence[i]] = i;
	}

	// construct path value vec
	// ����תһ�����൱���߹�2������
	// ע�⣬�ѵ�һ�������һ�����ӵ�ת��Ҳ������

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

// u��v����region�µ�����һά���, �߰�����ʱ���ǵ�תΪ��ά�����жϱ߽�
// �����������⣬���Ҫ�ñ����ж϶Խ��߰�Χ���,��Ȼ������Ժ����ð�����dis���������;��������ø���
// A*���Խ�ת�����������
double PathCut::A_star(int u, int v) {
	// ���ԭ����ͼ����region�ϵ�A*
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

// ������Գ���һ��·��ѹ��A*
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
	// ע�⣬�����β��indexС�ڿ�ͷ��index�����Ǽ�β�����
	// ���⣬�Ͻ�һ��˵�����cut�����һ������ȫ��·������ת�۵�Ļ��������-1����Ȼ��ǰ����ʱ��ע��ȡģ
	int start = cuts[i].start;
	int ending = (cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen;
	double endingTurn = isSameLine(pathSequence[(ending - 1 + circleLen) % circleLen], pathSequence[ending], pathSequence[(ending + 1) % circleLen]) ? 1.0 : 0.0;
	if (start <= ending)
		return pathValue[ending] - pathValue[start] + endingTurn;
	else
		return pathValue[circleLen - 1] - pathValue[start] + pathValue[ending] + endingTurn;
}

double PathCut::estimatePathCost(nav_msgs::Path& path){
	double res = 0;
	double x1, x2, x3, y1, y2, y3;
	for(int i = 1; i < path.poses.size(); ++i){
		x1 = path.poses[i].pose.position.x, x2 = path.poses[i - 1].pose.position.x;
		y1 = path.poses[i].pose.position.y, y2 = path.poses[i - 1].pose.position.y;
		res += sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
		
		// if(i > 1){
		// 	x3 = path.poses[i - 2].pose.position.x, y3 = path.poses[i - 2].pose.position.y;
		// 	pair<double, double> vec1 = { x2 - x1, y2 - y1 };
		// 	pair<double, double> vec2 = { x3 - x1, y3 - y1 };
		// 	double angle = atan2(vec2.second, vec2.first) - atan2(vec1.second, vec1.first);
		// 	res += fabs(4.0 * angle) / PI * ONE_TURN_VAL / 3;
		// }
	}
	//res += ONE_TURN_VAL;

	return res;
}

double PathCut::ROSGlobalPlannerCost(int start_id, int goal_id){
	static int seqno;
	double res = globalCostmap->info.resolution * 10.0;
	double start_x = (start_id % bigcols) * res + res / 2 + globalCostmap->info.origin.position.x;
	double start_y = (start_id / bigcols) * res + res / 2 + globalCostmap->info.origin.position.y;
	double goal_x = (goal_id % bigcols) * res + res / 2 + globalCostmap->info.origin.position.x;
	double goal_y = (goal_id / bigcols) * res + res / 2 + globalCostmap->info.origin.position.y;

	//geometry_msgs::Quaternion q();
	float tolerance = 0.0;

	geometry_msgs::PoseStamped start_pose;
	start_pose.header.frame_id = "map";
	start_pose.header.stamp = ros::Time::now();
	start_pose.header.seq = seqno;
	start_pose.pose.position.x = start_x;
	start_pose.pose.position.y = start_y;
	start_pose.pose.orientation.w = 1;
	
	geometry_msgs::PoseStamped goal_pose;
	goal_pose.header.frame_id = "map";
	goal_pose.header.stamp = ros::Time::now();
	goal_pose.header.seq = seqno++;
	goal_pose.pose.position.x = goal_x;
	goal_pose.pose.position.y = goal_y;
	goal_pose.pose.orientation.w = 1;

	ros::ServiceClient client = nh->serviceClient<nav_msgs::GetPlan>("/move_base1/make_plan", true);
	nav_msgs::GetPlan getPlanSrv;
	getPlanSrv.request.start = start_pose;
	getPlanSrv.request.goal = goal_pose;
	getPlanSrv.request.tolerance = tolerance;

	if(client.call(getPlanSrv)){
		return estimatePathCost(getPlanSrv.response.plan);
	} else {
		ROS_ERROR("Failed to get ROS planning path. Can't generate path cost!");
		return 0;
	}
}

double PathCut::updateCutVal(int i) {
	// ����depot��������cut��� + cut + cut�յ㵽depot��Ȩ��
	int cut_start_region_label = pathSequence[cuts[i].start];
	int cut_end_region_label = pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen];

	// cover without back to starting point
	if(!coverAndReturn){
		if(!useROSPlanner)
			return 0.5 * A_star(depot[cut_depot[i]], cut_start_region_label) + getTurnAndLength(i);   //rough A*
		else{
			// precise Navfn planning path value. Use navfn/MakeNavPlan Service
			return ROSGlobalPlannerCost(depot[cut_depot[i]], cut_start_region_label) + getTurnAndLength(i);
		}
	} else {
		if(!useROSPlanner)
			return 0.5 * A_star(depot[cut_depot[i]], cut_start_region_label) + getTurnAndLength(i) + 0.5 * A_star(cut_end_region_label, depot[cut_depot[i]]); //rough A*
		else{
			// precise Navfn planning path value. Use navfn/MakeNavPlan Service
			return 1.1 * ROSGlobalPlannerCost(depot[cut_depot[i]], cut_start_region_label) + getTurnAndLength(i) + 1.1 * ROSGlobalPlannerCost(cut_end_region_label, depot[cut_depot[i]]);
		}
	}
}

// depot�ǻ�������region�µ�һάlabel
void PathCut::MSTC_Star() {
	// ע��ÿ�������˵�·����ʼ���Ȳ��ܼ򵥵ظ��ݺ��ǰ��ȡ����ΪԲ����һ�������˲�һ��������һ����ŵĻ�����
	// �������������һ��ӳ�䣬����Բ�ϵ�˳���ţ������ź�robot_init_posһһ��Ӧ�������ӳ���ȥ�Ϳ�����
	// ����Ҫ��Ҫ֪��depot label -> circle label(cuts label)
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

	double opt = 0, wst = 2e9;
	for (int i = 0; i < depot.size(); ++i) {
		cuts[i].start = invSequence[depot[cut_depot[i]]];
		cuts[i].len = (invSequence[depot[cut_depot[(i + 1) % depot.size()]]] - invSequence[depot[cut_depot[i]]] + circleLen) % circleLen; // nr - nl + 1 - 1
		cuts[i].val = updateCutVal(i);
		opt = std::max(opt, cuts[i].val);
		wst = std::min(wst, cuts[i].val);
	}

	// ע�⣬MSTC*��������������������������·��Ȩ�ش��ڵ����·����ʵ�ʲ��Զ���ĳЩ��ͼ������������ܽ���ѭ����������Ҫ�޸�һ��
	int cur_iter = 0, max_iter = 10;
	// while (cur_iter < max_iter) 
	// infinite loop
	while (opt - wst > 10.0) {
		cout << "cutting for balancing...\n";     // just a sign
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

		cout << "before adjustment opt and wst: " << maxx << "  " << minn << "\n";
		// �ж�����ʱ�뻹��˳ʱ����һ��
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
		//opt = std::max_element(cuts.begin(), cuts.end(), [](cut& a, cut& b) { return a.val < b.val; })->val;
		//wst = std::min_element(cuts.begin(), cuts.end(), [](cut& a, cut& b) { return a.val < b.val; })->val;
		opt = 0, wst = 2e9;
		for(int i = 0; i < cuts.size(); ++i){
			opt = std::max(opt, cuts[i].val);
			wst = std::min(wst, cuts[i].val);
		}

		cout << "after adjustment opt and wst: " << opt << "  " << wst << "\n";
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

	if (dir == -1) std::reverse(res.begin(), res.end());   // ���������ն���˳ʱ����¸����ε���㣬���������鷳

	return res;
}

// ֻ��Ҫ����cut����㡢���Ⱥ�Ȩ�أ��������cut vec�޸ļ���
void PathCut::Balanced_Cut(vector<int>& adjustCuts) {
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

	//int lef = 0, rig = cuts[r_first].len + cuts[r_last].len - 1;  // -1��ȷ��������ֳ���Ϊ0�����
	double lef = 0, rig = getTurnAndLength(r_first) + getTurnAndLength(r_last);

	// ԭ��ֻ�ó��Ƚ��ж��֣������������ת�䣬���Ƕ�·������+ת��Ȩ�ؽ��ж��֣�Ȼ���ٸ��³���
	// ���³���ֻ�ܴ�ͷ��ʼ���ֲ���·����
	while (rig - lef > eps) {
		double mid = (lef + rig) / 2;
		int firstCutLen = std::lower_bound(pathValue.begin() + cuts[r_first].start, pathValue.end(), mid + pathValue[cuts[r_first].start]) - pathValue.begin() - cuts[r_first].start + 1;
		cuts[r_first].len = firstCutLen;
		cuts[r_last].len = old_len_r_first + old_len_r_last - firstCutLen;

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

		if (cuts[r_first].val < cuts[r_last].val)	lef = mid + 1;
		else if (cuts[r_first].val > cuts[r_last].val)	rig = mid - 1;
		else break;
	}
	
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

// ���ݵ����õ�cuts���������յ�·����ÿ�������˵�·���õ�һά���꣬��ʱ����ת������ʵ��������Ϳ�����
// ����A*���ɵ�·����ʵ������Ҫ����Ϊ���Խ���ROS��move_base��navigationȥ��
Mat PathCut::generatePath() {
	Mat path_for_each_robot(depot.size(), vector<int>{});
	for (int i = 0; i < cuts.size(); ++i) {
		// //cout << "using A star path...\n";
		// //cout << depot[i] << ", " << pathSequence[cuts[i].start] << endl;
		// vector<int> p1 = A_star_path(depot[cut_depot[i]], pathSequence[cuts[i].start]);
		// //cout << pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen] << ", " << depot[i] << endl;
		// vector<int> p2 = A_star_path(pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen], depot[cut_depot[i]]);
		// //cout << "A star path ending...\n";
		// for (int j = 0; j < cuts[i].len; ++j)	p1.push_back(pathSequence[(cuts[i].start + j) % circleLen]);

		// p1.insert(p1.end(), p2.begin(), p2.end());
		vector<int> p1;
		for (int j = 0; j < cuts[i].len; ++j)	p1.push_back(pathSequence[(cuts[i].start + j) % circleLen]);
		path_for_each_robot[i] = p1;
	}

	Mat path_final(depot.size(), vector<int>{});
	for (int i = 0; i < cuts.size(); ++i) {
		path_final[cut_depot[i]] = path_for_each_robot[i];

		// �����ɸ��Ǻ�ص�ԭ��Ļ��Ͱ���ʼ�����
		if(coverAndReturn){
			path_final[cut_depot[i]].push_back(depot[cut_depot[i]]);
		}
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