#include "ACO_STC.h"
#include <iostream>
#include <fstream>
#include <sstream>

void ACO_STC::reset() {
	for (int i = 0; i < info.size(); ++i) {
		for (int j = 0; j < 4; ++j) {
			info[i][j] = pow(pheromone[i][j], alpha) * pow(DIS, beta);
		}
	}
}

vector<int> ACO_STC::is_valid(int dx, int dy) {
	vector<int> neighbor;
	for (int i = 0; i < 4; ++i) {
		int sx = dx + dir[i][0], sy = dy + dir[i][1];
		if (sx < 0 || sx >= row || sy < 0 || sy >= col || vis[sx * col + sy] || !Map[sx][sy])	continue;

		neighbor.push_back(sx * col + sy);
	}

	return neighbor;
}

bool ACO_STC::select_next() {
	// can't expand here, then backtrack
	//cout << cur_pos << endl;
	vis[cur_pos] = true;
	int dx = cur_pos / col, dy = cur_pos % col;
	vector<int> neighbor = is_valid(dx, dy);
	while (!neighbor.size()) {
		path.pop();
		if (path.empty())	return true;

		cur_pos = path.top();
		dx = path.top() / col, dy = path.top() % col;
		neighbor = is_valid(dx, dy);
	}

	// elite rule ? q0. Implement this later
	/*double q0 = 0.2;
	double q = 1.0 * rand() / RAND_MAX;*/

	double prob_sum = 0, prob_pre = 0;
	for (int i = 0; i < neighbor.size(); ++i) {
		if (neighbor[i] - cur_pos == -col)	    prob_sum += info[cur_pos][0];
		else if (neighbor[i] - cur_pos == col)	prob_sum += info[cur_pos][2];
		else if (neighbor[i] - cur_pos == 1)	prob_sum += info[cur_pos][1];
		else									prob_sum += info[cur_pos][3];
	}

	double rnd = 1.0 * rand() / RAND_MAX;
	rnd *= prob_sum;
	for (int i = 0; i < neighbor.size(); ++i) {
		if (neighbor[i] - cur_pos == -col)	    prob_pre += info[cur_pos][0];
		else if (neighbor[i] - cur_pos == col)	prob_pre += info[cur_pos][2];
		else if (neighbor[i] - cur_pos == 1)	prob_pre += info[cur_pos][1];
		else									prob_pre += info[cur_pos][3];

		if (prob_pre >= rnd) {
			cur_MST[cur_pos].push_back(neighbor[i]);
			cur_MST[neighbor[i]].push_back(cur_pos);
			cur_pos = neighbor[i];
			path.push(neighbor[i]);
			return false;
		}
	}

	return false;
}

int ACO_STC::get_turns(Mat& MST) {
	int turns = 0, nodes = 0;
	for (int i = 0; i < MST.size(); ++i) {
		if (MST[i].size()) nodes++;

		if (MST[i].size() == 3 || MST[i].size() == 1)		 turns += 2;
		else if (MST[i].size() == 4)						 turns += 4;
		else {
			if (MST[i].size() == 2 && MST[i][0] + MST[i][1] != 2 * i)	turns += 2;
		}
	}
	//cout << "There are " << nodes << " node on the tree.\n";

	return turns;
}

void ACO_STC::construct_solution() {
	for (int ant = 0; ant < ant_num; ++ant) {
		// clear vis and cur_MST
		//cout << "-------------------------------------\n";
		//cout << "No." << ant << "'s ant\n";
		cur_pos = ant_init_pos[ant];
		cur_MST = Mat(cell_num, vector<int>{});
		vis = vector<bool>(cell_num, false);
		while (!path.empty())	path.pop();

		path.push(cur_pos);
		while (!select_next());

		int cur_turn = get_turns(cur_MST);
		// cout << "cur_MST turn: " << get_turns(cur_MST) << "\n";
		if (min_turn_this_iter > cur_turn) {
			min_turn_this_iter = cur_turn;
			cur_iter_best_MST = cur_MST;
			cout << "cur_MST turn: " << cur_turn << "\n";
		}
		if (min_turn_so_far > min_turn_this_iter) {
			min_turn_so_far = min_turn_this_iter;
			best_MST = cur_MST;
		}

		//cout << "---------------------------------------\n";
	}
}

void ACO_STC::update_pheromone() {
	for (int i = 0; i < cur_iter_best_MST.size(); ++i) {
		for (int j = 0; j < 4; ++j)	pheromone[i][j] *= (1 - pho);
		for (int j = 0; j < cur_iter_best_MST[i].size(); ++j) {
			if (cur_iter_best_MST[i][j] - i == -col)	 pheromone[i][0] += PHE / min_turn_this_iter;
			else if (cur_iter_best_MST[i][j] - i == 1)	 pheromone[i][1] += PHE / min_turn_this_iter;
			else if (cur_iter_best_MST[i][j] - i == col) pheromone[i][2] += PHE / min_turn_this_iter;
			else									     pheromone[i][3] += PHE / min_turn_this_iter;
		}
	}
}

void ACO_STC::use_best_MST_phe() {
	for (int i = 0; i < best_MST.size(); ++i) {
		int turn_1 = 0, turn_2 = 0;
		if (best_MST[i].size() == 3 || best_MST[i].size() == 1)		 turn_1 += 2;
		else if (best_MST[i].size() == 4)							 turn_1 += 4;
		else {
			if (best_MST[i].size() == 2 && best_MST[i][0] + best_MST[i][1] != 2 * i)	turn_1 += 2;
		}

		for (int j = 0; j < best_MST[i].size(); ++j) {
			int to = best_MST[i][j];
			if (best_MST[to].size() == 3 || best_MST[to].size() == 1)		 turn_2 += 2;
			else if (best_MST[to].size() == 4)								 turn_2 += 4;
			else {
				if (best_MST[to].size() == 2 && best_MST[to][0] + best_MST[to][1] != 2 * i)	turn_2 += 2;
			}

			if (best_MST[i][j] - i == -col)		 pheromone[i][0] += pheromone_init * 10 / (turn_1 + turn_2);
			else if (best_MST[i][j] - i == 1)	 pheromone[i][1] += pheromone_init * 10 / (turn_1 + turn_2);
			else if (best_MST[i][j] - i == col)  pheromone[i][2] += pheromone_init * 10 / (turn_1 + turn_2);
			else								 pheromone[i][3] += pheromone_init * 10 / (turn_1 + turn_2);
		}
	}
}

Mat ACO_STC::aco_stc_solver() {
	clock_t start, finish;
	start = clock();

	// use best construct tree as start
	if(best_MST.size()){
		use_best_MST_phe();
		min_turn_so_far = get_turns(best_MST);
	}

	int bad_times = 0;
	int last_turns = 1234567890;
	for (int it = 0; it < max_iter; ++it) {
		if (bad_times > 20) {
			cout << "bad luck. break...\n";
			break;
		}

		reset();
		construct_solution();
		update_pheromone();

		if (last_turns > min_turn_so_far) {
			last_turns = min_turn_so_far;
			bad_times = 0;
		}
		else  bad_times++;
	}

	finish = clock();
	cout << "ACO_ST solver used time: " << finish - start << endl;
	get_result_info();
	cout << "-------------------ACO_STC Solver End-------------------\n\n";

	return best_MST;
}

void ACO_STC::get_result_info() {
	std::cout << "The number of optimized MST's turn is: " << min_turn_so_far << std::endl;
	int freecell = 0, edges = 0;
	for (int i = 0; i < Map.size(); ++i)
		for (int j = 0; j < Map[0].size(); ++j)
			if (Map[i][j])	freecell++;

	for (int i = 0; i < best_MST.size(); ++i)	edges += best_MST[i].size();

	std::cout << "free cells and edges: " << freecell << " " << edges / 2 << std::endl;
}

//int main() {
//	int h, w;
//	std::ifstream infile("C:/Users/a1120/Desktop/test_data/random_10_10.txt");
//	infile >> h >> w;
//	std::cout << h << " " << w << std::endl;
//	Mat Map(h, vector<int>(w, 0));
//
//	int p;
//	std::string line;
//	int i = 0;
//	getline(infile, line);
//	while (getline(infile, line)) {
//		//cout << "line length: " << line.length() << endl;
//		int len = 0;
//		for (int j = 0; j < line.length(); ++j) {
//			if (line[j] != '1' && line[j] != '0')	continue;
//
//			p = line[j] - '0';
//			//注意地图标号 是否要取反 !!!!!!!!!! 自己造的random_n不需要取反而别人的bath/school要取反
//			//p ^= 1;
//			//if (p)	paint1(i, j);
//			Map[i][len++] = p;
//			//cout << p << " ";
//		}
//		i++;
//		//cout << "\n";
//	}
//
//	// int _alpha, int _beta, double _pheromone_init, double _pho, int _ant_num, int _max_iter, Mat& _Map
//	ACO_STC aco(1, 1, 1, 0.15, 30, 150, Map);
//	Mat optimized_mst = aco.aco_stc_solver();
//	aco.get_result_info();
//
//	return 0;
//}