#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <random>

using std::vector;
using std::cout;
using std::endl;

#define EPS 1e-8
#define DIS 1.0
#define PHE 100.0
typedef vector<vector<int> > Mat;

class ACO_STC {
	vector<vector<double> > pheromone;  // start from 1
	vector<vector<double> > info;       // pheromone ^ alpha * herustic ^ beta
	int ant_num;
	double alpha, beta;
	double pheromone_init;
	double pho;						    // »Ó·¢ÏµÊý
	int max_iter;

	Mat Map;
	Mat cur_MST, best_MST, cur_iter_best_MST;
	vector<bool> vis;
	int min_turn_so_far, min_turn_this_iter;
	int row, col, cell_num;

	vector<int> dir[4] = { {-1, 0}, {0, 1}, {1, 0}, {0, -1} };
	vector<int> step;
	vector<int> ant_init_pos;
	std::stack<int> path;
	int cur_pos, cur_turns;

public:
	ACO_STC(double _alpha, double _beta, double _pheromone_init, double _pho, int _ant_num, int _max_iter, Mat& _Map, Mat _best_MST) :
		alpha(_alpha), beta(_beta), pheromone_init(_pheromone_init), pho(_pho), ant_num(_ant_num), max_iter(_max_iter), Map(_Map), best_MST(_best_MST) {
		row = Map.size(); col = Map[0].size();
		cout << "row and col " << row << " " << col << endl;
		cell_num = row * col;

		pheromone.resize(cell_num, vector<double>(4, 1));   // non best MST edge, start with 0.1
		info.resize(cell_num, vector<double>(4, 0));
		//cur_MST.resize(cell_num, {});
		//vis.resize(cell_num, false);

		step = { -col, 1, col, -1 };

		// generate ants' initial position randomly
		std::unordered_map<int, bool> used;
		ant_init_pos.resize(ant_num);
		std::random_device rd;   // non-deterministic generator
		std::mt19937 gen(rd());  // to seed mersenne twister.
		for (int i = 0; i < ant_num; ++i) {
			while (true) {
				int rnd = gen() % cell_num;
				if (!used.count(rnd) && Map[rnd / col][rnd % col]) {
					used[rnd] = true;
					ant_init_pos[i] = rnd;
					break;
				}
			}
			//cout << ant_init_pos[i] << " ";
		}
		//cout << "\n";

		min_turn_so_far = 1234567890;
		min_turn_this_iter = 1234567890;
	}

	void reset();					// update info for upcoming iteration
	bool select_next();				// expand a new node and return its index, return true when finish constructing
	void construct_solution();		// find the best MST in this iteration among all these ants
	void update_pheromone();	    // use the number of best MST's turn to update
	Mat aco_stc_solver();			// wrapper
	vector<int> is_valid(int dx, int dy);
	int get_turns(Mat& MST);
	void get_result_info();
	void use_best_MST_phe();
};
