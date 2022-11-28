/* Use for algorithm debugging, doesn't involve in ROS */

#include "PathCut.h"
#include "MaximumSubRectDivision.h"
#include "ACO_STC.h"
#include "Dinic.h"
#include "m_TSP.h"
#include "HeuristicPartition.h"

#define DEBUG

bool isSameLine(int& a, int& b, int& c) {
	return a + c == 2 * b;
}

void getPathInfo(Mat& path) {
	for (int i = 0; i < path.size(); ++i) {
		int curTurn = 0;
		for (int j = 1; j < path[i].size() - 1; ++j) {
			curTurn += isSameLine(path[i][j - 1], path[i][j], path[i][j + 1]) ? 0 : 1;
		}
		cout << "No." << i << "'s path's length and turns and its value: " << path[i].size() << " " << curTurn << " " << 1.0 * path[i].size() + ONE_TURN_VAL * curTurn << endl;
	}
	cout << "-----------------------------------------\n";
}

void checkMST(Mat& mst) {
	for (int i = 0; i < mst.size(); ++i) {
		if (!mst[i].size())	continue;
		cout << i << " - ";
		for (auto nxt : mst[i])	cout << nxt << " ";
		cout << endl;
	}
}

int main() {
	srand(0);
	// Mat Map(5, vector<int>(5, 0));
	// Map[0][1] = Map[0][2] = Map[0][3] = Map[0][4] = 1;
	// Map[1][0] = Map[1][1] = Map[1][2] = Map[1][3] = Map[1][4] = 1;
	// Map[2][2] = Map[2][3] = Map[2][4] = 1;
	// Map[3][1] = Map[3][2] = Map[3][3] = Map[3][4] = 1;
	// Map[4][0] = Map[4][1] = Map[4][2] = Map[4][3] = Map[4][4] = 1;

	// ��Ҫ���ݵ�ͼ��ģ����

	// ���������
	// ---------------------------------------------------------
	int h, w;
	ifstream infile("/home/courierlo/test_data_sample/Denver_2.txt");
	infile >> h >> w;
	cout << h << " " << w << endl;
	Mat Map(h, vector<int>(w, 0));

	int p;
	string line;
	int i = 0;
	getline(infile, line);
	while (getline(infile, line)) {
		//cout << "line length: " << line.length() << endl;
		int len = 0;
		for (int j = 0; j < line.length(); ++j) {
			if (line[j] != '1' && line[j] != '0')	continue;

			p = line[j] - '0';
			//ע���ͼ��� �Ƿ�Ҫȡ�� !!!!!!!!!! �Լ����random_n����Ҫȡ�������˵�bath/schoolҪȡ��
			// p ^= 1;
			//if (p)	paint1(i, j);
			Map[i][len++] = p;
			cout << p << " ";
		}
		i++;
		cout << "\n";
	}
	//-----------------------------------------------------------


	Mat Region(2 * Map.size(), vector<int>(2 * Map[0].size(), 0));
	for (int i = 0; i < Map.size(); ++i) {
		for (int j = 0; j < Map[0].size(); ++j) {
			Region[2 * i][2 * j] = Map[i][j];
			Region[2 * i][2 * j + 1] = Map[i][j];
			Region[2 * i + 1][2 * j] = Map[i][j];
			Region[2 * i + 1][2 * j + 1] = Map[i][j];
		}
	}

	cout << "begin hp...\n";
	HeuristicSolver::HeuristicPartition hp(Map, 1000);
    Mat MST = hp.hpSolver(true);
	// hp.showRanks();
	hp.checkConnectivity();
	// 2021.11.27 �����Map����С�ģ���Ҫ�ٹ���ԭ����region
	// Division div(Map);
	// vector<int> robot_init_pos{ 15 }; // 11, 12, 9991, 9992
	// //vector<int> robot_init_pos{ 3, 1520, 9301, 9999 };
	// Mat MST = div.rectDivisionSolver();
	// //PathCut cut(Map, Region, MST, robot_init_pos);
	// //Mat ans = cut.cutSolver();

	// Mat dfsMST_vertical = div.dfsWithStackSolver(VERTICAL);
	// //PathCut dfsCut_vertical(Map, Region, dfsMST_vertical, robot_init_pos);
	// //Mat vtl = dfsCut_vertical.cutSolver();

	// Mat dfsMST_horizontal = div.dfsWithStackSolver(HORIZONTAL);
	// //PathCut dfsCut_horizontal(Map, Region, dfsMST_horizontal, robot_init_pos);
	// //Mat hrl = dfsCut_horizontal.cutSolver();

	// Mat kruskalMST = div.kruskalSolver();
	// //PathCut kruskalCut(Map, Region, kruskalMST, robot_init_pos);
	// //Mat kal = kruskalCut.cutSolver();
	
	// -------------------------------GA-MTSP------------------------------
	// Mat bricks = dinic.dinic_solver(Map, false);
	// dinic.formBricksForMTSP(Map);
	// for(int i = 0; i < dinic.bricks.size(); ++i){
	// 	cout << i << "th Brick's info: \n";
	// 	cout << "4 corners : \n";
	// 	for(int j = 0; j < 4; ++j){
	// 		cout << "(" << dinic.bricks[i].corner[j].first << ", " << dinic.bricks[i].corner[j].second << ")";
	// 	}
	// 	cout << "\nmid pts: \n";
	// 	for(int j = 0; j < dinic.bricks[i].mid_pts.size(); ++j){
	// 		cout << "(" << dinic.bricks[i].mid_pts[j].first << ", " << dinic.bricks[i].mid_pts[j].second << ")";
	// 	}

	// 	cout << "\n";
	// }
	
	// GA_CONF conf{ 0.9, 0.01, 0.3, 2, 10000, 30, 100 };
	// vector<int> depot{ 3, 96 };
	// MTSP mtsp(conf, dinic, depot, 10, true);
	// for(int i = 0; i < 100000; ++i){
	// 	// cout << i << endl;
	// 	mtsp.GANextGeneration();
	// 	if(mtsp.unchanged_gens >= 1000)   break;
	// 	if(i % 1000 == 0){
	// 		// display cost for debugging
	// 		cout << i << " GA-cost: " << mtsp.best_val << "\n";
	// 	}
	// }

	// mtsp.showpath();

	// Mat path_idx = mtsp.MTSP2Path();

	// for(int i = 0; i < path_idx.size(); ++i){
	// 	cout << "path " << i << ": ";
	// 	for(int j = 0; j < path_idx[i].size(); ++j){
	// 		cout << path_idx[i][j] << " ";
	// 	}
	// 	cout << "\n";
	// }
	//---------------------------------------------------------------------

	// int _alpha, int _beta, double _pheromone_init, double _pho, int _ant_num, int _max_iter, Mat& _Map
	// ACO_STC aco(2.5, 1, 1, 0.15, 30, 300, Map, dinic_MST);  // 400 1200
	// Mat optimized_mst = aco.aco_stc_solver();
	// aco.get_result_info();
	//PathCut acoCut(Map, Region, optimized_mst, robot_init_pos);
	//Mat aco_path = acoCut.cutSolver();

	//vector<rect>ans = chessboardPartion(Map);
	//��ʵû��Ҫsort��
	//sort(ans.begin(), ans.end());

	/*for (int i = 0; i < ans.size(); ++i) {
		cout << "No." << i << "'s rectangle��> area: ";
		cout << ans[i].height * ans[i].width << " ";
		cout << " / corner: " << ans[i].corner.first << " " << ans[i].corner.second;
		cout << " / dir: " << (int)ans[i].dir;
		cout << "\n";
	}
	cout << "-----------------------------------\n";*/

	/*cout << "\n\n------------------------------------------ \n";
	cout << "The number of turns from Rectangle Division is " << endl;
	getPathInfo(ans);

	cout << "The number of turns from Dfs Vertical is " << endl;
	getPathInfo(vtl);

	cout << "The number of turns from Dfs Horizontal is " << endl;
	getPathInfo(hrl);

	cout << "The number of turns from Kruskal is " << endl;
	getPathInfo(kal);

	cout << "The number of turns from ACO is " << endl;
	getPathInfo(aco_path);*/

	return 0;
}