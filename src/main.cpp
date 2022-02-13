#include "PathCut.h"
#include "MaximumSubRectDivision.h"

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

int main() {
	/*Mat Map(5, vector<int>(5, 0));
	Map[0][1] = Map[0][2] = Map[0][3] = Map[0][4] = 1;
	Map[1][0] = Map[1][1] = Map[1][2] = Map[1][3] = Map[1][4] = 1;
	Map[2][2] = Map[2][3] = Map[2][4] = 1;
	Map[3][1] = Map[3][2] = Map[3][3] = Map[3][4] = 1;
	Map[4][0] = Map[4][1] = Map[4][2] = Map[4][3] = Map[4][4] = 1;*/
	
	// 需要根据地图规模调整
	
	//----------------------------------------------------------
	//int h, w;
	//// file input
	//ifstream infile;
	//infile.open("C:/Users/a1120/Desktop/map2.txt", ios::in);
	//infile >> h >> w;
	//Mat Map(h, vector<int>(w, 0));
	//for (int i = 0; i < h; ++i) {
	//	for (int j = 0; j < w; ++j) {
	//		infile >> Map[i][j];  //主要是有的测试文件没有空格所以不能这样直接读入
	
	//		// 新地图要取反！！
	
	//		//Map[i][j] ^= 1;
	//		//cout << Map[i][j] << " ";
	//	}
	//	//cout << endl;
	//}
	//---------------------------------------------------------
	
	// 用下面这个
	//---------------------------------------------------------
	int h, w;
	ifstream infile("/home/courierlo/testData/bath.txt");
	infile >> h >> w;
	cout << h << " " << w << endl;
	//cout << "wtf??\n";
	Mat Map(h, vector<int>(w, 0));
	
	int p;
	string line;
	int i = 0;
	getline(infile, line);
	while (getline(infile, line)) {
		//cout << "line length: " << line.length() << endl;
		for (int j = 0; j < line.length(); ++j) {
			p = line[j] - '0';
	
			//注意地图标号 是否要取反 !!!!!!!!!! 自己造的random_n不需要取反而别人的bath/school要取反
			p ^= 1;
			//if (p)	paint1(i, j);
			Map[i][j] = p;
			//cout << p << " ";
		}
		i++;
		//cout << "\n";
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

	// 2021.11.27 这里的Map是缩小的，需要再构造原本的region
	Division div(Map);
	vector<int> robot_init_pos{ 39991 };// 11, 12, 39991, 39992;
	//vector<int> robot_init_pos{ 3, 1520, 9301, 9999 };
	Mat MST = div.rectDivisionSolver();
	
	PathCut cut(Map, Region, MST, robot_init_pos);
	Mat ans = cut.cutSolver();

	Mat dfsMST_vertical_MST = div.dfsWithStackSolver(VERTICAL);
	Mat dfsMST_horizontal_MST = div.dfsWithStackSolver(HORIZONTAL);
	Mat kruskalMST = div.kruskalSolver();
	
	PathCut dfsCut_vertical(Map, Region, dfsMST_vertical_MST, robot_init_pos);
	Mat vtl = dfsCut_vertical.cutSolver();

	PathCut dfsCut_horizontal(Map, Region, dfsMST_horizontal_MST, robot_init_pos);
	Mat hrl = dfsCut_horizontal.cutSolver();

	PathCut kruskalCut(Map, Region, kruskalMST, robot_init_pos);
	Mat kal = kruskalCut.cutSolver();

	//vector<rect>ans = chessboardPartion(Map);
	//其实没必要sort了
	//sort(ans.begin(), ans.end());
	
	/*for (int i = 0; i < ans.size(); ++i) {
		cout << "No." << i << "'s rectangle—> area: ";
		cout << ans[i].height * ans[i].width << " ";
		cout << " / corner: " << ans[i].corner.first << " " << ans[i].corner.second;
		cout << " / dir: " << (int)ans[i].dir;
		cout << "\n";
	}
	cout << "-----------------------------------\n";*/
	
	cout << "\n\n------------------------------------------ \n";
	cout << "The number of turns from Rectangle Division is " << endl;
	getPathInfo(ans);

	cout << "The number of turns from Dfs Vertical is " << endl;
	getPathInfo(vtl);

	cout << "The number of turns from Dfs Horizontal is " << endl;
	getPathInfo(hrl);

	cout << "The number of turns from Kruskal is " << endl;
	getPathInfo(kal);
	
	
	return 0;
}