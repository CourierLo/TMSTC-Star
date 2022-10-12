#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

typedef std::vector<std::vector<int>> intMatrix;
typedef std::vector<std::vector<double>> doubleMatrix;
typedef std::vector<std::vector<float>> floatMatrix;
typedef std::vector<std::vector<bool>> boolMatrix;

class ConnectComponent {
public:
	int MAX_LABELS;
	int next_label = 1;
	intMatrix label2d, BinaryRobot, BinaryNonRobot;
	int rows, cols;
	std::vector<int> parent, labels;

	ConnectComponent(int r, int c) : rows(r), cols(c){
		label2d.resize(rows, std::vector<int>(cols));
		MAX_LABELS = rows * cols;
		parent.resize(rows * cols + 5, 0);
		//for (int i = 0; i < rows * cols; ++i) parent[i] = i;
		labels.resize(rows * cols + 5, 0);
	}

	void unite(int x, int y) {
		//cout << x << " " << y << endl;
		while (parent[x] > 0)	x = parent[x];
		while (parent[y] > 0)	y = parent[y];
		if (x != y) {
			if (x < y)	parent[x] = y;
			else  parent[y] = x;
		}
	}

	int find(int x) {
		while (parent[x] > 0)	x = parent[x];
		if (!labels[x])	labels[x] = next_label++;

		return labels[x];
	}

	intMatrix deepCopyMatrix(intMatrix &input) {
		if (input.size() == 0)	return {};
		intMatrix res(input.size(), std::vector<int>(input[0].size()));
		// check here!
		for (int i = 0; i < input.size(); ++i)
			std::copy(input[i].begin(), input[i].end(), res[i].begin());

		return res;
	}

	void constructBinaryImages(int robotLabel) {
		BinaryRobot = label2d;
		BinaryNonRobot = label2d;

		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				if (label2d[i][j] == robotLabel) {
					BinaryRobot[i][j] = 1;
					BinaryNonRobot[i][j] = 0;
				}
				else if (label2d[i][j] != 0) {
					BinaryRobot[i][j] = 0;
					BinaryNonRobot[i][j] = 1;
				}
			}
		}
	}

	std::vector<float> getVector(floatMatrix& A, int row) {
		std::vector<float> ret;
		for (auto num : A[row])	ret.push_back(num);
		return ret;
	}

	void DT1D(std::vector<float> f, std::vector<float> &d, std::vector<int> &v, std::vector<float> &z) {
		int k = 0;
		v[0] = 0;  z[0] = -1e30;  z[1] = 1e30;

		for (int q = 1; q < f.size(); ++q) {
			float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);

			while (s <= z[k]) {
				k--;
				s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
			}
			k++;
			v[k] = q;
			z[k] = s;
			z[k + 1] = 1e30;
		}

		k = 0;
		for (int q = 0; q < f.size(); q++) {
			while (z[k + 1] < q)  k++;

			d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
		}
	}

	std::vector<int> transformImage2Dto1D(intMatrix &a) {
		std::vector<int> ret(rows * cols);
		int k = 0;
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				ret[k++] = a[i][j];
			}
		}
		return ret;
	}

	floatMatrix normalizedEuclideanDistance(bool robotR) {
		floatMatrix region(rows, std::vector<float>(cols));
		std::vector<float> f(std::max(rows, cols));
		std::vector<float> d(f.size());
		std::vector<int>   v(f.size());
		std::vector<float> z(f.size() + 1);

		for (int x = 0; x < cols; ++x) {
			for (int y = 0; y < rows; ++y) {
				if (robotR)	f[y] = BinaryRobot[y][x] == 0 ? 1e30 : 0;
				else		f[y] = BinaryNonRobot[y][x] == 0 ? 1e30 : 0;
			}

			DT1D(f, d, v, z);
			for (int y = 0; y < rows; ++y)	region[y][x] = d[y];
		}

		float maxV = 0, minV = 1e30;
		for (int y = 0; y < rows; ++y) {
			DT1D(getVector(region, y), d, v, z);

			for (int x = 0; x < cols; ++x) {
				region[y][x] = (float)sqrt(d[x]);
				maxV = std::max(maxV, region[y][x]);
				minV = std::min(minV, region[y][x]);
			}
		}

		for (int y = 0; y < rows; ++y) {
			for (int x = 0; x < cols; ++x) {
				if (robotR) region[y][x] = (region[y][x] - minV) * (1.0 / (maxV - minV)) + 1;
				else		region[y][x] = (region[y][x] - minV) * (1.0 / (maxV - minV));
			}
		}

		return region;
	}

	std::vector<int> labeling(std::vector<int>& image, bool zeroAsBg) {
		std::vector<int> rst(rows * cols + 5, 0);

		int next_region = 1;
		for (int y = 0; y < rows; ++y) {
			for (int x = 0; x < cols; ++x) {
				if (!image[y * cols + x] && zeroAsBg)  continue;

				int k = 0;
				bool connected = false;
				if (x > 0 && image[y * cols + x - 1] == image[y * cols + x]) {
					k = rst[y * cols + x - 1];
					connected = true;
				}

				if (y > 0 && image[(y - 1) * cols + x] == image[y * cols + x] && (!connected || image[(y - 1) * cols + x] < k)) {
					k = rst[(y - 1) * cols + x];
					connected = true;
				}

				if (!connected)  k = next_region++;
				//cout << y * cols + x << endl;
				rst[y * cols + x] = k;
				if (x > 0 && image[y * cols + x - 1] == image[y * cols + x] && rst[y * cols + x - 1] != k)
					unite(k, rst[y * cols + x - 1]);
				if (y > 0 && image[(y - 1) * cols + x] == image[y * cols + x] && rst[(y - 1) * cols + x] != k)
					unite(k, rst[(y - 1) * cols + x]);
			}
		}

		next_label = 1;
		for (int i = 0; i < rows * cols; ++i) {
			if (image[i] != 0 || !zeroAsBg) {
				rst[i] = find(rst[i]);
				if (!zeroAsBg)	rst[i]--;
			}
		}
		next_label--;
		if (!zeroAsBg) next_label--;

		//for (int i = 0; i < 36; ++i)	cout << rst[i] << " ";
		//cout << endl;

		return rst;
	}

	intMatrix compactLabeling(intMatrix& m, bool zeroAsBg) {
		std::vector<int> image = transformImage2Dto1D(m);
		std::vector<int> label = labeling(image, zeroAsBg);
		std::vector<int> stat(next_label + 1);

		for (int i = 0; i < image.size(); ++i)	stat[label[i]]++;

		//remap  ÊÇ±àºÅ°´Ë³ÐòµÝÔö
		stat[0] = 0;
		int j = 1;
		for (int i = 1; i < stat.size(); ++i)
			if (stat[i])	stat[i] = j++;

		next_label = j - 1;
		int locIDX = 0;
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				label[locIDX] = stat[label[locIDX]];
				label2d[i][j] = label[locIDX];
				locIDX++;
			}
		}

		return label2d;
	}

};
