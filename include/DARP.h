#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <random>
#include <cmath>
#include "ConnectComponent.h"

typedef std::vector<std::vector<int>> intMatrix;
typedef std::vector<std::vector<double>> doubleMatrix;
typedef std::vector<std::vector<float>> floatMatrix;
typedef std::vector<std::vector<bool>> boolMatrix;

class DARP {
public:
	int rows, cols, nr, ob, maxIter;
	double variateWeight, randomLevel;
	intMatrix GridEnv;
	intMatrix A;
	int minAssigned, maxAssigned, discr;
	bool success, canceled, useImportance;
	std::vector<std::pair<int, int>> robotInit;
	std::vector<bool> robotRegionsConnected;
	std::vector<int> elements;
	std::vector<intMatrix> robotRegion;

	DARP(intMatrix& src, int iters, double vWeight, double rLevel, int discr, bool imp) {
		rows = src.size();  cols = src[0].size();
		nr = ob = maxAssigned = minAssigned = 0;
		maxIter = iters;
		variateWeight = vWeight;
		this->discr = discr;
		canceled = false;
		useImportance = imp;		//for now it's useless
		randomLevel = rLevel;
		success = false;

		GridEnv = src;

		A.resize(rows, std::vector<int>(cols, 0));

		defineRobotsObstacles();

		robotRegionsConnected.resize(nr, false);  //?
		//no need to resize robotInit
		elements.resize(nr, 0);

		robotRegion.resize(nr, std::vector<std::vector<int>>(rows, std::vector<int>(cols, false)));
	}

	void defineRobotsObstacles() {
		for (int i = 0; i < rows; ++i) {              // int i = 0; i < rows; ++i
			for (int j = 0; j < cols; ++j) {		  // int j = 0; j < cols; ++j
				if (GridEnv[i][j] == 2) {
					robotInit.push_back({ i, j });	
					std::cout << "darp robot 2D coor: " << i << " " << j << std::endl;
					GridEnv[i][j] = nr;
					A[i][j] = nr;
					nr++;
				}
				else if (GridEnv[i][j] == 1) {
					ob++;
					GridEnv[i][j] = -2;
				}
				else  GridEnv[i][j] = -1;
			}
		}
		std::cout << "nr and ob number: " << nr << " " << ob << std::endl;
	}

	doubleMatrix finalUpdateOnE(doubleMatrix CM, doubleMatrix RM, doubleMatrix curOne, floatMatrix CC) {
		doubleMatrix newM;
		newM.resize(rows, std::vector<double>(cols));
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < cols; ++j)
				newM[i][j] = curOne[i][j] * CM[i][j] * RM[i][j] * CC[i][j];

		return newM;
	}

	doubleMatrix generateRandomMatrix() {
		doubleMatrix randomM(rows, std::vector<double>(cols));
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<>random(0.0, 1.0);
		//random(gen)
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < cols; ++j)
				randomM[i][j] = 2.0 * randomLevel * random(gen) + 1.0 - randomLevel;

		return randomM;
	}

	void getRobotRegion() {
		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < cols; ++j)
				if (A[i][j] < nr)	robotRegion[A[i][j]][i][j] = true;
	}

	doubleMatrix getCriterionMatrix(doubleMatrix tilesImp, double minImp, double maxImp, double corMult, bool smallerThan0) {
		doubleMatrix criter(rows, std::vector<double>(cols));
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				if (useImportance) {
					if (smallerThan0)	criter[i][j] = (tilesImp[i][j] - minImp) * ((corMult - 1) / (maxImp - minImp)) + 1;
					else   criter[i][j] = (tilesImp[i][j] - minImp) * ((corMult - 1) / (maxImp - minImp)) + corMult;
				}
				else  criter[i][j] = corMult;
			}
		}
		return criter;
	}

	bool isGoalState(int threshold) {
		maxAssigned = 0;  minAssigned = 1234567891;
		for (int r = 0; r < nr; ++r) {
			maxAssigned = std::max(maxAssigned, elements[r]);
			minAssigned = std::min(minAssigned, elements[r]);

			if (!robotRegionsConnected[r])	return false;
		}

		return threshold >= maxAssigned - minAssigned;
	}

	floatMatrix getMatrixCr(floatMatrix dist1, floatMatrix dist2) {
		floatMatrix C(rows, std::vector<float>(cols));
		float maxV = 0, minV = 1e30;
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				C[i][j] = dist1[i][j] - dist2[i][j];
				maxV = std::max(maxV, C[i][j]);
				minV = std::min(minV, C[i][j]);
			}
		}

		for (int i = 0; i < rows; ++i)
			for (int j = 0; j < cols; ++j)
				C[i][j] = (C[i][j] - minV) * ((2 * (float)variateWeight) / (maxV - minV)) + (1 - (float)variateWeight);

		return C;
	}

	void assign(std::vector<doubleMatrix> E) {
		// init robotRegion
		for (int r = 0; r < nr; ++r)
			for (int i = 0; i < rows; ++i)
				fill(robotRegion[r][i].begin(), robotRegion[r][i].end(), 0);

		for (int r = 0; r < nr; ++r)
			robotRegion[r][robotInit[r].first][robotInit[r].second] = 1;

		fill(elements.begin(), elements.end(), 0);
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				if (GridEnv[i][j] == -1) {
					double minV = E[0][i][j];
					int indexMin = 0;
					for (int r = 0; r < nr; ++r) {
						if (minV > E[r][i][j]) {
							minV = E[r][i][j];
							indexMin = r;
						}
					}
					A[i][j] = indexMin;
					robotRegion[indexMin][i][j] = 1;
					elements[indexMin]++;
				}
				else if (GridEnv[i][j] == -2)
					A[i][j] = nr;
			}
		}
	}

	double EuclideanDistance(std::pair<int, int> p1, std::pair<int, int> p2) {
		return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
	}

	void constructAssignmentMatrix() {
		int tilesNum = rows * cols;
		int termThr;
		int freeCells = tilesNum - nr - ob;
		termThr = (freeCells % nr) == 0 ? 0 : 1;

		std::vector<doubleMatrix> allDistances, tilesImportance;
		for (int r = 0; r < nr; ++r) {
			allDistances.push_back(doubleMatrix(rows, std::vector<double>(cols)));
			tilesImportance.push_back(doubleMatrix(rows, std::vector<double>(cols)));
		}

		std::vector<double> maxDist(nr), maxImportance(nr, 0), minImportance(nr, 1e30);
		floatMatrix ONES2D(rows, std::vector<float>(cols));

		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				double tempSum = 0.0;
				for (int r = 0; r < nr; ++r) {
					allDistances[r][i][j] = EuclideanDistance(robotInit[r], { i, j });
					maxDist[r] = std::max(maxDist[r], allDistances[r][i][j]);
					tempSum += allDistances[r][i][j];
				}

				for (int r = 0; r < nr; ++r) {
					tilesImportance[r][i][j] = 1.0 / (tempSum - allDistances[r][i][j]);
					maxImportance[r] = std::max(maxImportance[r], tilesImportance[r][i][j]);
					minImportance[r] = std::min(minImportance[r], tilesImportance[r][i][j]);
				}
				ONES2D[i][j] = 1;
			}
		}

		success = false;

		std::vector<doubleMatrix> E = allDistances;
		
		doubleMatrix criterionMatrix(rows, std::vector<double>(cols));

		while (termThr <= discr && !success && !canceled) {
			double downThres = (1.0 * tilesNum - 1.0 * termThr * (nr - 1)) / (1.0 * tilesNum * nr);
			double upperThres = (1.0 * tilesNum + termThr) / (1.0 * tilesNum * nr);
			success = true;
			int iter = 0;
			while (iter <= maxIter && !canceled) {
				assign(E);
				std::vector<floatMatrix> C(nr);
				std::vector<double> plainError(nr);
				std::vector<double> divFairError(nr);

				for (int r = 0; r < nr; ++r) {
					floatMatrix Cr = ONES2D;
					//for (int i = 0; i < rows; ++i)	Cr.push_back(ONES2D[i]);
					robotRegionsConnected[r] = true;

					ConnectComponent cc(rows, cols);
					intMatrix Ilabel = cc.compactLabeling(robotRegion[r], true);

					//printIntMat(Ilabel);

					if (cc.next_label > 1) {
						robotRegionsConnected[r] = false;
						cc.constructBinaryImages(Ilabel[robotInit[r].first][robotInit[r].second]);
						Cr = getMatrixCr(cc.normalizedEuclideanDistance(true), cc.normalizedEuclideanDistance(false));
						///printFloatMat(Cr);
					}
					//C.push_back(Cr);
					C[r] = Cr;

					plainError[r] = elements[r] / (1.0 * freeCells);
					if (plainError[r] < downThres) divFairError[r] = downThres - plainError[r];
					else if (plainError[r] > upperThres) divFairError[r] = upperThres - plainError[r];
				}

				if (isGoalState(termThr))	break;

				double totalNegPerc = 0.0, totalNegPlainError = 0.0;
				std::vector<double> correctionMult(nr);

				for (int r = 0; r < nr; ++r) {
					if (divFairError[r] < 0) {
						totalNegPerc += fabs(divFairError[r]);
						totalNegPlainError += plainError[r];
					}
					correctionMult[r] = 1.0;
				}

				for (int r = 0; r < nr; ++r) {
					if (totalNegPlainError != 0.0) {
						if (divFairError[r] < 0.0)
							correctionMult[r] = 1.0 + (plainError[r] / totalNegPlainError) * (totalNegPerc / 2.0);
						else
							correctionMult[r] = 1.0 - (plainError[r] / totalNegPlainError) * (totalNegPerc / 2.0);
					
						criterionMatrix = getCriterionMatrix(tilesImportance[r], minImportance[r], 
															 maxImportance[r], correctionMult[r], (divFairError[r] < 0));
					}
					
					//cout << "Criterion Matrix: " << endl;
					//printDoubleMat(criterionMatrix);

					E[r] = finalUpdateOnE(criterionMatrix, generateRandomMatrix(), E[r], C[r]);
				}

				iter++;
			}
			if (iter >= maxIter) {
				maxIter /= 2;
				success = false;
				termThr++;
			}
		}
		//
		getRobotRegion();
	}


	static void printIntMat(intMatrix& cur) {
		for (int i = cur.size() - 1; i >= 0; --i) {
			for (int j = 0; j < cur[0].size(); ++j)
				cout << cur[i][j] << " ";
			cout << endl;
		}
		cout << endl;
	}

	static void printFloatMat(floatMatrix& cur) {
		for (int i = 0; i < cur.size(); ++i) {
			for (int j = 0; j < cur[0].size(); ++j)
				cout << cur[i][j] << " ";
			cout << endl;
		}
		cout << endl;
	}

	static void printDoubleMat(doubleMatrix& cur) {
		for (int i = 0; i < cur.size(); ++i) {
			for (int j = 0; j < cur[0].size(); ++j)
				cout << cur[i][j] << " ";
			cout << endl;
		}
		cout << endl;
	}

	static void printBoolMat(boolMatrix& cur) {
		for (int i = 0; i < cur.size(); ++i) {
			for (int j = 0; j < cur[0].size(); ++j)
				cout << cur[i][j] << " ";
			cout << endl;
		}
		cout << endl;
	}
};
