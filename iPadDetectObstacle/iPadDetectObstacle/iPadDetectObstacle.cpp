// iPadDetectObstacle.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>

#include "RansacPlane.h"
#include "DetectHeight.h"
#include "AppendixMethod.h"

#define FIRST_CALC_GRAVITY_SCANS 10
#define ONE_SCAN_POINT_NUMBER 49152
#define PLANE_THRESHOLD 0.05//地面を地面と認識する閾値(±m)
#define FIRST_DATA_NUM 3//スキャン数を指定して処理をする際の最初のスキャン数
#define LAST_DATA_NUM 6//スキャン数を指定して処理をする際の最後のスキャン数(含まない)
#define OBSTACLE_MIN_HEIGHT 0.1//障害物と認識する最低の高さ
#define MIN_ARCONFIDENCE_LEVEL 0//点の信頼度情報の最小値
#define MAX_ARCONFIDENCE_LEVEL 2//点の信頼度情報の最大値
#define GRID_MAP_SIZE 0.05
#define DATA_MODE 0//データの入力方法0:FIRST,LASTまで連続、1:実行の際に手入力、2:DATA_2_NUMで指定した6個
#define DATA_2_NUM_1 0
#define DATA_2_NUM_2 1
#define DATA_2_NUM_3 2
#define DATA_2_NUM_4 3
#define DATA_2_NUM_5 4
#define DATA_2_NUM_6 5

using namespace std;

using namespace Eigen;

struct Posture
{
	double roll;
	double pitch;
	double yaw;
	double vRoll;
	double vPitch;
	double vYaw;
	double accx;
	double accy;
	double accz;
	double timestamp;
};

int main()
{
	printf("IMU情報読み込み開始");
	ifstream inputFile("InputData\\timeAndIMU.csv", std::ios::in);
	int scanCount = 0;//総スキャン数、姿勢角の行数をスキャン数とする
	int firstScanNum = 0;//最初のスキャン数
	int lastScanNum = 0;//最後のスキャンとなるスキャン数
	vector<Posture> iPadPostures;
	vector<MatrixXd> groundPlanes;
	string strbuf;
	string strconmabuf;

	while (getline(inputFile, strbuf))
	{
		scanCount++;
		istringstream i_strbuf(strbuf);
		int postureitemcount = 0;
		Posture temp;
		while (getline(i_strbuf, strconmabuf, ','))//コンマ毎に切り分ける
		{
			switch (postureitemcount)
			{
			case 0://タイムスタンプ
				temp.timestamp = stod(strconmabuf);
				break;
			case 1://Roll角
				temp.roll = stod(strconmabuf);
				break;
			case 2://Pitch角
				temp.pitch = stod(strconmabuf);
				break;
			case 3://Yaw角
				temp.yaw = stod(strconmabuf);
				break;
			case 4://Roll角の各加速度
				temp.vRoll = stod(strconmabuf);
				break;
			case 5://Pitch角の角加速度
				temp.vPitch = stod(strconmabuf);
				break;
			case 6://Yaw角の角加速度
				temp.vYaw = stod(strconmabuf);
				break;
			case 7://X方向の加速度
				temp.accx = stod(strconmabuf);
				break;
			case 8://Y方向の加速度
				temp.accy = stod(strconmabuf);
				break;
			case 9://Z方向の加速度
				temp.accz = stod(strconmabuf);
				break;
			}
			postureitemcount++;
		}
		iPadPostures.push_back(temp);
	}
	printf("IMU情報読み込み終わり,データ数%d\n", scanCount);
	if (DATA_MODE == 1)
	{
		cout << "データを入力してください" << endl;
		cin>>firstScanNum;
		lastScanNum = firstScanNum;
	}
	else if(DATA_MODE==0)
	{
		if (FIRST_DATA_NUM > scanCount || FIRST_DATA_NUM > LAST_DATA_NUM)
		{
			cout << "エラー:コード内FIRSTが大きすぎます。";
			return 0;
		}
		firstScanNum = FIRST_DATA_NUM;
		lastScanNum = LAST_DATA_NUM;
		if (LAST_DATA_NUM > scanCount)
		{
			lastScanNum = scanCount - 1;
		}
		if (FIRST_DATA_NUM < 0)
		{
			firstScanNum = 0;
		}
	}
	else//
	{
		firstScanNum = 0;
		lastScanNum = 6;
	}

	int thisProcessScanCount = lastScanNum - firstScanNum + 1;
	if (DATA_MODE == 2)
	{
		thisProcessScanCount = scanCount;
	}
	//scanCountの数分点群データを読み込み
	printf("点群データ読み込み開始\n");
	vector<vector<MatrixXd>> alliPadCoordinatePoint;
	for (int i = 0; i < thisProcessScanCount; i++)//スキャン数分csvデータがあるはずなので、読み込む
	{
		if (DATA_MODE == 2)
		{
			if (i != DATA_2_NUM_1 && i != DATA_2_NUM_2 && i != DATA_2_NUM_3 && i != DATA_2_NUM_4 && i != DATA_2_NUM_5 && i != DATA_2_NUM_6) continue;
		}
		ifstream inputFile("InputData\\iPadScanPoint" + to_string(i + firstScanNum) + ".csv", std::ios::in);
		vector<MatrixXd> oneScaniPadCoordinatePoint;//1スキャンのiPad座標の点群
		while (getline(inputFile, strbuf))
		{
			istringstream i_strbuf(strbuf);
			int count = 0;
			MatrixXd temp(4, 1);
			while (getline(i_strbuf, strconmabuf, ','))
			{
				count++;
				switch (count)
				{
				case 1://タイムスタンプ
					temp(0, 0) = atof(strconmabuf.c_str());
					break;
				case 2:
					temp(1, 0) = atof(strconmabuf.c_str());
					break;
				case 3://Z座標だけ反転して読み込むことで下向きになる(高さがマイナスにならないように)
					temp(2, 0) = -atof(strconmabuf.c_str());
					break;
				case 4://信頼度
					temp(3, 0) = atoi(strconmabuf.c_str());
				}
			}
			if (temp(3, 0) >= MIN_ARCONFIDENCE_LEVEL & temp(3, 0) <= MAX_ARCONFIDENCE_LEVEL)
			{
				oneScaniPadCoordinatePoint.push_back(temp.block(0, 0, 3, 1));
			}
		}
		inputFile.close();
		alliPadCoordinatePoint.push_back(oneScaniPadCoordinatePoint);
		printf("%d個目終了", i);
	}
	printf("点群データ読み込み完了\n");
	if(DATA_MODE==2) thisProcessScanCount=alliPadCoordinatePoint.size();
	printf("点群データ回転・出力開始");
	vector<vector<MatrixXd>> allRotatediPadCoordinatePoint;//回転後の点群(iPad座標系)
	for (int i = 0; i < thisProcessScanCount; i++)
	{
		//スキャンごとの回転行列を計算
		MatrixXd rollZ(3, 3);
		MatrixXd rollY(3, 3);
		MatrixXd rollX(3, 3);
		//いけたやつ　ZYXの順番
		rollZ <<
			cos(iPadPostures[i + firstScanNum].yaw), -sin(iPadPostures[i + firstScanNum].yaw), 0,//正方向回転
			sin(iPadPostures[i + firstScanNum].yaw), cos(iPadPostures[i + firstScanNum].yaw), 0,
			0, 0, 1;
		rollY <<
			cos(iPadPostures[i + firstScanNum].pitch), 0, sin(iPadPostures[i + firstScanNum].pitch),
			0, 1, 0,
			-sin(iPadPostures[i + firstScanNum].pitch), 0, cos(iPadPostures[i + firstScanNum].pitch);//正方向回転

		rollX <<
			1, 0, 0,
			0, cos(iPadPostures[i + firstScanNum].roll), -sin(iPadPostures[i + firstScanNum].roll),
			0, sin(iPadPostures[i + firstScanNum].roll), cos(iPadPostures[i + firstScanNum].roll);//正方向回転

		cout << "ROLLX:" << rollX << endl;
		cout << "ROLLY:" << rollY << endl;
		cout << "ROLLZ:" << rollZ << endl;

		vector<MatrixXd> oneScanPeopleCoordinatePoint;//1スキャンの人座標のポイント数
		ofstream outputFile("OutputData\\NormalizedPoint\\Normalize" + std::to_string(i + firstScanNum) + ".csv", ios::out);
		printf("%d番目:%d\n", i + firstScanNum, alliPadCoordinatePoint[i].size());
		for (int j = 0; j < /*ONE_SCAN_POINT_NUMBER*/alliPadCoordinatePoint[i].size(); j++)
		{
			if (abs(alliPadCoordinatePoint[i][j](2, 0)) > 30 | abs(alliPadCoordinatePoint[i][j](1, 0)) > 30)
			{
				cout << "error!" << endl;
			}
			MatrixXd onePoint(3, 1);
			onePoint = rollZ * rollY * rollX * alliPadCoordinatePoint[i][j];
			oneScanPeopleCoordinatePoint.push_back(onePoint);
			if (abs(oneScanPeopleCoordinatePoint[j](2, 0)) > 30 | abs(oneScanPeopleCoordinatePoint[j](1, 0)) > 30)
			{
				cout << "alliPadCoordinatePoint[i][j]" << alliPadCoordinatePoint[i][j] << endl << endl;
				cout << "onePoint" << onePoint << endl << endl;
				cout << "onescanPeople:" << oneScanPeopleCoordinatePoint[j] << endl << "rollZ * rollY * rollX * alliPadCoordinatePoint[i][j]:" << rollZ * rollY * rollX * alliPadCoordinatePoint[i][j] << endl;
				cout << "error!" << endl;
				return -1;
			}
			outputFile << oneScanPeopleCoordinatePoint[j](0, 0) << "," << oneScanPeopleCoordinatePoint[j](1, 0) << "," << oneScanPeopleCoordinatePoint[j](2, 0) << "," << endl;
		}
		outputFile.close();
		printf("Output:%d\n", i + firstScanNum);
		allRotatediPadCoordinatePoint.push_back(oneScanPeopleCoordinatePoint);
	}
	printf("点群データ回転・出力終了");
	//alliPadCoordinatePoint.clear();

	printf("平面推定開始");
	vector<vector<MatrixXd>> planes;
	for (int i = 0; i < thisProcessScanCount; i++)
	{
		cout << "-----------------" << i + firstScanNum << "点目" << "---------------------" << endl;
		vector<MatrixXd> lastPoint(allRotatediPadCoordinatePoint[i].size());//点をそのままコピー
		copy(allRotatediPadCoordinatePoint[i].begin(), allRotatediPadCoordinatePoint[i].end(), lastPoint.begin());
		RansacPlane rp;
		vector<MatrixXd> tempPlane;
		for (int j = 0; j < 5; j++)
		{
			if (lastPoint.size() > 2)
			{
				MatrixXd plane = rp.GuessPlane(lastPoint, true, PLANE_THRESHOLD, 10000, i);
				cout << j << ":" << endl << plane << endl;
				//cout << "lastpointSize:" << lastPoint.size() << endl;
				tempPlane.push_back(plane);
			}
			else if (j == 0)//点群がそもそも2点以下のとき
			{
				MatrixXd plane(4, 1);
				plane << -1, -1, -1, -1;
				printf("有効な点がほぼありません\n");
				tempPlane.push_back(plane);
			}
		}
		planes.push_back(tempPlane);
	}
	printf("平面推定終了");

	printf("路面検出開始");
	MatrixXd gravity(1, 3);
	gravity << 0, 0, 1;
	for (int i = 0; i < thisProcessScanCount; i++)//点が0点になることを考慮する
	{
		cout << "-----------------" << i + firstScanNum << "点目" << "---------------------" << endl;
		for (int j = 0; j < planes[i].size(); j++)
		{
			if (planes[i][j](0, 0) == -1 && planes[i][j](1, 0) == -1 && planes[i][j](2, 0) == -1 && planes[i][j](3, 0) == -1)
			{
				if (j == planes[i].size() - 1)//路面推定失敗
				{
					cout << "路面推定失敗!スキップします" << endl;
					groundPlanes.push_back(gravity);
					break;
				}
			}
			MatrixXd Normal(3, 1);
			Normal(0, 0) = planes[i][j](0, 0);
			Normal(1, 0) = planes[i][j](1, 0);
			Normal(2, 0) = planes[i][j](2, 0);
			Normal.normalize();
			cout << "平面" << j << ":" << abs((gravity * Normal)(0, 0)) << endl;
			if (abs((gravity * Normal)(0, 0)) > sqrt(3) / 2)//内積の絶対値がsqrt(3)/2以上の場合(平面の法線と重力方向のなす角の最大角が30度以下の場合)平面と判断
			{
				cout << "平面は" << planes[i][j] << "です" << endl;
				groundPlanes.push_back(planes[i][j]);
				break;
			}
			if (j == planes[i].size() - 1)//路面推定失敗
			{
				cout << "路面推定失敗!スキップします" << endl;
				groundPlanes.push_back(gravity);
			}
		}
	}
	printf("路面検出終了");

	printf("高さ検出開始");//路面座標に変換→グリッドマップにマッピング→高さ検出
	DetectHeight dh;
	dh.count = 0;
	vector<float> ObstacleMaxDistanceList;
	vector<float> ObstacleMaxHeightList;
	vector<float> ObstacleMinDistanceList;
	vector<float> ObstacleMinHeightList;
	for (int i = 0; i < thisProcessScanCount; i++)
	{
		cout << "-----------------" << i + firstScanNum << "点目" << "---------------------" << endl;
		//const double obstacleThreshold = 0.5;//50cm以上のものを障害物と認識
		if (groundPlanes[i].rows() == 4)
		{
			double minUpDistance = 1000;
			double minDownDistance = 1000;
			double obstacleMaxHeight = 0;
			double obstacleMinHeight = 0;
			DetectHeight::HeightData heightData = dh.MappingGridHeight(allRotatediPadCoordinatePoint[i], groundPlanes[i], GRID_MAP_SIZE, firstScanNum);
			ofstream outputFile("OutputData\\HeightMap\\Height" + std::to_string(i + firstScanNum) + ".csv", ios::out);
			for (int j = 0; j < heightData.mapSizeY; j++)
			{
				for (int k = 0; k < heightData.mapSizeX; k++)
				{
					outputFile << heightData.gridMaxHeightMap(j, k) << ",";
					if (heightData.gridMaxHeightMap(j, k) > OBSTACLE_MIN_HEIGHT)
					{
						double thisDistance = sqrt((heightData.gridMaxHeightMap(j, k) + heightData.groundHeight) * (heightData.gridMaxHeightMap(j, k) + heightData.groundHeight)
							+ (k + 1 - heightData.centerX) * heightData.gridSize * (k + 1 - heightData.centerX) * heightData.gridSize +
							(j + 1 - heightData.centerY) * heightData.gridSize * (j + 1 - heightData.centerY) * heightData.gridSize);
						if (minUpDistance > thisDistance)
						{
							minUpDistance = thisDistance;
							obstacleMaxHeight = heightData.gridMaxHeightMap(j, k);
						}
					}
				}
				outputFile << endl;
			}
			outputFile << endl << endl;
			for (int j = 0; j < heightData.mapSizeY; j++)
			{
				for (int k = 0; k < heightData.mapSizeX; k++)
				{
					outputFile << heightData.gridMinHeightMap(j, k) << ",";
					if (heightData.gridMinHeightMap(j, k) < -OBSTACLE_MIN_HEIGHT)
					{
						double thisDistance = sqrt((heightData.gridMinHeightMap(j, k) + heightData.groundHeight) * (heightData.gridMinHeightMap(j, k) + heightData.groundHeight)
							+ (k + 1 - heightData.centerX) * heightData.gridSize * (k + 1 - heightData.centerX) * heightData.gridSize +
							(j + 1 - heightData.centerY) * heightData.gridSize * (j + 1 - heightData.centerY) * heightData.gridSize);
						if (minDownDistance > thisDistance)
						{
							minDownDistance = thisDistance;
							obstacleMinHeight = heightData.gridMinHeightMap(j, k);
						}
					}
				}
				outputFile << endl;
			}
			outputFile << "centerx:," << heightData.centerX << "," << "centery:," << heightData.centerY << ",groundHeight" << heightData.groundHeight << "\n";
			//原点(iPad)の座標や障害物の高さを検出
			if (obstacleMaxHeight > OBSTACLE_MIN_HEIGHT)
			{
				printf("%fmの距離に%fcmの高さの障害物あり\n", minUpDistance, obstacleMaxHeight * 100);
				outputFile << "障害物の距離:," << minUpDistance << "," << "高さ:," << obstacleMaxHeight << ",";
				ObstacleMaxDistanceList.push_back(minUpDistance);
				ObstacleMaxHeightList.push_back(obstacleMaxHeight);
			}
			else
			{
				ObstacleMaxDistanceList.push_back(0);
				ObstacleMaxHeightList.push_back(0);
			}
			if (obstacleMinHeight < -OBSTACLE_MIN_HEIGHT)
			{
				printf("%fmの距離に%fcmの高さの障害物あり\n", minDownDistance, obstacleMinHeight * 100);
				outputFile << "障害物の距離:," << minDownDistance << "," << "高さ:," << obstacleMinHeight << ",";
				ObstacleMinDistanceList.push_back(minDownDistance);
				ObstacleMinHeightList.push_back(obstacleMinHeight);
			}
			else
			{
				ObstacleMinDistanceList.push_back(0);
				ObstacleMinHeightList.push_back(0);
			}
			outputFile.close();
		}
		else
		{
			cout << "以前のエラーのためスキップします" << endl;
			double mostNearPointDistance = AppendixMethod::CalcMostNearPointDistance(alliPadCoordinatePoint[i]);
			cout << "一番近い物体は" << mostNearPointDistance << "mのところにあります" << endl;
			ofstream outputFile("OutputData\\HeightMap\\Height" + std::to_string(i + firstScanNum) + ".csv", ios::out);
			outputFile << "点群の高さ計算失敗！" << endl << "一番近い点までの距離:," << mostNearPointDistance << endl;
			ObstacleMaxDistanceList.push_back(mostNearPointDistance);
			ObstacleMaxHeightList.push_back(-1);
			ObstacleMinDistanceList.push_back(mostNearPointDistance);
			ObstacleMinHeightList.push_back(1);
			dh.count++;
		}
	}
	ofstream outputFile("OutputData\\HeightMap\\Obstacle" + to_string(firstScanNum) + "-" + to_string(firstScanNum + thisProcessScanCount - 1) + ".csv", ios::out);
	for (int i = 0; i < thisProcessScanCount; i++)
	{
		outputFile << i + firstScanNum << ",上の距離," << ObstacleMaxDistanceList[i] << ",高さ," << ObstacleMaxHeightList[i] << ",下の距離," << ObstacleMinDistanceList[i] << ",高さ," << ObstacleMinHeightList[i] << endl;
	}
	outputFile.close();
	printf("高さ検出終了");

}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
