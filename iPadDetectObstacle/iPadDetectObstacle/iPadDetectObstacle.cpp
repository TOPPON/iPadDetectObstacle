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

#define FIRST_CALC_GRAVITY_SCANS 5
#define ONE_SCAN_POINT_NUMBER 49152
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
	//string filename = "test.txt";
	//ifstream inputFile(filename,std::ios::in);
	//inputFile.open("test.txt", ios::in);
	int scanCount = 0;//総スキャン数、姿勢角の行数をスキャン数とする
	vector<Posture> iPadPostures;
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

	//scanCountの数分点群データを読み込み
	printf("点群データ読み込み開始\n");
	vector<vector<MatrixXd>> alliPadCoordinatePoint;
	for (int i = 0; i < scanCount; i++)//スキャン数分csvデータがあるはずなので、読み込む
	{
		ifstream inputFile("InputData\\invpKscale" + to_string(i) + ".csv", std::ios::in);
		vector<MatrixXd> oneScaniPadCoordinatePoint;//1スキャンのiPad座標の点群
		while (getline(inputFile, strbuf))
		{
			istringstream i_strbuf(strbuf);
			int count = 0;
			MatrixXd temp(3, 1);
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
				case 3:
					temp(2, 0) = atof(strconmabuf.c_str());
					break;
				}
			}
			oneScaniPadCoordinatePoint.push_back(temp);
		}
		inputFile.close();
		alliPadCoordinatePoint.push_back(oneScaniPadCoordinatePoint);
		printf("%d個目終了", i);
	}
	printf("点群データ読み込み完了\n");
	//scanCount = 1;
	printf("点群データ回転・出力開始");
	vector<vector<MatrixXd>> allRotatediPadCoordinatePoint;//回転後の点群
	for (int i = 0; i < scanCount; i++)
	{
		//スキャンごとの回転行列を計算
		MatrixXd rollZ(3, 3);
		MatrixXd rollY(3, 3);
		MatrixXd rollX(3, 3);
		rollZ <<
			cos(iPadPostures[i].yaw), sin(iPadPostures[i].yaw), 0,
			-sin(iPadPostures[i].yaw), cos(iPadPostures[i].yaw), 0,
			0, 0, 1;
		rollY <<
			cos(iPadPostures[i].pitch), 0, -sin(iPadPostures[i].pitch),
			0, 1, 0,
			sin(iPadPostures[i].pitch), 0, cos(iPadPostures[i].pitch);
		rollX <<
			1, 0, 0,
			0, cos(iPadPostures[i].roll), sin(iPadPostures[i].roll),
			0, -sin(iPadPostures[i].roll), cos(iPadPostures[i].roll);
		cout << "ROLLX:" << rollX << endl;
		cout << "ROLLY:" << rollY << endl;
		cout << "ROLLZ:" << rollZ << endl;


		vector<MatrixXd> oneScanPeopleCoordinatePoint;//1スキャンの人座標のポイント数
		ofstream outputFile("OutputData\\test" + std::to_string(i) + ".csv", ios::out);
		for (int j = 0; j < ONE_SCAN_POINT_NUMBER; j++)
		{
			oneScanPeopleCoordinatePoint.push_back(rollX * rollY*rollZ*alliPadCoordinatePoint[i][j]);
			outputFile << oneScanPeopleCoordinatePoint[j](0, 0) << "," << oneScanPeopleCoordinatePoint[j](1, 0) << "," << oneScanPeopleCoordinatePoint[j](2, 0) << "," << endl;
		}
		outputFile.close();
		printf("Output:%d\n", i);
		allRotatediPadCoordinatePoint.push_back(oneScanPeopleCoordinatePoint);
	}
	printf("点群データ回転・出力終了");

	printf("平面推定開始");
	for (int i = 0; i < scanCount; i++)
	{
		cout << "-----------------" << i << "点目" << "---------------------"<<endl;
		vector<MatrixXd> lastPoint(allRotatediPadCoordinatePoint[i].size());//点をそのままコピー
		copy(allRotatediPadCoordinatePoint[i].begin(), allRotatediPadCoordinatePoint[i].end(), lastPoint.begin());
		RansacPlane rp;
		for (int j = 0; j < 5; j++)
		{
			if (lastPoint.size() > 2)
			{
				MatrixXd plane = rp.GuessPlane(lastPoint, true, 0.01, 500,i);
				cout << j << ":" << endl << plane << endl;
				cout << "lastpointSize:" << lastPoint.size()<<endl;
			}
		}
	}
	printf("平面推定終了");

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
