#include "RansacPlane.h"
#include "fstream"
#include <iostream>

MatrixXd RansacPlane::GuessPlane(std::vector<MatrixXd> &points, bool exceptPoints, double threshold, int tryNum,int scanCount)
{
	int maxScore = 0;
	MatrixXd maxPlaneCoefficient;
	for (int i = 0; i < tryNum; i++)
	{
		vector<int> plane3Point;
		//被りがないように三点を選ぶ
		for (int i = 0; i < 3; i++)
		{
			while (1)
			{
				bool isCheckConflictPoint = false;
				int tempPoint = rand() % points.size();
				for (int j = 0; j < plane3Point.size(); j++)
				{
					if (tempPoint == plane3Point[j])
					{
						isCheckConflictPoint = true;
						break;
					}
				}
				if (isCheckConflictPoint == false)
				{
					plane3Point.push_back(tempPoint);
					break;
				}
			}
		}
		//平面の係数を求める
		double a, b, c, d;//ax+by+cz+d=0の係数
		a = (points[plane3Point[1]](1, 0) - points[plane3Point[0]](1, 0))*(points[plane3Point[2]](2, 0) - points[plane3Point[0]](2, 0)) -
			(points[plane3Point[2]](1, 0) - points[plane3Point[0]](1, 0))*(points[plane3Point[1]](2, 0) - points[plane3Point[0]](2, 0));
		b = (points[plane3Point[1]](2, 0) - points[plane3Point[0]](2, 0))*(points[plane3Point[2]](0, 0) - points[plane3Point[0]](0, 0)) -
			(points[plane3Point[2]](2, 0) - points[plane3Point[0]](2, 0))*(points[plane3Point[1]](0, 0) - points[plane3Point[0]](0, 0));
		c = (points[plane3Point[1]](0, 0) - points[plane3Point[0]](0, 0))*(points[plane3Point[2]](1, 0) - points[plane3Point[0]](1, 0)) -
			(points[plane3Point[2]](0, 0) - points[plane3Point[0]](0, 0))*(points[plane3Point[1]](1, 0) - points[plane3Point[0]](1, 0));
		d = -a * points[plane3Point[0]](0, 0) - b * points[plane3Point[0]](1, 0) - c * points[plane3Point[0]](2, 0);


		MatrixXd planeCoefficient(4, 1);
		planeCoefficient << a, b, c, d;

		//平面
		int score = CalcPlaneScore(points, planeCoefficient, threshold);
		//cout << "score:" << score << endl;
		if (maxScore < score)
		{
			//cout << "highscore!!!--------------------" << score << endl << endl;
			maxScore = score;
			maxPlaneCoefficient = planeCoefficient;
		}
	}
	//チェック用に平面ファイルを出力
	//ofstream outputFile("Plane" + std::to_string(points.size()) + "-" + std::to_string(maxScore) + ".csv", ios::out);
	//
	if (exceptPoints)//検出された点を除く
	{
		DeletePlanePoint(points, maxPlaneCoefficient, threshold,scanCount);
	}
	else
	{

	}
	return maxPlaneCoefficient;
}

int RansacPlane::CalcPlaneScore(vector<MatrixXd> points, MatrixXd planeCoefficient, double threshold)
{
	int score = 0;
	double a = planeCoefficient(0, 0);
	double b = planeCoefficient(1, 0);
	double c = planeCoefficient(2, 0);
	double d = planeCoefficient(3, 0);
	for (int i = 0; i < points.size(); i++)
	{
		double distance = abs(a*points[i](0, 0) + b * points[i](1, 0) + c * points[i](2, 0) + d) / sqrt(a*a + b * b + c * c);
		if (distance < threshold)score++;
	}
	return score;
}

void RansacPlane::DeletePlanePoint(vector<MatrixXd>& points, MatrixXd planeCoefficient, double threshold,int scanCount)
{
	vector<MatrixXd> lastPoints;
	double a = planeCoefficient(0, 0);
	double b = planeCoefficient(1, 0);
	double c = planeCoefficient(2, 0);
	double d = planeCoefficient(3, 0);

	//チェック用に出力
	cout << "points.size():" << points.size() << endl;
	ofstream outputFile("Plane" +std::to_string(scanCount)+"_"+ std::to_string(points.size()) + ".csv", ios::out);
	for (int i = 0; i < points.size(); i++)
	{
		double distance = abs(a*points[i](0, 0) + b * points[i](1, 0) + c * points[i](2, 0) + d) / sqrt(a*a + b * b + c * c);
		if (distance >= threshold)
		{
			lastPoints.push_back(points[i]);
			//cout << "外point" << endl;
		}
		else
		{
			//cout << "distance:" << distance << endl;
			outputFile << points[i](0, 0) << "," << points[i](1, 0) << "," << points[i](2, 0) << endl;
		}
	}
	outputFile.close();
	points.clear();
	points.resize(lastPoints.size());
	cout << "lastPoints.size():" << lastPoints.size() << endl;
	std::copy(lastPoints.begin(), lastPoints.end(), points.begin());
	cout << "points.size():" << points.size() << endl;
}