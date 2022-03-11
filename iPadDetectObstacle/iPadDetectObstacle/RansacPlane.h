#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;
class RansacPlane
{
public:
	MatrixXd GuessPlane(vector<MatrixXd>& points, bool exceptPoints, double threshold, int trynum, int scanCount);
	/*
	GuessPlane
	平面をRansacで推測する
	Input
	vector<MatrixXd(3,1)> &points:1スキャンの点群
	bool exceptPoints:検出された平面の点を削除する(true)かしないか(false)　するならpointsが削除される
	double threshold:平面で認められる段差(±threshold),単位m
	int tryNum:RANSACで繰り返す回数
	int scanCount:(デバッグ用)現在のスキャン数

	Output
	MatrixXd(4,1) returnValue:点数の一番多い平面の式(aX+bY+cZ+d=0)
	*/
private:
	int CalcPlaneScore(vector<MatrixXd> points, MatrixXd planeCoefficient, double threshold);
	/*
	CalcPlanePoint
	平面に従う点数を計算する
	Input
	vector<MatrixXd(3,1)> points:点数を数える対象となりうる点
	MatrixXd planeCoefficient:平面の係数(ax+by+cz+d=0)の(a,b,c,d))
	double threshold:平面で認められる高さの差

	Output
	int returnValue:平面に従う点の数

	*/
	double CalcDistanceScore(vector<MatrixXd> points, MatrixXd PlaneCoefficient);
	/*
	CalcDistanceScore
	平面に従う点から距離の合計を計算する
	Input
	vector<MatrixXd> points:点数を数える対象となる点
	MatrixXd PlaneCoefficient:平面の係数(ax+by+cz+d=0)の(a,b,c,d)
	
	Output
	double returnValue:平均距離
	*/
	void OptimizePlane(vector<MatrixXd> points, MatrixXd oldPlaneCoefficient, double threshold,int tryNum, MatrixXd& newPlaneCoefficient);
	/*
	OptimizePlane
	求めた点群をRansacを用いて距離ベースで最適化する
	Input
	vector<MatrixXd> points:スキャンの全点
	MatrixXd oldPlaneCoefficient:最適化前の係数
	double threshold:平面で認められる距離の差
	int tryNum:Ransacの試行回数
	MatrixXd& newPlaneCoefficient:新規係数
	Output
	MatrixXd& newPlaneCoefficient:最適化後の係数
	*/
	void DeletePlanePoint(vector<MatrixXd>& points, MatrixXd planeCoefficient, double threshold, int scanCount);
	/*
	DeletePlanePoint
	平面に従う点を削除する
	Input
	vector<MatrixXd(3,1)>& points:削除対象の点、消えた状態で帰る
	MatrixXd planeCoefficient:平面の係数(ax+by+cz+d=0)の(a,b,c,d)
	double threshold:平面で認められる距離の差
	int scanCount:(デバッグ用)現在のスキャン数

	Output
	vector<MatrixXd>& points:削除後の点
	*/

};