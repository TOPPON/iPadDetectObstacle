#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
using namespace Eigen;
using namespace std;
class RansacPlane
{
public:
	MatrixXd GuessPlane(vector<MatrixXd> &points, bool exceptPoints, double threshold,int trynum,int scanCount);
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
	int CalcPlaneScore(vector<MatrixXd> points, MatrixXd planeCoefficient,double threshold);
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
	void DeletePlanePoint(vector<MatrixXd>& points, MatrixXd planeCoefficient, double threshold,int scanCount);
	/*
	DeletePlanePoint
	平面に従う点を削除する
	Input
	vector<MatrixXd(3,1)>& points:削除対象の点、消えた状態で帰る
	MatrixXd planeCoefficient:平面の係数(ax+by+cz+d=0)の(a,b,c,d)
	double threshold:平面で認められる距離の差
	int scanCount:(デバッグ用)現在のスキャン数
	
	Output
	vector<MatrixXd>& points:
	*/

};