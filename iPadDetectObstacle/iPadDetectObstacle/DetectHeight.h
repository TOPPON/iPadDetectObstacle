#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

class DetectHeight
{
public:
	int count;//何回呼ばれたか
	struct HeightData
	{
		HeightData();
		MatrixXd gridHeightMap;//-1ならセルはなし
		double gridSize;//gridの大きさ
		int centerX;
		int centerY;
		int mapSizeX;
		int mapSizeY;
		double groundHeight;
	};
	/*
	HeightData
		一スキャンの二次元ボクセル=(ピクセル？)毎の高さ情報を格納するデータ型
		MatrixXd gridHeightMap : グリッド毎の高さ、-1ならこのグリッドに点群はなし
		double gridSize : グリッドの大きさ,単位メートル
		int centerX : 中心(0,0)となる点のX方向のindex
		int centerY : 中心(0,0)となる点のY方向のindex
		double groundHeight : 地面の高さ、単位メートル
	*/
	HeightData MappingGridHeight(vector<MatrixXd> &oneScanPoint, MatrixXd plane, double gridSize, int firstScanNum);
	/*
	MappingGridHeight
		点のPlaneからの高さをグリッドマップにマッピングして計算する
		Input
		vector<MatrixXd(3,1)> &oneScanPoint:1スキャンの点群
		MatrixXd plane : 路面の係数
		double gridSize : グリッドマップのグリッドのサイズ、メートル単位

		Output
		HeightData returnValue : それぞれのグリッドマップの一番高い点を格納したデータ
	*/
private:
	double RotateGroundFlat(vector<MatrixXd> &flatGroundScanPoint, MatrixXd plane,int firstScanNum);//点群を路面とぴったり合うように回転し、路面の高さを0になるよう変換する、平面の高さを返す
	HeightData MappingHeight(vector<MatrixXd> &flatGroundScanPoint,double gridSize);//回転された点群をHeightDataに格納する
};