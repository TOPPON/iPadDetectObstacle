#include "DetectHeight.h"
#include <iostream>
#include <fstream>

double DetectHeight::RotateGroundFlat(vector<MatrixXd> &flatGroundScanPoint, MatrixXd plane,int firstScanNum)
{

	cout << "plane:" << endl << plane << endl;
	double groundHeight = 0;//回転後の路面の高さ
	MatrixXd rotationGroundMatrix = MatrixXd::Zero(3, 3);//傾き路面→平な路面への回転行列
	MatrixXd invRotationGroundMatrix = MatrixXd::Zero(3, 3);//傾き路面→平な路面への回転逆行列
	if (plane(2, 0) < 0)plane *= -1;
	invRotationGroundMatrix <<
		plane(2, 0), plane(0, 0), plane(0, 0),
		plane(1, 0), plane(2, 0), plane(1, 0),
		-plane(0, 0), -plane(1, 0), plane(2, 0);
	rotationGroundMatrix = invRotationGroundMatrix.inverse();
	//cout << "rotationGroundMatrix:" << endl << rotationGroundMatrix << endl << "det:" << endl << rotationGroundMatrix.determinant() << endl;
	//rotationGroundMatrix.normalize();
	rotationGroundMatrix /= pow(rotationGroundMatrix.determinant(),0.333);
	//if (rotationGroundMatrix.determinant() < 0)rotationGroundMatrix *= -1;
	cout << "rotationGroundMatrix:" << endl << rotationGroundMatrix << endl << "det:" << endl << rotationGroundMatrix.determinant() << endl;

	MatrixXd aPlanePoint = MatrixXd::Zero(3, 1);
	if (plane(2, 0) != 0)
	{
		aPlanePoint(2, 0) = -plane(3, 0) / plane(2, 0);
		groundHeight = (rotationGroundMatrix*aPlanePoint)(2, 0);
	}
	cout <<count+firstScanNum<<"番目の"<< "groundHeight:  " << groundHeight << endl;
	ofstream outputFile("OutputData\\GroundFlatPoint\\GroundFlat" + std::to_string(count+firstScanNum) + ".csv", ios::out);
	for (int i = 0; i < flatGroundScanPoint.size(); i++)
	{
		MatrixXd beforePoint = flatGroundScanPoint[i];
		flatGroundScanPoint[i] = rotationGroundMatrix * beforePoint;
		flatGroundScanPoint[i](2, 0) -= groundHeight;
		outputFile << flatGroundScanPoint[i](0, 0) << "," << flatGroundScanPoint[i](1, 0) << "," << flatGroundScanPoint[i](2, 0) << "," << endl;
	}
	outputFile.close();
	count++;
	return groundHeight;

	/*
X=(c  a  a)
  (b  c  b)
  (-a -b c)
Normalizeして
X-1を求めたら
点群にX-1をすべて掛けて
d´を(0 0 z=-d/c)変換により求め
一回出力
ちゃんと軸がZ>0になっていることを確認する
*/
}
DetectHeight::HeightData DetectHeight::MappingGridHeight(vector<MatrixXd> &oneScanPoint, MatrixXd plane, double gridSize,int firstScanNum)
{
	vector<MatrixXd> flatGroundPoint(oneScanPoint.size());
	std::copy(oneScanPoint.begin(), oneScanPoint.end(), flatGroundPoint.begin());
	double groundHeight=RotateGroundFlat(flatGroundPoint, plane,firstScanNum);
	HeightData heightGridMap = MappingHeight(flatGroundPoint, gridSize);
	heightGridMap.groundHeight = groundHeight;
	return heightGridMap;
}
DetectHeight::HeightData DetectHeight::MappingHeight(vector<MatrixXd> &flatGroundScanPoint, double gridSize)
{
	HeightData heightdata;
	double maxX = -100 / gridSize;
	double maxY = -100 / gridSize;
	double minX = 100 / gridSize;
	double minY = 100 / gridSize;
	for (int i = 0; i < flatGroundScanPoint.size(); i++)
	{
		if (flatGroundScanPoint[i](0, 0) / gridSize < minX)
		{
			minX = flatGroundScanPoint[i](0, 0) / gridSize;
		}
		if (flatGroundScanPoint[i](1, 0) / gridSize < minY)
		{
			minY = flatGroundScanPoint[i](1, 0) / gridSize;
		}
		if (flatGroundScanPoint[i](0, 0) / gridSize > maxX)
		{
			maxX = flatGroundScanPoint[i](0, 0) / gridSize;
		}
		if (flatGroundScanPoint[i](1, 0) / gridSize > maxY)
		{
			maxY = flatGroundScanPoint[i](1, 0) / gridSize;
		}
	}
	heightdata.centerX = -(int)(minX - 0.5);
	heightdata.centerY = -(int)(minY - 0.5);
	heightdata.mapSizeX = (int)(maxX - 0.5) + heightdata.centerX + 1;
	heightdata.mapSizeY = (int)(maxY - 0.5) + heightdata.centerY + 1;
	//heightdata.gridHeightMap.resize(mapSizeX, mapSizeY);
	heightdata.gridMaxHeightMap = Eigen::MatrixXd::Constant(heightdata.mapSizeY, heightdata.mapSizeX, -1);
	heightdata.gridMinHeightMap = Eigen::MatrixXd::Constant(heightdata.mapSizeY, heightdata.mapSizeX, 1);
	heightdata.gridSize = gridSize;
	for (int i = 0; i < flatGroundScanPoint.size(); i++)
	{
		int tempx = (flatGroundScanPoint[i](0, 0) / gridSize - 0.5) + heightdata.centerX;
		int tempy = (flatGroundScanPoint[i](1, 0) / gridSize - 0.5) + heightdata.centerY;
		if (heightdata.gridMaxHeightMap(tempy, tempx) == -1)
		{
			heightdata.gridMaxHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
			heightdata.gridMinHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
		if (heightdata.gridMaxHeightMap(tempy, tempx) == 1)
		{
			heightdata.gridMinHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
		if (heightdata.gridMaxHeightMap(tempy, tempx) < flatGroundScanPoint[i](2, 0))
		{
			heightdata.gridMaxHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
		if (heightdata.gridMinHeightMap(tempy, tempx) > flatGroundScanPoint[i](2, 0))
		{
			heightdata.gridMinHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
	}
	//cout << "This.heightGridMap:" << heightdata.gridHeightMap << endl;
	return heightdata;
}

DetectHeight::HeightData::HeightData()
{
	gridSize = 1;
	gridMaxHeightMap = MatrixXd::Zero(1, 1);
	gridMinHeightMap = MatrixXd::Zero(1, 1);
}
