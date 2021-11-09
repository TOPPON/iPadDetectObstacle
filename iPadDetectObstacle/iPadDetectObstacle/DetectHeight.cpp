#include "DetectHeight.h"
#include <iostream>
#include <fstream>

double DetectHeight::RotateGroundFlat(vector<MatrixXd> &flatGroundScanPoint, MatrixXd plane,int firstScanNum)
{

	cout << "plane:" << endl << plane << endl;
	double groundHeight = 0;//‰ñ“]Œã‚Ì˜H–Ê‚Ì‚‚³
	MatrixXd rotationGroundMatrix = MatrixXd::Zero(3, 3);//ŒX‚«˜H–Ê¨•½‚È˜H–Ê‚Ö‚Ì‰ñ“]s—ñ
	MatrixXd invRotationGroundMatrix = MatrixXd::Zero(3, 3);//ŒX‚«˜H–Ê¨•½‚È˜H–Ê‚Ö‚Ì‰ñ“]‹ts—ñ
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
	cout <<count+firstScanNum<<"”Ô–Ú‚Ì"<< "groundHeight:  " << groundHeight << endl;
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
Normalize‚µ‚Ä
X-1‚ð‹‚ß‚½‚ç
“_ŒQ‚ÉX-1‚ð‚·‚×‚ÄŠ|‚¯‚Ä
dL‚ð(0 0 z=-d/c)•ÏŠ·‚É‚æ‚è‹‚ß
ˆê‰ño—Í
‚¿‚á‚ñ‚ÆŽ²‚ªZ>0‚É‚È‚Á‚Ä‚¢‚é‚±‚Æ‚ðŠm”F‚·‚é
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
	heightdata.gridHeightMap = Eigen::MatrixXd::Constant(heightdata.mapSizeY, heightdata.mapSizeX, -1);
	heightdata.gridSize = gridSize;
	for (int i = 0; i < flatGroundScanPoint.size(); i++)
	{
		int tempx = (flatGroundScanPoint[i](0, 0) / gridSize - 0.5) + heightdata.centerX;
		int tempy = (flatGroundScanPoint[i](1, 0) / gridSize - 0.5) + heightdata.centerY;
		if (heightdata.gridHeightMap(tempy, tempx) == -1)
		{
			heightdata.gridHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
		else if (heightdata.gridHeightMap(tempy, tempx) < flatGroundScanPoint[i](2, 0))
		{
			heightdata.gridHeightMap(tempy, tempx) = flatGroundScanPoint[i](2, 0);
		}
	}
	//cout << "This.heightGridMap:" << heightdata.gridHeightMap << endl;
	return heightdata;
}

DetectHeight::HeightData::HeightData()
{
	gridSize = 1;
	gridHeightMap = MatrixXd::Zero(1, 1);
}
