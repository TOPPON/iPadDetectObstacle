#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

class AppendixMethod
{
public:
	static double CalcMostNearPointDistance(vector<MatrixXd> &oneScanPoint);
	/*CalcMostNearPointDistance
	oneScanPointの中で原点(0,0,0)から一番近い点の距離を計算して返す
	Input
	vector<MatrixXd(3,1)> &oneScanPoint : 原点から一番近い点を探す元となる点群、原点を中心に回転したものであれば手を加えても可

	Output
	double returnValue : oneScanPointの中で原点から一番近い点の距離(m)を返す
	*/
};