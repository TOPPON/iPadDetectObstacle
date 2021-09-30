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
	oneScanPoint�̒��Ō��_(0,0,0)�����ԋ߂��_�̋������v�Z���ĕԂ�
	Input
	vector<MatrixXd(3,1)> &oneScanPoint : ���_�����ԋ߂��_��T�����ƂȂ�_�Q�A���_�𒆐S�ɉ�]�������̂ł���Ύ�������Ă���

	Output
	double returnValue : oneScanPoint�̒��Ō��_�����ԋ߂��_�̋���(m)��Ԃ�
	*/
};