#include "AppendixMethod.h"

double AppendixMethod::CalcMostNearPointDistance(vector<MatrixXd> &oneScanPoint)
{
	double mostNearDistance = 10000;//ƒ[ƒgƒ‹’PˆÊ
	for (int i = 0; i < oneScanPoint.size();i++)
	{
		double distance=sqrt(oneScanPoint[i](0, 0)*oneScanPoint[i](0, 0) + oneScanPoint[i](1, 0)*oneScanPoint[i](1, 0) + oneScanPoint[i](2, 0)*oneScanPoint[i](2, 0));
		if (distance < mostNearDistance)mostNearDistance = distance;
	}
	return mostNearDistance;
}