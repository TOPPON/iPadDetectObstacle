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
	���ʂ�Ransac�Ő�������
	Input
	vector<MatrixXd(3,1)> &points:1�X�L�����̓_�Q
	bool exceptPoints:���o���ꂽ���ʂ̓_���폜����(true)�����Ȃ���(false)�@����Ȃ�points���폜�����
	double threshold:���ʂŔF�߂���i��(�}threshold),�P��m
	int tryNum:RANSAC�ŌJ��Ԃ���
	int scanCount:(�f�o�b�O�p)���݂̃X�L������

	Output
	MatrixXd(4,1) returnValue:�_���̈�ԑ������ʂ̎�(aX+bY+cZ+d=0)
	*/
private:
	int CalcPlaneScore(vector<MatrixXd> points, MatrixXd planeCoefficient,double threshold);
	/*
	CalcPlanePoint
	���ʂɏ]���_�����v�Z����
	Input
	vector<MatrixXd(3,1)> points:�_���𐔂���ΏۂƂȂ肤��_
	MatrixXd planeCoefficient:���ʂ̌W��(ax+by+cz+d=0)��(a,b,c,d))
	double threshold:���ʂŔF�߂��鍂���̍�

	Output
	int returnValue:���ʂɏ]���_�̐�
	
	*/
	void DeletePlanePoint(vector<MatrixXd>& points, MatrixXd planeCoefficient, double threshold,int scanCount);
	/*
	DeletePlanePoint
	���ʂɏ]���_���폜����
	Input
	vector<MatrixXd(3,1)>& points:�폜�Ώۂ̓_�A��������ԂŋA��
	MatrixXd planeCoefficient:���ʂ̌W��(ax+by+cz+d=0)��(a,b,c,d)
	double threshold:���ʂŔF�߂��鋗���̍�
	int scanCount:(�f�o�b�O�p)���݂̃X�L������
	
	Output
	vector<MatrixXd>& points:
	*/

};