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
	int count;//����Ă΂ꂽ��
	struct HeightData
	{
		HeightData();
		MatrixXd gridHeightMap;//-1�Ȃ�Z���͂Ȃ�
		double gridSize;//grid�̑傫��
		int centerX;
		int centerY;
		int mapSizeX;
		int mapSizeY;
		double groundHeight;
	};
	/*
	HeightData
		��X�L�����̓񎟌��{�N�Z��=(�s�N�Z���H)���̍��������i�[����f�[�^�^
		MatrixXd gridHeightMap : �O���b�h���̍����A-1�Ȃ炱�̃O���b�h�ɓ_�Q�͂Ȃ�
		double gridSize : �O���b�h�̑傫��,�P�ʃ��[�g��
		int centerX : ���S(0,0)�ƂȂ�_��X������index
		int centerY : ���S(0,0)�ƂȂ�_��Y������index
		double groundHeight : �n�ʂ̍����A�P�ʃ��[�g��
	*/
	HeightData MappingGridHeight(vector<MatrixXd> &oneScanPoint, MatrixXd plane, double gridSize, int firstScanNum);
	/*
	MappingGridHeight
		�_��Plane����̍������O���b�h�}�b�v�Ƀ}�b�s���O���Čv�Z����
		Input
		vector<MatrixXd(3,1)> &oneScanPoint:1�X�L�����̓_�Q
		MatrixXd plane : �H�ʂ̌W��
		double gridSize : �O���b�h�}�b�v�̃O���b�h�̃T�C�Y�A���[�g���P��

		Output
		HeightData returnValue : ���ꂼ��̃O���b�h�}�b�v�̈�ԍ����_���i�[�����f�[�^
	*/
private:
	double RotateGroundFlat(vector<MatrixXd> &flatGroundScanPoint, MatrixXd plane,int firstScanNum);//�_�Q��H�ʂƂ҂����荇���悤�ɉ�]���A�H�ʂ̍�����0�ɂȂ�悤�ϊ�����A���ʂ̍�����Ԃ�
	HeightData MappingHeight(vector<MatrixXd> &flatGroundScanPoint,double gridSize);//��]���ꂽ�_�Q��HeightData�Ɋi�[����
};