/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018��PCM�Ŷ� 
 *                      All rights reserved.
 * @Info   ��    PCA.H �ļ�ע��
 * @�ļ���  :    PCA.H
 * @�ļ�·��:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @����    :    Liuyang
 * @����ʱ��:    2018/8/22 19:23
 * @�ļ�����:    ����PCAУ����������ϵ
 *********************************************************************/
#pragma once

#include <Eigen/Eigen>

#include "FlannKDTreeShell.h"

#define PI 3.1415
/********************************************************************
 *@  Info          :    ��ע��
 *@  ClassName     :    PCA
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/22 19:25
 *@  Brief         :    ʵ��PCA�㷨����ȡ����ϵ��������������תƽ�ƾ���
 *********************************************************************/
template <typename PointT>
class PCA
{
  public:
	void runPCA(std::vector<PointT> &points1, std::vector<PointT> &points2);

	void computeEigenVectors(std::vector<PointT> &points, Eigen::Matrix3f &ve, Eigen::Vector3f &box);

	void computeTransformation();

	inline Eigen::Matrix4f getTransformation() { return transformation_; }

  private:
	std::vector<PointT> points1_;
	std::vector<PointT> points2_;

	Eigen::Matrix4f transformation_;
};

/*****************************************************************************
* @brief   : �����������������ݣ�����任����
* @author  : Liuyang
* @date    : 2018/8/22 19:26
* @version : ver 1.0
* @inparam : points1 ��ʼ���ƣ�points2 Ŀ�����
* @outparam: 
* @�޶�˵��: 
*****************************************************************************/
template <typename PointT>
void PCA<PointT>::runPCA(std::vector<PointT> &points1, std::vector<PointT> &points2)
{
	points1_ = points1;
	points2_ = points2;

	computeTransformation();
}
/*****************************************************************************
* @brief   : ����㼯�ϵ���������
* @author  : Liuyang
* @date    : 2018/8/22 19:26
* @version : ver 1.0
* @inparam : points ����
* @outparam: ve �������Ტ������ֵ�Ĵ�С�����������box ��Χ������ֵ
* @�޶�˵��: 
*****************************************************************************/
template <typename PointT>
void PCA<PointT>::computeEigenVectors(std::vector<PointT> &points, Eigen::Matrix3f &ve, Eigen::Vector3f &box)
{
	int npts = points.size();

	std::vector<int> indices;
	indices.resize(npts);

	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud(3, npts);

	for (int i = 0; i < npts; ++i)
	{
		cloud(0, i) = points[i].x;
		cloud(1, i) = points[i].y;
		cloud(2, i) = points[i].z;

		indices.emplace_back(i);
	}

	Eigen::Vector3f min = cloud.rowwise().minCoeff();
	Eigen::Vector3f max = cloud.rowwise().maxCoeff();
	box << 0.5 * (min(0) + max(0)), 0.5 * (min(1) + max(1)), 0.5 * (min(2) + max(2));

	Eigen::Vector3f mean = cloud.rowwise().mean();

	Eigen::Matrix3f cov = ((cloud.colwise() - mean) * (cloud.colwise() - mean).transpose()) / (static_cast<float>(npts - 1));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov, Eigen::ComputeEigenvectors);
	ve = es.eigenvectors(); //�൱��U
	//Eigen::Matrix<float, 3, 1> va = es.eigenvalues();

	PointT searchpoint;
	searchpoint.x = box(0);
	searchpoint.y = box(1);
	searchpoint.z = box(2);

	FlannKDTree<PointT> tree;
	vector<int> k_indices;
	vector<float> k_distances;
	tree.BuildKDTree(points, indices);
	tree.setSortedResults(true);
	tree.nearestKSearch(searchpoint, npts, k_indices, k_distances);
	Eigen::Vector3f far;
	far << points[k_indices[npts - 1]].x - box(0), points[k_indices[npts - 1]].y - box(1), points[k_indices[npts - 1]].z - box(2);
	far.normalize();

	for (int i = 2; i > 0; --i)
	{
		if ((acos(ve.col(i).dot(far)) * 180 / PI) > 90)
		{
			ve.col(i) = -ve.col(i);
		}
	}
	ve.col(0) = ve.col(2).cross(ve.col(1));

	std::vector<int>().swap(indices);
	std::vector<int>().swap(k_indices);
	std::vector<float>().swap(k_distances);
}
/*****************************************************************************
* @brief   : �ɳ�ʼ���ƺ�Ŀ����Ƶ��������任����
* @author  : Liuyang
* @date    : 2018/8/22 19:26
* @version : ver 1.0
* @inparam : 
* @outparam: 
* @�޶�˵��: 
*****************************************************************************/
template <typename PointT>
void PCA<PointT>::computeTransformation()
{
	if (points1_.empty() || points2_.empty())
	{
		return;
	}

	Eigen::Matrix3f ve1;
	Eigen::Matrix3f ve2;
	Eigen::Vector3f box1;
	Eigen::Vector3f box2;

	computeEigenVectors(points1_, ve1, box1);
	computeEigenVectors(points2_, ve2, box2);

	Eigen::Matrix3f R = ve2 * ve1.transpose();
	Eigen::Vector3f T = box2 - R * box1;

	transformation_ = Eigen::Matrix4f::Identity();
	transformation_.block<3, 3>(0, 0) = R;
	transformation_.block<3, 1>(0, 3) = T;
}