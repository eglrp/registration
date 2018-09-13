/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018��PCM�Ŷ� 
 *                      All rights reserved.
 * /��Ϣ		:	FLANNKDTREESHELL.H �ļ�ע��
 * /�ļ���	: 	FLANNKDTREESHELL.H
 * /����		:	Caodi
 * /����ʱ��	:	2018/7/31 16:42
 * /�ļ�����	:	����flann�⹹��kd��
 *********************************************************************/
#pragma once

#include <flann/flann.hpp>

#include <vector>

using std::vector;

/* /
* /����	:	CFlannKDTree��ע��
* /����	:	����flann�⹹��kd��
* /����:	
* /������	:	
* /�����ռ�:	
* /����	:	Caodi
* /����	:	2018/7/31 20:41
* /˵��	:	
*/
template <typename PointT>
class FlannKDTree
{
  public:
	FlannKDTree();
	~FlannKDTree();

  public:
	//����KD-Tree
	bool BuildKDTree(vector<PointT> &pointCloud, vector<int> pointsIndex);

	//���ò��빹��kd����ά��
	void SetDimMark(bool xDimMark = true, bool yDimMark = true, bool zDimMark = true);

	//Set the search epsilon precision (error bound) for nearest neighbors searches.
	void setEpsilon(float eps);

	//�Ƿ���������������������ή���ٶ�
	void setSortedResults(bool sorted); //����û������

	//����������ֵ��Χ�ڵ���������㣬����������������������
	int radiusSearch(const PointT &point,			  //param[in] point a given \a valid (i.e., finite) query point
					 double radius,					  //param[in] radius the radius of the sphere bounding all of p_q's neighbors
					 std::vector<int> &k_indices,	 //param[out] k_indices the resultant indices of the neighboring points
					 std::vector<float> &k_distances, //param[out] k_distances the resultant squared distances to the neighboring points
					 unsigned int max_nn = 0) const;  //param[in] max_nn if given, bounds the maximum returned neighbors to this value. If \a max_nn is set to0 or to a number higher than the number of points in the input cloud, all neighbors in \a radius will be *returned.

	//����������K���ڽ��㣬����������������������
	int nearestKSearch(const PointT &point,					   //param[in] point a given \a valid (i.e., finite) query point
					   int k,								   //param[in] k the number of neighbors to search for
					   std::vector<int> &k_indices,			   //param[out] k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
					   std::vector<float> &k_distances) const; //param[out] k_distances the resultant squared distances to the neighboring points (must be resized to \a k* a priori!)

  private:
	//��������
	vector<PointT> m_pointCloud;

	//������ʽ�ĵ�������
	float *m_pointCloudMat;

	//falnn kdtreeָ��
	::flann::Index<flann::L2_Simple<float>> *m_kdTreePtr;

	//��ǲ��빹��kd����ά�ȣ�����ֻ��һ��������ά�Ȳ��빹��kd��
	bool m_dimMark[3];

	//��ǲ��빹��kd����ά�ȵ�����
	int m_dimCount;

	//eps precision (error bound) for nearest neighbors searches
	float m_epsilon;

	//brief Whether the searched neighbors by radius will be sorted.
	bool m_sorted;

	//
	::flann::SearchParams m_param_k;

	//
	::flann::SearchParams m_param_radius;
};

template <typename PointT>
FlannKDTree<PointT>::FlannKDTree()
{
	//��ʼ��
	m_pointCloudMat = nullptr;
	m_kdTreePtr = nullptr;
	for (int i = 0; i < 3; ++i)
	{
		m_dimMark[i] = true;
	}
	m_dimCount = 0;
	m_epsilon = 0.0f;
	m_sorted = false;
	m_param_k = ::flann::SearchParams(-1, m_epsilon);
	m_param_radius = ::flann::SearchParams(-1, m_epsilon, false);
}

template <typename PointT>
FlannKDTree<PointT>::~FlannKDTree()
{
	if (m_pointCloudMat != nullptr)
	{
		delete m_pointCloudMat;
		m_pointCloudMat = nullptr;
	}
	if (m_kdTreePtr != nullptr)
	{
		delete[] m_kdTreePtr;
		m_kdTreePtr = nullptr;
	}
}

template <typename PointT>
bool FlannKDTree<PointT>::BuildKDTree(vector<PointT> &pointCloud, vector<int> pointsIndex)
{
	m_pointCloud = pointCloud;

	//������빹��kd����ά������
	for (int i = 0; i < 3; ++i)
	{
		if (m_dimMark[i])
		{
			m_dimCount += 1;
		}
	}

	//std::cout << "���뽨����ά��������" << m_dimCount << std::endl;

	//Ϊ�洢�������ݵ��������ٿռ�
	m_pointCloudMat = new float[pointsIndex.size() * m_dimCount];

	//��������ת��
	if (m_dimCount == 3)
	{
		for (int i = 0; i < pointsIndex.size(); ++i)
		{
			m_pointCloudMat[i * m_dimCount] = pointCloud[pointsIndex[i]].x;
			m_pointCloudMat[i * m_dimCount + 1] = pointCloud[pointsIndex[i]].y;
			m_pointCloudMat[i * m_dimCount + 2] = pointCloud[pointsIndex[i]].z;
		}
	}
	else
	{
		for (int i = 0; i < pointsIndex.size(); ++i)
		{
			if (m_dimMark[0])
				m_pointCloudMat[i * m_dimCount] = pointCloud[pointsIndex[i]].x;
			if (m_dimMark[1])
				m_pointCloudMat[i * m_dimCount + 1] = pointCloud[pointsIndex[i]].y;
			if (m_dimMark[2])
				m_pointCloudMat[i * m_dimCount + 2] = pointCloud[pointsIndex[i]].z;
		}
	}

	//����kd��
	m_kdTreePtr = new ::flann::Index<::flann::L2_Simple<float>>(
		::flann::Matrix<float>(m_pointCloudMat, pointsIndex.size(), m_dimCount),
		::flann::KDTreeSingleIndexParams(15));

	m_kdTreePtr->buildIndex();

	//std::cout << "����KD�����" << m_dimCount << std::endl;

	return true;
}

template <typename PointT>
int FlannKDTree<PointT>::nearestKSearch(const PointT &point, int k, std::vector<int> &k_indices, std::vector<float> &k_distances) const
{
	//assert(point_representation_->isValid(point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");

	if (k > m_pointCloud.size())
	{
		k = m_pointCloud.size();
	}

	k_indices.resize(k);
	k_distances.resize(k);

	std::vector<float> query(m_dimCount);
	if (m_dimMark[0])
		query[0] = point.x;
	if (m_dimMark[1])
		query[1] = point.y;
	if (m_dimMark[2])
		query[2] = point.z;

	::flann::Matrix<int> k_indices_mat(&k_indices[0], 1, k);
	::flann::Matrix<float> k_distances_mat(&k_distances[0], 1, k);

	// Wrap the k_indices and k_distances vectors (no data copy)
	m_kdTreePtr->knnSearch(::flann::Matrix<float>(&query[0], 1, m_dimCount),
						   k_indices_mat,
						   k_distances_mat,
						   k,
						   m_param_k);

	//k_indices = k_distances_mat[0];
	//k_distances = k_distances_mat[0];

	return k;
}

template <typename PointT>
int FlannKDTree<PointT>::radiusSearch(const PointT &point, double radius, std::vector<int> &k_indices, std::vector<float> &k_distances, unsigned int max_nn) const
{
	//assert(point_representation_->isValid(point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!");

	std::vector<float> query(m_dimCount);
	if (m_dimMark[0])
		query[0] = point.x;
	if (m_dimMark[1])
		query[1] = point.y;
	if (m_dimMark[2])
		query[2] = point.z;

	// Has max_nn been set properly?
	if (max_nn == 0 || max_nn >= m_pointCloud.size())
		max_nn = m_pointCloud.size() - 1; // return all neighbors in radius

	std::vector<std::vector<int>> indices(1);
	std::vector<std::vector<float>> dists(1);

	int neighborsCountInRadius = m_kdTreePtr->radiusSearch(
		::flann::Matrix<float>(&query[0], 1, m_dimCount),
		indices,
		dists,
		static_cast<float>(radius * radius),
		m_param_radius);

	k_indices = indices[0];
	k_distances = dists[0];

	return neighborsCountInRadius;
}

template <typename PointT>
void FlannKDTree<PointT>::SetDimMark(bool xDimMark, bool yDimMark, bool zDimMark)
{
	//ȷ�����ٻ���һ��ά�ȵ����ݲ��빹��kd��
	assert(!(!xDimMark && !yDimMark && !zDimMark));

	//������
	m_dimMark[0] = xDimMark; //���X�Ƿ���뽨��
	m_dimMark[1] = yDimMark; //���Y�Ƿ���뽨��
	m_dimMark[2] = zDimMark; //���Z�Ƿ���뽨��
}

template <typename PointT>
void FlannKDTree<PointT>::setEpsilon(float eps)
{
	m_epsilon = eps;
	m_param_k = ::flann::SearchParams(-1, m_epsilon);
	m_param_radius = ::flann::SearchParams(-1, m_epsilon, m_sorted);
}

template <typename PointT>
void FlannKDTree<PointT>::setSortedResults(bool sorted)
{
	m_sorted = sorted;
	m_param_k = ::flann::SearchParams(-1, m_epsilon);
	m_param_radius = ::flann::SearchParams(-1, m_epsilon, m_sorted);
}