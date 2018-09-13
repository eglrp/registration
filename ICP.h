/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018��PCM�Ŷ� 
 *                      All rights reserved.
 * @Info   ��    ICP.H �ļ�ע��
 * @�ļ���  :    ICP.H
 * @�ļ�·��:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @����    :    Liuyang
 * @����ʱ��:    2018/8/11 9:46
 * @�ļ�����:    ����ICP�㷨�ĵ�����׼
 *********************************************************************/

#ifndef ICP_H
#define ICP_H

#include <Eigen/Eigen>

#include "point_types.h"

#include "FlannKDTreeShell.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    ��ע��
 *@  ClassName     :    ICP
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/14 14:04
 *@  Brief         :    �������ڵ��ƣ����о�ϸ��׼������任����
 *********************************************************************/
class ICP
{
  public:
	//���캯��
	ICP() : source_(),
			target_(),
			tree_(new FlannKDTree<wl::PointXYZ>),
			max_iterations_(10),
			approximate_threshold_(0.01),
			euclidean_fitness_epsilon_(1.0e-5),
			fitness_score_(0.0),
			converged_(false),
			iterations_(0),
			previous_transformation_(Eigen::Matrix4f::Identity()),
			transformation_(Eigen::Matrix4f::Identity()),
			final_transformation_(Eigen::Matrix4f::Identity())
	{
	}

	//������������
	void setInputSource(Points &points);

	//����Ŀ����������
	void setInputTarget(Points &points);

	//����Ŀ����Ƶ�KdTree
	void setTargetTree();

	//�Դ��������ƹ���KdTree���������ڽ���K���㣬�����㱾��
	bool searchForNeighbors(Points &points, std::vector<int> &indices, wl::PointXYZ &point, int k,
							std::vector<int> &k_indices, std::vector<float> &k_sqrdistances);

	//����任���뷽��
	double computeFitnessScore(Points &input_transfomed);

	//ȷ��ƥ��㣬�洢ƥ��������
	void getMatchPointsIndices(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2);

	//�ж�ƥ����Ƿ����
	bool isMatched(wl::PointXYZ p0, wl::PointXYZ p1, wl::PointXYZ p2, wl::PointXYZ q0, wl::PointXYZ q1, wl::PointXYZ q2);

	//����任����
	Eigen::Matrix4f computeTransformation(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2);

	//ʹ�ñ任����任points
	void transformPoints(Points &input, Points &output, Eigen::Matrix4f &tf);

	//�������ձ任����
	void align();

  public:
	//����ƥ������ƶ���ֵ
	inline void
	setApproximateThreshold(double approximate_threshold) { approximate_threshold_ = approximate_threshold; }

	//�����������α任������Ŀ����Ƶľ��뷽��֮��
	inline void
	setEuclideanFitnessEpsilon(double epsilon) { euclidean_fitness_epsilon_ = epsilon; }

	//���ó�ʼ�任����Ĭ��Ϊ��λ��
	inline void
	setPreviousTransformation(Eigen::Matrix4f guess) { previous_transformation_ = guess; }

	inline void
	setMaxIterations(int max_iterations) { max_iterations_ = max_iterations; }

	//��ȡ����������ľ��뷽��
	inline double
	getFitnessScore() { return fitness_score_; }

	//��ȡ���յ�����״̬
	inline bool
	hasconverged() { return converged_; }

	//��ȡ�����ɹ�ʱ��������
	inline int
	getIterations() { return iterations_; }

	//��ȡ���յı任����
	inline Eigen::Matrix4f
	getFinalTransformation() { return final_transformation_; }

  private:
	//��������������
	Points source_;

	//��������������
	Points target_;

	//������������������
	std::vector<int> source_indices_;

	//������������������
	std::vector<int> target_indices_;

	//��target_����KdTree
	FlannKDTree<wl::PointXYZ> *tree_;

	//����������
	int max_iterations_;

	//ƥ�����ƶ���ֵ
	double approximate_threshold_;

	//���뷽��֮��
	double euclidean_fitness_epsilon_;

	//��������ʱ���뷽��
	double fitness_score_;

	//�ж�����
	bool converged_;

	//��������
	int iterations_;

	//��ʼ�任����
	Eigen::Matrix4f previous_transformation_;

	//�����м�任����
	Eigen::Matrix4f transformation_;

	//���ձ任����
	Eigen::Matrix4f final_transformation_;
};

#endif