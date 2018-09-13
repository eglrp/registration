/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018��PCM�Ŷ� 
 *                      All rights reserved.
 * @Info   ��    RANSACLINE.H �ļ�ע��
 * @�ļ���  :    RANSACLINE.H
 * @�ļ�·��:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @����    :    Liuyang
 * @����ʱ��:    2018/8/9 21:07
 * @�ļ�����:    ����Ransac�㷨�ĵ�����������ȡ
 *********************************************************************/

#ifndef RANSACLINE_H
#define RANSACLINE_H

#include <vector>

#include <Eigen/Eigen>

#include "point_types.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    ��ע��
 *@  ClassName     :    RansacLine
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/9 21:13
 *@  Brief         :    ������Ƽ��������������������ϵ������
 *********************************************************************/
class RansacLine
{
  public:
	//���캯��
	RansacLine() : max_sample_checks_(1000),
				   choose_probability_(0.99),
				   outliers_probability_(0.9),
				   max_iterations_(std::numeric_limits<int>::max()),
				   threshold_(0.3),
				   iterations_(10)
	{
	}

	//�������ϵ���
	void setInputCloud(Points &points, std::vector<int> &indices);

	//��ȡ����ֱ��ģ�͵�����
	void getSample(std::vector<int> &sample);

	//��ȡ���ѡ������������
	void getSampleIndex(std::vector<int> &sample);

	//�ж�ֱ��ģ�������Ƿ�Ϊͬһ��
	bool isSample(std::vector<int> &sample);

	//����ģ�Ͳ���
	bool hasModelParameters(std::vector<int> sample, Eigen::VectorXf &model_parameters);

	//����ģ�����ھ�����ֵ�ڵ��������
	void getPointsOfRange(Eigen::VectorXf &model_parameters, double threshold, std::vector<int> &inliers, int &count);

	//�ж�ģ���Ƿ����
	bool isModel(Eigen::VectorXf &model_parameters);

	//����һ��ģ��
	void computeOneModel();

	//��������ģ��
	void computeAllModel();

  public:
	//���õ㵽ֱ�߾�����ֵ
	inline void setThreshold(double threshold) { threshold_ = threshold; }

	//���õ�������
	inline void setIterations(int iterations) { iterations_ = iterations; }

	//����choose_probability
	inline void setChooseProbability(double choose_probability) { choose_probability_ = choose_probability; }

	//����outliers_probability_
	inline void setOutliersProbability(double outliers_probability) { outliers_probability_ = outliers_probability; }

	//��ȡ���ڵ�
	inline std::vector<int> getInliers() { return inliers_; }

  private:
	//����ϵĵ���
	Points points_;

	//�������
	std::vector<int> indices_;

	//���������ĵ�
	std::vector<int> random_indices_;

	//Ѱ��һ��ģ������ģ�͵�Ĵ���
	int max_sample_checks_;

	//��������������M������ֱ�Ӽ���õ�
	int max_iterations_;

	//����M�β���֮������ֱ�߱�ѡ�еĸ��ʣ��ڷ���
	double choose_probability_;

	//����ֱ��֮��ĵ���ռ�����Ĺ���ֵ���ڷ�ĸ
	double outliers_probability_;

	//�㵽ֱ�ߵľ�����ֵ
	double threshold_;

	//RANSAC��������
	int iterations_;

	//�洢���ڵ�
	std::vector<int> inliers_;
};

#endif