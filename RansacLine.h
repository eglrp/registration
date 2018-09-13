/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018，PCM团队 
 *                      All rights reserved.
 * @Info   ：    RANSACLINE.H 文件注释
 * @文件名  :    RANSACLINE.H
 * @文件路径:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @作者    :    Liuyang
 * @创建时间:    2018/8/9 21:07
 * @文件描述:    基于Ransac算法的点云特征线提取
 *********************************************************************/

#ifndef RANSACLINE_H
#define RANSACLINE_H

#include <vector>

#include <Eigen/Eigen>

#include "point_types.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    类注释
 *@  ClassName     :    RansacLine
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/9 21:13
 *@  Brief         :    输入点云及其索引，输出特征线拟合点的索引
 *********************************************************************/
class RansacLine
{
  public:
	//构造函数
	RansacLine() : max_sample_checks_(1000),
				   choose_probability_(0.99),
				   outliers_probability_(0.9),
				   max_iterations_(std::numeric_limits<int>::max()),
				   threshold_(0.3),
				   iterations_(10)
	{
	}

	//传入待拟合点云
	void setInputCloud(Points &points, std::vector<int> &indices);

	//获取构成直线模型的两点
	void getSample(std::vector<int> &sample);

	//获取随机选择的两点的索引
	void getSampleIndex(std::vector<int> &sample);

	//判断直线模型两点是否为同一点
	bool isSample(std::vector<int> &sample);

	//计算模型参数
	bool hasModelParameters(std::vector<int> sample, Eigen::VectorXf &model_parameters);

	//计算模型下在距离阈值内的邻域点数
	void getPointsOfRange(Eigen::VectorXf &model_parameters, double threshold, std::vector<int> &inliers, int &count);

	//判断模型是否合理
	bool isModel(Eigen::VectorXf &model_parameters);

	//计算一个模型
	void computeOneModel();

	//计算所有模型
	void computeAllModel();

  public:
	//设置点到直线距离阈值
	inline void setThreshold(double threshold) { threshold_ = threshold; }

	//设置迭代次数
	inline void setIterations(int iterations) { iterations_ = iterations; }

	//设置choose_probability
	inline void setChooseProbability(double choose_probability) { choose_probability_ = choose_probability; }

	//设置outliers_probability_
	inline void setOutliersProbability(double outliers_probability) { outliers_probability_ = outliers_probability; }

	//获取局内点
	inline std::vector<int> getInliers() { return inliers_; }

  private:
	//待拟合的点云
	Points points_;

	//点的索引
	std::vector<int> indices_;

	//随机排序过的点
	std::vector<int> random_indices_;

	//寻找一个模型搜索模型点的次数
	int max_sample_checks_;

	//计算最大迭代次数M，可以直接计算得到
	int max_iterations_;

	//经过M次采样之后最优直线被选中的概率，在分子
	double choose_probability_;

	//最优直线之外的点所占比例的估计值，在分母
	double outliers_probability_;

	//点到直线的距离阈值
	double threshold_;

	//RANSAC迭代次数
	int iterations_;

	//存储局内点
	std::vector<int> inliers_;
};

#endif