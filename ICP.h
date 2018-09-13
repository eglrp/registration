/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018，PCM团队 
 *                      All rights reserved.
 * @Info   ：    ICP.H 文件注释
 * @文件名  :    ICP.H
 * @文件路径:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @作者    :    Liuyang
 * @创建时间:    2018/8/11 9:46
 * @文件描述:    基于ICP算法的点云配准
 *********************************************************************/

#ifndef ICP_H
#define ICP_H

#include <Eigen/Eigen>

#include "point_types.h"

#include "FlannKDTreeShell.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    类注释
 *@  ClassName     :    ICP
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/14 14:04
 *@  Brief         :    输入两期点云，进行精细配准，输出变换矩阵
 *********************************************************************/
class ICP
{
  public:
	//构造函数
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

	//传入特征点云
	void setInputSource(Points &points);

	//传入目标特征点云
	void setInputTarget(Points &points);

	//建立目标点云的KdTree
	void setTargetTree();

	//对待搜索点云构建KdTree树搜索最邻近的K个点，包括点本身
	bool searchForNeighbors(Points &points, std::vector<int> &indices, wl::PointXYZ &point, int k,
							std::vector<int> &k_indices, std::vector<float> &k_sqrdistances);

	//计算变换距离方差
	double computeFitnessScore(Points &input_transfomed);

	//确定匹配点，存储匹配点的索引
	void getMatchPointsIndices(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2);

	//判断匹配点是否符合
	bool isMatched(wl::PointXYZ p0, wl::PointXYZ p1, wl::PointXYZ p2, wl::PointXYZ q0, wl::PointXYZ q1, wl::PointXYZ q2);

	//计算变换矩阵
	Eigen::Matrix4f computeTransformation(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2);

	//使用变换矩阵变换points
	void transformPoints(Points &input, Points &output, Eigen::Matrix4f &tf);

	//计算最终变换矩阵
	void align();

  public:
	//设置匹配点相似度阈值
	inline void
	setApproximateThreshold(double approximate_threshold) { approximate_threshold_ = approximate_threshold; }

	//设置连续两次变换点云与目标点云的距离方差之差
	inline void
	setEuclideanFitnessEpsilon(double epsilon) { euclidean_fitness_epsilon_ = epsilon; }

	//设置初始变换矩阵，默认为单位阵
	inline void
	setPreviousTransformation(Eigen::Matrix4f guess) { previous_transformation_ = guess; }

	inline void
	setMaxIterations(int max_iterations) { max_iterations_ = max_iterations; }

	//获取最终收敛后的距离方差
	inline double
	getFitnessScore() { return fitness_score_; }

	//获取最终的收敛状态
	inline bool
	hasconverged() { return converged_; }

	//获取收敛成功时迭代次数
	inline int
	getIterations() { return iterations_; }

	//获取最终的变换矩阵
	inline Eigen::Matrix4f
	getFinalTransformation() { return final_transformation_; }

  private:
	//杆塔点云特征点
	Points source_;

	//杆塔点云特征点
	Points target_;

	//杆塔点云特征点索引
	std::vector<int> source_indices_;

	//杆塔点云特征点索引
	std::vector<int> target_indices_;

	//以target_构建KdTree
	FlannKDTree<wl::PointXYZ> *tree_;

	//最大迭代次数
	int max_iterations_;

	//匹配点近似度阈值
	double approximate_threshold_;

	//距离方差之差
	double euclidean_fitness_epsilon_;

	//迭代结束时距离方差
	double fitness_score_;

	//判断收敛
	bool converged_;

	//迭代次数
	int iterations_;

	//初始变换矩阵
	Eigen::Matrix4f previous_transformation_;

	//迭代中间变换矩阵
	Eigen::Matrix4f transformation_;

	//最终变换矩阵
	Eigen::Matrix4f final_transformation_;
};

#endif