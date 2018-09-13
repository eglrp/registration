#include "RansacLine.h"

#define SampleSize 2

/*****************************************************************************
* @brief   : 传入待拟合点云，获取点的索引
* @author  : Liuyang
* @date    : 2018/8/9 21:16
* @version : ver 1.0
* @inparam : points 杆塔点云，indices 杆塔点云索引，从0开始
* @outparam:  
* @修订说明: 
*****************************************************************************/
void RansacLine::setInputCloud(Points &points, std::vector<int> &indices)
{
	if (points.empty())
	{
		return;
	}

	points_ = points;
	indices_ = indices;
	random_indices_ = indices_;

	//计算找到一条直线需要的迭代次数M
	double log_choose_probability = log(1 - choose_probability_);
	double log_outliers_probability = log(1 - pow((1 - outliers_probability_), 2));
	max_iterations_ = static_cast<int>(log_choose_probability / log_outliers_probability);
}
/*****************************************************************************
* @brief   : 寻找一个合格的模型，即直线的两点的索引
* @author  : Liuyang
* @date    : 2018/8/9 21:17
* @version : ver 1.0
* @inparam : 
* @outparam: sample 一个合格的模型
* @修订说明: 
*****************************************************************************/
void RansacLine::getSample(std::vector<int> &sample)
{
	//判断，如果点云数量少于模型需要的点数，返回
	if (indices_.size() < SampleSize)
	{
		sample.clear();
		return;
	}

	//确定搜索的次数，找到一条直线
	sample.resize(SampleSize);
	for (int i = 0; i < max_sample_checks_; ++i)
	{
		//从随机点云索引中获取两个索引值
		getSampleIndex(sample);

		//一旦有合格的模型，返回
		if (isSample(sample))
		{
			return;
		}
	}

	//如果没有模型
	sample.clear();
}
/*****************************************************************************
* @brief   : 寻找一个随机的模型
* @author  : Liuyang
* @date    : 2018/8/9 21:18
* @version : ver 1.0
* @inparam : 
* @outparam: sample 一个随机的模型
* @修订说明: 
*****************************************************************************/
void RansacLine::getSampleIndex(std::vector<int> &sample)
{
	size_t sample_size = sample.size();
	size_t random_indices_size = random_indices_.size();

	for (size_t i = 0; i < sample_size; ++i)
	{
		std::swap(random_indices_[i], random_indices_[i + (rand() % (random_indices_size - i))]);
	}
	std::copy(random_indices_.begin(), random_indices_.begin() + sample_size, sample.begin());
}
/*****************************************************************************
* @brief   : 判断两点不为同一点
* @author  : Liuyang
* @date    : 2018/8/9 21:20
* @version : ver 1.0
* @inparam : sample 寻找到的一个随机的模型 
* @outparam: 如果模型合格，返回true 
* @修订说明: 
*****************************************************************************/
bool RansacLine::isSample(std::vector<int> &sample)
{
	if (!inliers_.empty())
	{
		for (size_t i = 0; i < inliers_.size(); ++i)
		{
			if (sample[0] == inliers_[i] || sample[1] == inliers_[i])
			{
				return false;
			}
			else
			{
				continue;
			}
		}
	}

	if (points_[sample[0]].x != points_[sample[1]].x &&
		points_[sample[0]].y != points_[sample[1]].y &&
		points_[sample[0]].z != points_[sample[1]].z)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*****************************************************************************
* @brief   : 计算模型参数
* @author  : Liuyang
* @date    : 2018/8/9 21:21
* @version : ver 1.0
* @inparam : sample 一个合格的模型 
* @outparam: model_parameters 模型的参数，并返回true
* @修订说明: 
*****************************************************************************/
bool RansacLine::hasModelParameters(std::vector<int> sample, Eigen::VectorXf &model_parameters)
{
	if (sample.size() != 2)
	{
		return false;
	}

	model_parameters.resize(6);

	model_parameters[0] = points_[sample[0]].x;
	model_parameters[1] = points_[sample[0]].y;
	model_parameters[2] = points_[sample[0]].z;

	model_parameters[3] = points_[sample[1]].x - model_parameters[0];
	model_parameters[4] = points_[sample[1]].y - model_parameters[1];
	model_parameters[5] = points_[sample[1]].z - model_parameters[2];

	model_parameters.template tail<3>().normalize(); //后三个元素归一化

	return true;
}
/*****************************************************************************
* @brief   : 计算模型下在距离阈值内的邻域点号和点数
* @author  : Liuyang
* @date    : 2018/8/9 21:22
* @version : ver 1.0
* @inparam : model_parameters 模型，threshold 距离阈值
* @outparam: inliers 该模型下的局内点的索引，count 局内点数
* @修订说明: 
*****************************************************************************/
void RansacLine::getPointsOfRange(Eigen::VectorXf &model_parameters, double threshold, std::vector<int> &inliers, int &count)
{
	//判断模型是否合理
	if (!isModel(model_parameters))
	{
		return;
	}

	inliers.clear();
	count = 0; //记录局内点数

	double sqr_threshold = threshold * threshold; //利用距离的平方判断

	Eigen::Vector4f line_pt(model_parameters[0], model_parameters[1], model_parameters[2], 0); //直线上一点
	Eigen::Vector4f line_dr(model_parameters[3], model_parameters[4], model_parameters[5], 0); //直线方向的向量，已经归一化

	//遍历所有点
	for (size_t i = 0; i < indices_.size(); ++i)
	{
		Eigen::Vector4f point(points_[indices_[i]].x, points_[indices_[i]].y, points_[indices_[i]].z, 0);

		//利用向量的叉积计算空间点到直线的距离
		double sqr_distance = (line_pt - point).cross3(line_dr).squaredNorm();

		if (sqr_distance < sqr_threshold)
		{
			inliers.emplace_back(indices_[i]);
			++count;
		}
	}

	inliers.resize(count);
}
/*****************************************************************************
* @brief   : 判断模型是否合格
* @author  : Liuyang
* @date    : 2018/8/9 21:24
* @version : ver 1.0
* @inparam : model_parameters 模型参数
* @outparam: 模型合格，返回true
* @修订说明: 
*****************************************************************************/
bool RansacLine::isModel(Eigen::VectorXf &model_parameters)
{
	if (model_parameters.size() == 6)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*****************************************************************************
* @brief   : 计算出一个模型，并获得该模型所有的局内点
* @author  : Liuyang
* @date    : 2018/8/9 21:24
* @version : ver 1.0
* @inparam : 
* @outparam: 
* @修订说明: 
*****************************************************************************/
void RansacLine::computeOneModel()
{
	int iterations = 0; //记录迭代次数

	std::vector<int> best_inliers; //记录局内点数最多的模型
	int best_inliers_count = -INT_MAX;

	std::vector<int> inliers; //用于存储计算出来的局内点号
	int inliers_count;

	std::vector<int> sample; //选择的模型
	Eigen::VectorXf model_parameters;

	while (iterations < max_iterations_) //设定好的迭代次数，搜索局内点最多的模型
	{
		//获取一个模型
		getSample(sample);

		if (sample.empty())
		{
			break;
		}

		//计算模型参数
		if (!hasModelParameters(sample, model_parameters))
		{
			continue; //执行下一个循环
		}

		//传入模型参数、距离阈值，获得局内点号和点数
		getPointsOfRange(model_parameters, threshold_, inliers, inliers_count);

		//依据局内点数判断出最好的模型
		if (inliers_count > best_inliers_count)
		{
			//获取局内点最多时的点号和数量
			best_inliers = inliers;
			best_inliers_count = inliers_count;
		}

		iterations++;

		//迭代结束条件
		if (iterations > max_iterations_)
		{
			break;
		}
	}

	//保存所有局内点
	for (int i = 0; i < best_inliers_count; ++i)
	{
		inliers_.emplace_back(best_inliers[i]);
	}

	//释放内存
	std::vector<int>().swap(best_inliers);
	std::vector<int>().swap(inliers);
}
/*****************************************************************************
* @brief   : 计算所有模型，获得所有的局内点的索引（不重复）
* @author  : Liuyang
* @date    : 2018/8/9 21:26
* @version : ver 1.0
* @inparam : 
* @outparam:
* @修订说明: 
*****************************************************************************/
void RansacLine::computeAllModel()
{
	for (int i = 0; i < iterations_; ++i)
	{
		//计算一个模型，获取局内点
		computeOneModel();
	}

	//去除重复值
	std::sort(inliers_.begin(), inliers_.end());

	inliers_.erase(std::unique(inliers_.begin(), inliers_.end()), inliers_.end());
}