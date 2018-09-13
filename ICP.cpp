#include <algorithm>

#include "ICP.h"

/*****************************************************************************
* @brief   : 传入待配准数据
* @author  : Liuyang
* @date    : 2018/8/17 19:07
* @version : ver 1.0
* @inparam : points 待配准数据 
* @outparam: 
* @修订说明: 
*****************************************************************************/
void ICP::setInputSource(Points &points)
{
	source_ = points;

	source_indices_.resize(source_.size());
	for (size_t i = 0; i < source_indices_.size(); ++i)
	{
		source_indices_[i] = static_cast<int>(i);
	}
}
/*****************************************************************************
* @brief   : 传入目标数据
* @author  : Liuyang
* @date    : 2018/8/17 19:07
* @version : ver 1.0
* @inparam : points 目标数据
* @outparam: 
* @修订说明: 
*****************************************************************************/
void ICP::setInputTarget(Points &points)
{
	target_ = points;

	target_indices_.resize(target_.size());
	for (size_t i = 0; i < target_indices_.size(); ++i)
	{
		target_indices_[i] = static_cast<int>(i);
	}
}
/*****************************************************************************
* @brief   : 建立目标点云的KdTree
* @author  : Liuyang
* @date    : 2018/8/17 19:07
* @version : ver 1.0
* @inparam : 
* @outparam: 
* @修订说明: 
*****************************************************************************/
void ICP::setTargetTree()
{
	tree_->setSortedResults(true);
	tree_->BuildKDTree(target_, target_indices_);
}

/*****************************************************************************
* @brief   : 任意点云构建KdTree
* @author  : Liuyang
* @date    : 2018/8/15 10:17
* @version : ver 1.0
* @inparam : points 点云数据，indices 点云索引，point 某点，k 需要邻域点的个数
* @outparam: k_indices 邻域点的索引，k_sqrdistances，邻域点与某点的距离平方
* @修订说明: 
*****************************************************************************/
bool ICP::searchForNeighbors(Points &points, std::vector<int> &indices, wl::PointXYZ &point, int k,
							 std::vector<int> &k_indices, std::vector<float> &k_sqrdistances)
{
	Points points_tmp = points;

	FlannKDTree<wl::PointXYZ> tree;

	tree.SetDimMark(true, true, true);
	tree.setSortedResults(true);
	tree.BuildKDTree(points_tmp, indices);

	if (tree.nearestKSearch(point, k, k_indices, k_sqrdistances) > 0)
	{
		Points().swap(points_tmp);
		return true;
	}
	else
	{
		return false;
	}
}
/*****************************************************************************
* @brief   : 计算变换后，与target_距离平方和的均值
* @author  : Liuyang
* @date    : 2018/8/15 10:21
* @version : ver 1.0
* @inparam : source_transformed 变换后的数据 
* @outparam: fitness_score 距离平方和均值 
* @修订说明: 
*****************************************************************************/
double ICP::computeFitnessScore(Points &input_transfomed)
{
	double fitness_score = 0.0;

	int k = 1;
	std::vector<int> k_indices(1);
	std::vector<float> k_sqrdistances(1);

	int n = 0;
	for (int i = 0; i < input_transfomed.size(); ++i)
	{
		//已经建树了，逐点搜索最邻近点
		if (tree_->nearestKSearch(input_transfomed[i], k, k_indices, k_sqrdistances) > 0)
		{
			if (k_sqrdistances[0] <= std::numeric_limits<float>::max()) //可以考虑设置一个阈值
			{
				fitness_score += k_sqrdistances[0];
				n++;
			}
		}
	}

	//距离平方和除n
	if (n > 0)
	{
		return (fitness_score / n);
	}
	else
	{
		return std::numeric_limits<double>::max();
	}
}
/*****************************************************************************
* @brief   : 使用变换矩阵变换points，注意点为三维，变换矩阵为4*4
* @author  : Liuyang
* @date    : 2018/8/15 10:23
* @version : ver 1.0
* @inparam : input 变换前点云，tf 变换矩阵 
* @outparam: output 变换后点云
* @修订说明: 
*****************************************************************************/
void ICP::transformPoints(Points &input, Points &output, Eigen::Matrix4f &tf)

{
	output.clear();
	output.resize(input.size());

	//矩阵相乘，4*4和4*size(),可以以考虑使用Eigen
	for (int i = 0; i < input.size(); ++i)
	{
		float xx = input[i].x;
		float yy = input[i].y;
		float zz = input[i].z;

		output[i].x = static_cast<float>(tf(0, 0) * xx + tf(0, 1) * yy + tf(0, 2) * zz + tf(0, 3));
		output[i].y = static_cast<float>(tf(1, 0) * xx + tf(1, 1) * yy + tf(1, 2) * zz + tf(1, 3));
		output[i].z = static_cast<float>(tf(2, 0) * xx + tf(2, 1) * yy + tf(2, 2) * zz + tf(2, 3));
	}
}
/*****************************************************************************
* @brief   : 确定匹配点，存储匹配点的索引
* @author  : Liuyang
* @date    : 2018/8/15 10:24
* @version : ver 1.0
* @inparam : input_transformed 变换后点云
* @outparam: mp_indices1 变换后点云中匹配点的索引，mp_indices2 target_点云中匹配点的索引
* @修订说明: 
*****************************************************************************/
void ICP::getMatchPointsIndices(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2)
{
	if (input_transformed.empty()) //和source_一样的大小，索引可以通用
	{
		return;
	}

	//在tree_中搜索每个待匹配点的最邻近目标点
	std::vector<int> k_indices(1);
	std::vector<float> k_sqrdistances(1);

	//在tree_中搜索目标点的2个邻近点
	std::vector<int> indices_tgt(3);
	std::vector<float> sqrdistances_tgt(3);

	//在tree中搜索待匹配点的2个邻近点
	std::vector<int> indices_src(3);
	std::vector<float> sqrdistances_src(3);

	mp_indices1.resize(std::max(source_.size(), target_.size()));
	mp_indices2.resize(std::max(source_.size(), target_.size()));

	int mp_count = 0;

	//待匹配所有点，去除错误匹配点
	for (int i = 0; i < input_transformed.size(); ++i)
	{
		//在tree_中搜索每个待匹配点的最邻近目标点，第一个点不为本身
		if (tree_->nearestKSearch(input_transformed[i], 1, k_indices, k_sqrdistances) > 0)
		{
			//在tree_中搜索目标点的2个邻近点，第一个点为本身
			if (tree_->nearestKSearch(target_[k_indices[0]], 3, indices_tgt, sqrdistances_tgt) > 0)
			{
				//在tree中搜索待匹配点的2个邻近点，第一个点为本身
				if (searchForNeighbors(input_transformed, source_indices_, input_transformed[i], 3, indices_src, sqrdistances_src))
				{
					//匹配点的度量，除去不合适的匹配点，p01，p02，p1，p2，q01，q02，q1，q2
					if (isMatched(input_transformed[indices_src[0]], input_transformed[indices_src[1]], input_transformed[indices_src[2]],
						target_[indices_tgt[0]], target_[indices_tgt[1]], target_[indices_tgt[2]]))
					{
						mp_indices1[mp_count] = i;
						mp_indices2[mp_count] = k_indices[0];

						++mp_count;
					}
				}
			}
		}
	}

	mp_indices1.resize(mp_count);
	mp_indices2.resize(mp_count);

	//释放内存
	std::vector<int>().swap(k_indices);
	std::vector<float>().swap(k_sqrdistances);

	std::vector<int>().swap(indices_tgt);
	std::vector<float>().swap(sqrdistances_tgt);

	std::vector<int>().swap(indices_src);
	std::vector<float>().swap(sqrdistances_src);
}
/*****************************************************************************
* @brief   : 判断匹配点是否符合
* @author  : Liuyang
* @date    : 2018/8/15 10:27
* @version : ver 1.0
* @inparam : p01、p02、q01、q02 匹配点与其邻域点距离平方，p1、p2、q1、q2 邻域点
* @outparam: 
* @修订说明: 
*****************************************************************************/
bool ICP::isMatched(wl::PointXYZ p0, wl::PointXYZ p1, wl::PointXYZ p2, wl::PointXYZ q0, wl::PointXYZ q1, wl::PointXYZ q2)
{
	float p01 = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
	float q01 = sqrt(pow(q0.x - q1.x, 2) + pow(q0.y - q1.y, 2) + pow(q0.z - q1.z, 2));

	float p02 = sqrt(pow(p0.x - p2.x, 2) + pow(p0.y - p2.y, 2) + pow(p0.z - p2.z, 2));
	float q02 = sqrt(pow(q0.x - q2.x, 2) + pow(q0.y - q2.y, 2) + pow(q0.z - q2.z, 2));

	float p12 = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
	float q12 = sqrt(pow(q1.x - q2.x, 2) + pow(q1.y - q2.y, 2) + pow(q1.z - q2.z, 2));

	float max = std::max(std::max(std::max(p01, p02), p12), std::max(std::max(q01, q02), q12));

	if (std::abs(p01 - q01) / max < approximate_threshold_ &&
		std::abs(p02 - q02) / max < approximate_threshold_ &&
		std::abs(p12 - q12) / max < approximate_threshold_)
	{
		return true;
	}
	else
	{
		return false;
	}
}
/*****************************************************************************
* @brief   : 利用匹配点对计算变换矩阵
* @author  : Liuyang
* @date    : 2018/8/15 10:29
* @version : ver 1.0
* @inparam : input_transformed 变换后点云 mp_indices1 变换后点云中匹配点的索引，mp_indices2 target_点云中匹配点的索引
* @outparam: 
* @修订说明: 
*****************************************************************************/
Eigen::Matrix4f ICP::computeTransformation(Points &input_transformed, std::vector<int> &mp_indices1, std::vector<int> &mp_indices2)
{
	if (mp_indices1.size() != mp_indices2.size())
	{
		return Eigen::Matrix4f::Identity();
	}

	const int npts = static_cast<int>(mp_indices1.size());

	if (npts < 3)
	{
		return Eigen::Matrix4f::Identity();
	}

	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src(3, npts); //Dynamic表示列数未知
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_dst(3, npts);

	for (int i = 0; i < npts; ++i)
	{
		cloud_src(0, i) = input_transformed[mp_indices1[i]].x;
		cloud_src(1, i) = input_transformed[mp_indices1[i]].y;
		cloud_src(2, i) = input_transformed[mp_indices1[i]].z;

		cloud_dst(0, i) = target_[mp_indices2[i]].x;
		cloud_dst(1, i) = target_[mp_indices2[i]].y;
		cloud_dst(2, i) = target_[mp_indices2[i]].z;
	}

	//基于SVD分解计算变换矩阵
	//Eigen::Matrix4f transformation = Eigen::umeyama(cloud_src, cloud_dst, false);

	Eigen::Vector3f mean_src = cloud_src.rowwise().mean();
	Eigen::Vector3f mean_dst = cloud_dst.rowwise().mean();

	Eigen::Matrix3f cov = ((cloud_src.colwise() - mean_src) * (cloud_dst.colwise() - mean_dst).transpose()) / (static_cast<float>(npts));

	Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullV | Eigen::ComputeFullU);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();

	Eigen::Matrix3f R = V * U.transpose();
	Eigen::Vector3f T = mean_dst - R * mean_src;

	Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());

	transformation.block<3, 3>(0, 0) = R;
	transformation.block<3, 1>(0, 3) = T;

	return transformation;
}
/*****************************************************************************
* @brief   : 根据变换矩阵初值后迭代计算
* @author  : Liuyang
* @date    : 2018/8/15 10:32
* @version : ver 1.0
* @inparam : 
* @outparam: 
* @修订说明: 
*****************************************************************************/
void ICP::align()
{
	Points input_transformed;
	Points output;

	double fitness_score = 0.0;

	int iterations = 1;
	converged_ = false;

	if (previous_transformation_ != Eigen::Matrix4f::Identity())
	{
		//变换得到新的点云
		transformPoints(source_, input_transformed, previous_transformation_);
	}
	else
	{
		input_transformed = source_;
	}

	//计算变换后点云和
	fitness_score = computeFitnessScore(input_transformed);

	Eigen::Matrix4f last_transformation = previous_transformation_;

	std::vector<int> mp_indices1;
	std::vector<int> mp_indices2;

	//迭代直到收敛
	while (!converged_)
	{
		//搜索匹配点并剔除不满足条件的匹配点，存储索引
		getMatchPointsIndices(input_transformed, mp_indices1, mp_indices2); //很耗时，如何优化

		if (mp_indices1.size() < 3)
		{
			break;
		}

		//计算新的变换矩阵
		transformation_ = computeTransformation(input_transformed, mp_indices1, mp_indices2); //很快

		//用新的变换矩阵转换点云
		transformPoints(input_transformed, output, transformation_); //很快

		input_transformed = output;

		//计算新的距离方差
		fitness_score_ = computeFitnessScore(input_transformed);

		//与前一次距离方差的差
		double fitness_difference = fitness_score_ - fitness_score;

		//如果差值小于阈值，即收敛
		if (std::abs(fitness_difference) < euclidean_fitness_epsilon_ && fitness_score_ < 0.1)
		{
			converged_ = true;
			break; //迭代结束
		}
		else
		{
			fitness_score = fitness_score_;
		}

		Eigen::Matrix3f R = transformation_.block<3, 3>(0, 0);
		Eigen::Vector3f T = transformation_.block<3, 1>(0, 3);
		final_transformation_.block<3, 3>(0, 0) = R * last_transformation.block<3, 3>(0, 0);
		final_transformation_.block<3, 1>(0, 3) = R * last_transformation.block<3, 1>(0, 3) + T;

		last_transformation = final_transformation_;

		//如果迭代次数多于阈值，迭代结束，状态为不收敛
		if (iterations >= max_iterations_)
		{
			break;
		}

		//迭代次数+1
		++iterations;
	}

	//获得迭代次数
	iterations_ = iterations;

	//释放内存
	Points().swap(input_transformed);
	Points().swap(output);

	std::vector<int>().swap(mp_indices1);
	std::vector<int>().swap(mp_indices2);

	if (tree_ != nullptr)
	{
		delete tree_;
		tree_ = nullptr;
	}
}