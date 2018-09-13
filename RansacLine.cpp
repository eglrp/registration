#include "RansacLine.h"

#define SampleSize 2

/*****************************************************************************
* @brief   : �������ϵ��ƣ���ȡ�������
* @author  : Liuyang
* @date    : 2018/8/9 21:16
* @version : ver 1.0
* @inparam : points �������ƣ�indices ����������������0��ʼ
* @outparam:  
* @�޶�˵��: 
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

	//�����ҵ�һ��ֱ����Ҫ�ĵ�������M
	double log_choose_probability = log(1 - choose_probability_);
	double log_outliers_probability = log(1 - pow((1 - outliers_probability_), 2));
	max_iterations_ = static_cast<int>(log_choose_probability / log_outliers_probability);
}
/*****************************************************************************
* @brief   : Ѱ��һ���ϸ��ģ�ͣ���ֱ�ߵ����������
* @author  : Liuyang
* @date    : 2018/8/9 21:17
* @version : ver 1.0
* @inparam : 
* @outparam: sample һ���ϸ��ģ��
* @�޶�˵��: 
*****************************************************************************/
void RansacLine::getSample(std::vector<int> &sample)
{
	//�жϣ����������������ģ����Ҫ�ĵ���������
	if (indices_.size() < SampleSize)
	{
		sample.clear();
		return;
	}

	//ȷ�������Ĵ������ҵ�һ��ֱ��
	sample.resize(SampleSize);
	for (int i = 0; i < max_sample_checks_; ++i)
	{
		//��������������л�ȡ��������ֵ
		getSampleIndex(sample);

		//һ���кϸ��ģ�ͣ�����
		if (isSample(sample))
		{
			return;
		}
	}

	//���û��ģ��
	sample.clear();
}
/*****************************************************************************
* @brief   : Ѱ��һ�������ģ��
* @author  : Liuyang
* @date    : 2018/8/9 21:18
* @version : ver 1.0
* @inparam : 
* @outparam: sample һ�������ģ��
* @�޶�˵��: 
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
* @brief   : �ж����㲻Ϊͬһ��
* @author  : Liuyang
* @date    : 2018/8/9 21:20
* @version : ver 1.0
* @inparam : sample Ѱ�ҵ���һ�������ģ�� 
* @outparam: ���ģ�ͺϸ񣬷���true 
* @�޶�˵��: 
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
* @brief   : ����ģ�Ͳ���
* @author  : Liuyang
* @date    : 2018/8/9 21:21
* @version : ver 1.0
* @inparam : sample һ���ϸ��ģ�� 
* @outparam: model_parameters ģ�͵Ĳ�����������true
* @�޶�˵��: 
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

	model_parameters.template tail<3>().normalize(); //������Ԫ�ع�һ��

	return true;
}
/*****************************************************************************
* @brief   : ����ģ�����ھ�����ֵ�ڵ������ź͵���
* @author  : Liuyang
* @date    : 2018/8/9 21:22
* @version : ver 1.0
* @inparam : model_parameters ģ�ͣ�threshold ������ֵ
* @outparam: inliers ��ģ���µľ��ڵ��������count ���ڵ���
* @�޶�˵��: 
*****************************************************************************/
void RansacLine::getPointsOfRange(Eigen::VectorXf &model_parameters, double threshold, std::vector<int> &inliers, int &count)
{
	//�ж�ģ���Ƿ����
	if (!isModel(model_parameters))
	{
		return;
	}

	inliers.clear();
	count = 0; //��¼���ڵ���

	double sqr_threshold = threshold * threshold; //���þ����ƽ���ж�

	Eigen::Vector4f line_pt(model_parameters[0], model_parameters[1], model_parameters[2], 0); //ֱ����һ��
	Eigen::Vector4f line_dr(model_parameters[3], model_parameters[4], model_parameters[5], 0); //ֱ�߷�����������Ѿ���һ��

	//�������е�
	for (size_t i = 0; i < indices_.size(); ++i)
	{
		Eigen::Vector4f point(points_[indices_[i]].x, points_[indices_[i]].y, points_[indices_[i]].z, 0);

		//���������Ĳ������ռ�㵽ֱ�ߵľ���
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
* @brief   : �ж�ģ���Ƿ�ϸ�
* @author  : Liuyang
* @date    : 2018/8/9 21:24
* @version : ver 1.0
* @inparam : model_parameters ģ�Ͳ���
* @outparam: ģ�ͺϸ񣬷���true
* @�޶�˵��: 
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
* @brief   : �����һ��ģ�ͣ�����ø�ģ�����еľ��ڵ�
* @author  : Liuyang
* @date    : 2018/8/9 21:24
* @version : ver 1.0
* @inparam : 
* @outparam: 
* @�޶�˵��: 
*****************************************************************************/
void RansacLine::computeOneModel()
{
	int iterations = 0; //��¼��������

	std::vector<int> best_inliers; //��¼���ڵ�������ģ��
	int best_inliers_count = -INT_MAX;

	std::vector<int> inliers; //���ڴ洢��������ľ��ڵ��
	int inliers_count;

	std::vector<int> sample; //ѡ���ģ��
	Eigen::VectorXf model_parameters;

	while (iterations < max_iterations_) //�趨�õĵ����������������ڵ�����ģ��
	{
		//��ȡһ��ģ��
		getSample(sample);

		if (sample.empty())
		{
			break;
		}

		//����ģ�Ͳ���
		if (!hasModelParameters(sample, model_parameters))
		{
			continue; //ִ����һ��ѭ��
		}

		//����ģ�Ͳ�����������ֵ����þ��ڵ�ź͵���
		getPointsOfRange(model_parameters, threshold_, inliers, inliers_count);

		//���ݾ��ڵ����жϳ���õ�ģ��
		if (inliers_count > best_inliers_count)
		{
			//��ȡ���ڵ����ʱ�ĵ�ź�����
			best_inliers = inliers;
			best_inliers_count = inliers_count;
		}

		iterations++;

		//������������
		if (iterations > max_iterations_)
		{
			break;
		}
	}

	//�������о��ڵ�
	for (int i = 0; i < best_inliers_count; ++i)
	{
		inliers_.emplace_back(best_inliers[i]);
	}

	//�ͷ��ڴ�
	std::vector<int>().swap(best_inliers);
	std::vector<int>().swap(inliers);
}
/*****************************************************************************
* @brief   : ��������ģ�ͣ�������еľ��ڵ�����������ظ���
* @author  : Liuyang
* @date    : 2018/8/9 21:26
* @version : ver 1.0
* @inparam : 
* @outparam:
* @�޶�˵��: 
*****************************************************************************/
void RansacLine::computeAllModel()
{
	for (int i = 0; i < iterations_; ++i)
	{
		//����һ��ģ�ͣ���ȡ���ڵ�
		computeOneModel();
	}

	//ȥ���ظ�ֵ
	std::sort(inliers_.begin(), inliers_.end());

	inliers_.erase(std::unique(inliers_.begin(), inliers_.end()), inliers_.end());
}