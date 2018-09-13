#pragma execution_character_set("utf-8")

#include <QCoreApplication>

#include "las/lasdefinitions.h"
#include "las/lasreader.h"
#include "las/laswriter.h"

#include "PCA.h"
#include "RansacLine.h"
#include "ICP.h"
#include "DataGenerator.h"
#include "ProgressDialog.h"

/*****************************************************************************
* @brief   : 读取原始LAS点云文件，由于已分类过，提取其中的杆塔点
* @author  : Liuyang
* @date    : 2018/8/15 10:51
* @version : ver 1.0
* @inparam : filename 点云路径 
* @outparam: points PointXYZ格式的杆塔点云数据，indices 杆塔点云索引 
* @修订说明: 
*****************************************************************************/
void DataGenerator::readLasFile(QString &filename, Points &cloud, Points &points, std::vector<int> &indices)
{
	//读取lasfile，保存杆塔点
	FILE *fp = fopen(filename.toLocal8Bit().constData(), "rb");
	if (!fp)
	{
		return;
	}

	LASreader *reader = new LASreader();
	if (!reader->open(fp))
	{
		return;
	}

	//逐点读
	int id = 0;
	while (reader->read_point())
	{
		LASpoint laspoint = reader->point; //LASpoint格式读
		wl::PointXYZ pt;				   //PointXYZ格式存

		pt.x = laspoint.x * reader->header.x_scale_factor + reader->header.x_offset;
		pt.y = laspoint.y * reader->header.y_scale_factor + reader->header.y_offset;
		pt.z = laspoint.z * reader->header.z_scale_factor + reader->header.z_offset;

		cloud.emplace_back(pt);

		if (6 == laspoint.classification)
		{
			points.emplace_back(pt);
			indices.emplace_back(id);
			++id;
		}
	}

	//释放指针
	reader->close();
	delete reader;
	reader = nullptr;

	fclose(fp);
}
/*****************************************************************************
* @brief   : 调用RansacLine类提取杆塔特征线拟合点的索引
* @author  : Liuyang
* @date    : 2018/8/15 10:52
* @version : ver 1.0
* @inparam : points 杆塔点云数据，indices 杆塔点云索引
* @outparam: inliers 杆塔特征线拟合点的索引 
* @修订说明: 
*****************************************************************************/
void DataGenerator::runRansacLine(Points &points, std::vector<int> &indices, std::vector<int> &inliers)
{
	if (points.empty())
	{
		return;
	}

	RansacLine *ransacline = new RansacLine();

	//输入点云和参数
	ransacline->setInputCloud(points, indices);
	//从Registration类中获取的参数
	ransacline->setThreshold(threshold_ransacline_);
	ransacline->setIterations(iterations_ransacline_);

	//计算
	ransacline->computeAllModel();

	//获取所有局内点索引
	inliers = ransacline->getInliers();

	//释放内存
	if (ransacline != nullptr)
	{
		delete ransacline;
		ransacline = nullptr;
	}
}
/*****************************************************************************
* @brief   : 调用ICP类提取精细配准后的变换矩阵
* @author  : Liuyang
* @date    : 2018/8/15 10:54
* @version : ver 1.0
* @inparam : points1 前期杆塔特征线拟合点的点云，points2 后期杆塔特征线拟合点的点云
* @outparam: final_transformation 最终变换矩阵
* @修订说明: 
*****************************************************************************/
void DataGenerator::runICP(Points &points1, Points &points2)
{
	if (points1.empty() || points2.empty())
	{
		return;
	}

	//设置参数
	ICP *icp = new ICP();
	icp->setInputSource(points1);
	icp->setInputTarget(points2);
	icp->setTargetTree();
	//从Registration类中获取的参数
	icp->setPreviousTransformation(initial_transformation_);
	icp->setMaxIterations(max_iterations_);
	icp->setApproximateThreshold(approximate_threshold_);
	icp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

	//运行
	icp->align();

	//获取变换矩阵
	final_transformation_ = icp->getFinalTransformation();

	//释放内存
	if (icp != nullptr)
	{
		delete icp;
		icp = nullptr;
	}
}
/*****************************************************************************
* @brief   : 写出变换原始点云到文件中
* @author  : Liuyang
* @date    : 2018/8/15 14:41
* @version : ver 1.0
* @inparam : file_name 所有的原始点云，tf 最终变换矩阵
* @outparam:
* @修订说明:
*****************************************************************************/
void DataGenerator::writeLasFile(QString &file_name, Eigen::Matrix4f &tf)
{
	FILE *fp1 = fopen(file_name1_.toLocal8Bit().constData(), "rb");
	LASreader *reader = new LASreader();
	if (!reader->open(fp1))
	{
		return;
	}

	LASheader header = reader->header;

	FILE *fp2 = fopen(file_name.toLocal8Bit().constData(), "wb");
	LASwriter *writer = new LASwriter();
	if (!writer->open(fp2, &header))
	{
		return;
	}

	int count = header.number_of_point_records;
	for (int i = 0; i < count; ++i)
	{
		if (reader->read_point())
		{
			LASpoint pt = reader->point;

			double xx = pt.x * header.x_scale_factor + header.x_offset;
			double yy = pt.y * header.y_scale_factor + header.y_offset;
			double zz = pt.z * header.z_scale_factor + header.z_offset;

			xx = tf(0, 0) * xx + tf(0, 1) * yy + tf(0, 2) * zz + tf(0, 3);
			yy = tf(1, 0) * xx + tf(1, 1) * yy + tf(1, 2) * zz + tf(1, 3);
			zz = tf(2, 0) * xx + tf(2, 1) * yy + tf(2, 2) * zz + tf(2, 3);

			pt.x = static_cast<int>((xx - header.x_offset) / header.x_scale_factor);
			pt.y = static_cast<int>((yy - header.y_offset) / header.y_scale_factor);
			pt.z = static_cast<int>((zz - header.z_offset) / header.z_scale_factor);

			writer->write_point(&pt);
		}
	}

	//释放指针
	writer->close(true);
	reader->close();

	fclose(fp2);
	fclose(fp1);
}
/*****************************************************************************
* @brief   : ICP配准主函数
* @author  : Liuyang
* @date    : 2018/8/15 10:55
* @version : ver 1.0
* @inparam :  
* @outparam:  
* @修订说明: 
*****************************************************************************/
bool DataGenerator::DoRegistrate()
{
	ProgressDialog *plg = new ProgressDialog;

	if (plg)
	{
		plg->setMethodTitle("点云配准");
		plg->setInfo("正在读取文件...");
		plg->setCancelButton(0);
		plg->start();
		plg->show();
	}

	NormalizedProgress nprogress(plg, 4);

	//////////////////////////////////////////
	//读取两期点云数据文件，获取杆塔点和索引
	readLasFile(file_name1_, cloud1_, points1_, indices1_);
	readLasFile(file_name2_, cloud2_, points2_, indices2_);

	if (points1_.empty() || points2_.empty())
	{
		plg->stop();
		plg->close();
		return false;
	}

	nprogress.oneStep();

	//////////////////////////////////////////
	//调用RansacLine类提取杆塔特征线拟合点的索引
	plg->setInfo("正在提取特征点...");

	std::vector<int> inliers1;
	//std::vector<int> inliers2;
	runRansacLine(points1_, indices1_, inliers1);
	//runRansacLine(points2_, indices2_, inliers2);

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//调用PCA类特征匹配
	plg->setInfo("正在主轴变换...");

	PCA<wl::PointXYZ> pca;
	pca.runPCA(cloud1_, cloud2_); //全部点云
	initial_transformation_ = pca.getTransformation();

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//调用ICP类提取精细配准的变换矩阵
	plg->setInfo("正在配准...");

	Points points1_icp;
	points1_icp.resize(inliers1.size());
	for (int i = 0; i < inliers1.size(); ++i)
	{
		points1_icp[i] = points1_[inliers1[i]];
	}

	Points points2_icp(points2_);

	//获得变换矩阵
	runICP(points1_icp, points2_icp);

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//写出文件
	plg->setInfo("正在写出文件...");

	writeLasFile(file_name_out_, final_transformation_);

	plg->stop();

	//////////////////////////////////////////
	//释放内存
	std::vector<int>().swap(inliers1);
	//std::vector<int>().swap(inliers2);

	Points().swap(points1_icp);
	Points().swap(points2_icp);

	return true;
}