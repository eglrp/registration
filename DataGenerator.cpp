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
* @brief   : ��ȡԭʼLAS�����ļ��������ѷ��������ȡ���еĸ�����
* @author  : Liuyang
* @date    : 2018/8/15 10:51
* @version : ver 1.0
* @inparam : filename ����·�� 
* @outparam: points PointXYZ��ʽ�ĸ����������ݣ�indices ������������ 
* @�޶�˵��: 
*****************************************************************************/
void DataGenerator::readLasFile(QString &filename, Points &cloud, Points &points, std::vector<int> &indices)
{
	//��ȡlasfile�����������
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

	//����
	int id = 0;
	while (reader->read_point())
	{
		LASpoint laspoint = reader->point; //LASpoint��ʽ��
		wl::PointXYZ pt;				   //PointXYZ��ʽ��

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

	//�ͷ�ָ��
	reader->close();
	delete reader;
	reader = nullptr;

	fclose(fp);
}
/*****************************************************************************
* @brief   : ����RansacLine����ȡ������������ϵ������
* @author  : Liuyang
* @date    : 2018/8/15 10:52
* @version : ver 1.0
* @inparam : points �����������ݣ�indices ������������
* @outparam: inliers ������������ϵ������ 
* @�޶�˵��: 
*****************************************************************************/
void DataGenerator::runRansacLine(Points &points, std::vector<int> &indices, std::vector<int> &inliers)
{
	if (points.empty())
	{
		return;
	}

	RansacLine *ransacline = new RansacLine();

	//������ƺͲ���
	ransacline->setInputCloud(points, indices);
	//��Registration���л�ȡ�Ĳ���
	ransacline->setThreshold(threshold_ransacline_);
	ransacline->setIterations(iterations_ransacline_);

	//����
	ransacline->computeAllModel();

	//��ȡ���о��ڵ�����
	inliers = ransacline->getInliers();

	//�ͷ��ڴ�
	if (ransacline != nullptr)
	{
		delete ransacline;
		ransacline = nullptr;
	}
}
/*****************************************************************************
* @brief   : ����ICP����ȡ��ϸ��׼��ı任����
* @author  : Liuyang
* @date    : 2018/8/15 10:54
* @version : ver 1.0
* @inparam : points1 ǰ�ڸ�����������ϵ�ĵ��ƣ�points2 ���ڸ�����������ϵ�ĵ���
* @outparam: final_transformation ���ձ任����
* @�޶�˵��: 
*****************************************************************************/
void DataGenerator::runICP(Points &points1, Points &points2)
{
	if (points1.empty() || points2.empty())
	{
		return;
	}

	//���ò���
	ICP *icp = new ICP();
	icp->setInputSource(points1);
	icp->setInputTarget(points2);
	icp->setTargetTree();
	//��Registration���л�ȡ�Ĳ���
	icp->setPreviousTransformation(initial_transformation_);
	icp->setMaxIterations(max_iterations_);
	icp->setApproximateThreshold(approximate_threshold_);
	icp->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

	//����
	icp->align();

	//��ȡ�任����
	final_transformation_ = icp->getFinalTransformation();

	//�ͷ��ڴ�
	if (icp != nullptr)
	{
		delete icp;
		icp = nullptr;
	}
}
/*****************************************************************************
* @brief   : д���任ԭʼ���Ƶ��ļ���
* @author  : Liuyang
* @date    : 2018/8/15 14:41
* @version : ver 1.0
* @inparam : file_name ���е�ԭʼ���ƣ�tf ���ձ任����
* @outparam:
* @�޶�˵��:
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

	//�ͷ�ָ��
	writer->close(true);
	reader->close();

	fclose(fp2);
	fclose(fp1);
}
/*****************************************************************************
* @brief   : ICP��׼������
* @author  : Liuyang
* @date    : 2018/8/15 10:55
* @version : ver 1.0
* @inparam :  
* @outparam:  
* @�޶�˵��: 
*****************************************************************************/
bool DataGenerator::DoRegistrate()
{
	ProgressDialog *plg = new ProgressDialog;

	if (plg)
	{
		plg->setMethodTitle("������׼");
		plg->setInfo("���ڶ�ȡ�ļ�...");
		plg->setCancelButton(0);
		plg->start();
		plg->show();
	}

	NormalizedProgress nprogress(plg, 4);

	//////////////////////////////////////////
	//��ȡ���ڵ��������ļ�����ȡ�����������
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
	//����RansacLine����ȡ������������ϵ������
	plg->setInfo("������ȡ������...");

	std::vector<int> inliers1;
	//std::vector<int> inliers2;
	runRansacLine(points1_, indices1_, inliers1);
	//runRansacLine(points2_, indices2_, inliers2);

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//����PCA������ƥ��
	plg->setInfo("��������任...");

	PCA<wl::PointXYZ> pca;
	pca.runPCA(cloud1_, cloud2_); //ȫ������
	initial_transformation_ = pca.getTransformation();

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//����ICP����ȡ��ϸ��׼�ı任����
	plg->setInfo("������׼...");

	Points points1_icp;
	points1_icp.resize(inliers1.size());
	for (int i = 0; i < inliers1.size(); ++i)
	{
		points1_icp[i] = points1_[inliers1[i]];
	}

	Points points2_icp(points2_);

	//��ñ任����
	runICP(points1_icp, points2_icp);

	QCoreApplication::processEvents();
	nprogress.oneStep();

	//////////////////////////////////////////
	//д���ļ�
	plg->setInfo("����д���ļ�...");

	writeLasFile(file_name_out_, final_transformation_);

	plg->stop();

	//////////////////////////////////////////
	//�ͷ��ڴ�
	std::vector<int>().swap(inliers1);
	//std::vector<int>().swap(inliers2);

	Points().swap(points1_icp);
	Points().swap(points2_icp);

	return true;
}