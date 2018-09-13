/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018��PCM�Ŷ� 
 *                      All rights reserved.
 * @Info   ��    DATAGENERATOR.H �ļ�ע��
 * @�ļ���  :    DATAGENERATOR.H
 * @�ļ�·��:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @����    :    Liuyang
 * @����ʱ��:    2018/8/15 10:47
 * @�ļ�����:    ����Ransac�ĸ�����������ȡ��ICP�㷨�ĵ�����׼
 *********************************************************************/

#ifndef DATAGENERATOR_H
#define DATAGENERATOR_H

#include <vector>

#include <QtCore/QString>
#include <Eigen/Eigen>

#include "point_types.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    ��ע��
 *@  ClassName     :    DataGenerator
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/15 10:49
 *@  Brief         :    ����RansacLine���IterativeClosestPoint����е�����׼
 *********************************************************************/
class DataGenerator
{
  public:
    //ICP��׼������
    bool DoRegistrate();

    //��ȡ���ƣ���ȡ���������꣬��Ҫ�߶���ֵ��
    void readLasFile(QString &file_name, Points &cloud, Points &points, std::vector<int> &indices);

    //����RansacLine����ȡ���������ߺ�ѡ��
    void runRansacLine(Points &points, std::vector<int> &indices, std::vector<int> &inliers);

    //����ICP����ȡ��ϸ��׼�ı任����
    void runICP(Points &points1, Points &points2);

    //д���任ԭʼ���Ƶ��ļ���
    void writeLasFile(QString &file_name, Eigen::Matrix4f &tf);

  public:
    //����ԭʼ����·��
    inline void
    setFileName(QString file_name1, QString file_name2)
    {
        file_name1_ = file_name1;
        file_name2_ = file_name2;
    }

    //ʹ��RansacLine����Ҫ����Ĳ�����һ��Ҫ����
    inline void
    setRansacLineParameters(double threshold, int iterations)
    {
        threshold_ransacline_ = threshold;
        iterations_ransacline_ = iterations;
    }

    //ʹ��ICP����Ҫ����Ĳ�����һ��Ҫ����
    inline void
    setIcpParameters(int max_iterations, double approximate_threshold, double euclidean_fitness_epsilon)
    {
        max_iterations_ = max_iterations;
        approximate_threshold_ = approximate_threshold;
        euclidean_fitness_epsilon_ = pow(10, -euclidean_fitness_epsilon);
    }

    //����д���ļ�·��
    inline void
    setFileNameOut(QString file_name_out) { file_name_out_ = file_name_out; }

    //��ȡ���ձ任����
    inline Eigen::Matrix4f
    getFinalTransformation() { return final_transformation_; }

  private:
    //ǰ�����ڵ������ݵ�·��
    QString file_name1_;
    QString file_name2_;

    //ǰ�������е���
    Points cloud1_;
    Points cloud2_;

    //ǰ�����ڸ������Ƽ�������
    Points points1_;
    Points points2_;

    std::vector<int> indices1_;
    std::vector<int> indices2_;

    //ʹ��RansacLine����Ҫ����Ĳ���
    double threshold_ransacline_;
    int iterations_ransacline_;

    //ʹ��ICP����Ҫ����Ĳ���
    int max_iterations_;
    double approximate_threshold_;
    double euclidean_fitness_epsilon_;

    //����׼��ȡ�ı任������ΪICP��׼�ĳ�ֵ
    Eigen::Matrix4f initial_transformation_;

    //���ձ任����
    Eigen::Matrix4f final_transformation_;

    //д���ļ�·��
    QString file_name_out_;
};

#endif