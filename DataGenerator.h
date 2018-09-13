/********************************************************************
 *                       COPY RIGHT NOTICE  
 *                  Copyright(c) 2018，PCM团队 
 *                      All rights reserved.
 * @Info   ：    DATAGENERATOR.H 文件注释
 * @文件名  :    DATAGENERATOR.H
 * @文件路径:    D:\360MOVEDATA\DESKTOP\REGISTRATION\REGISTRATION
 * @作者    :    Liuyang
 * @创建时间:    2018/8/15 10:47
 * @文件描述:    基于Ransac的杆塔特征线提取和ICP算法的点云配准
 *********************************************************************/

#ifndef DATAGENERATOR_H
#define DATAGENERATOR_H

#include <vector>

#include <QtCore/QString>
#include <Eigen/Eigen>

#include "point_types.h"

typedef std::vector<wl::PointXYZ> Points;

/********************************************************************
 *@  Info          :    类注释
 *@  ClassName     :    DataGenerator
 *@  BaseClassname :    
 *@  UsingNameSpace:    
 *@  Author        :    Liuyang
 *@  Date          :    2018/8/15 10:49
 *@  Brief         :    调用RansacLine类和IterativeClosestPoint类进行点云配准
 *********************************************************************/
class DataGenerator
{
  public:
    //ICP配准主函数
    bool DoRegistrate();

    //读取点云，提取出杆塔坐标，需要高度阈值吗？
    void readLasFile(QString &file_name, Points &cloud, Points &points, std::vector<int> &indices);

    //调用RansacLine类提取杆塔特征线候选点
    void runRansacLine(Points &points, std::vector<int> &indices, std::vector<int> &inliers);

    //调用ICP类提取精细配准的变换矩阵
    void runICP(Points &points1, Points &points2);

    //写出变换原始点云到文件中
    void writeLasFile(QString &file_name, Eigen::Matrix4f &tf);

  public:
    //传入原始点云路径
    inline void
    setFileName(QString file_name1, QString file_name2)
    {
        file_name1_ = file_name1;
        file_name2_ = file_name2;
    }

    //使用RansacLine类需要传入的参数，一定要设置
    inline void
    setRansacLineParameters(double threshold, int iterations)
    {
        threshold_ransacline_ = threshold;
        iterations_ransacline_ = iterations;
    }

    //使用ICP类需要传入的参数，一定要设置
    inline void
    setIcpParameters(int max_iterations, double approximate_threshold, double euclidean_fitness_epsilon)
    {
        max_iterations_ = max_iterations;
        approximate_threshold_ = approximate_threshold;
        euclidean_fitness_epsilon_ = pow(10, -euclidean_fitness_epsilon);
    }

    //设置写出文件路径
    inline void
    setFileNameOut(QString file_name_out) { file_name_out_ = file_name_out; }

    //获取最终变换矩阵
    inline Eigen::Matrix4f
    getFinalTransformation() { return final_transformation_; }

  private:
    //前、后期点云数据的路径
    QString file_name1_;
    QString file_name2_;

    //前后期所有点云
    Points cloud1_;
    Points cloud2_;

    //前、后期杆塔点云及其索引
    Points points1_;
    Points points2_;

    std::vector<int> indices1_;
    std::vector<int> indices2_;

    //使用RansacLine类需要传入的参数
    double threshold_ransacline_;
    int iterations_ransacline_;

    //使用ICP类需要传入的参数
    int max_iterations_;
    double approximate_threshold_;
    double euclidean_fitness_epsilon_;

    //粗配准获取的变换矩阵，作为ICP配准的初值
    Eigen::Matrix4f initial_transformation_;

    //最终变换矩阵
    Eigen::Matrix4f final_transformation_;

    //写出文件路径
    QString file_name_out_;
};

#endif