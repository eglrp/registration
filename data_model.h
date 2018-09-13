#ifndef WL_DATA_MODEL_H_
#define WL_DATA_MODEL_H_

#include <string>

#include "point_cloud.h"

namespace wl
{

class LidarDataModel
{

  public:
	//获取高植被点云
	PointCloud *getHighVeg()
	{
		return getXLayer(3);
	}

	//获取导线点云
	PointCloud *getPowerLine()
	{
		return getXLayer(5);
	}

	std::string _las_file; //当前打开的金字塔文件名称

	/*-------------------------------------------input------------------------------------*/
	std::vector<PointCloud *> _vec_clouds;
	std::vector<std::string> _img_layers_name;

	/*-------------------------------------------output------------------------------------*/

	std::vector<PointCloud *> _new_clouds;
	std::vector<std::string> _new_img_files;   //contains the output images' name
	std::vector<std::string> _new_other_files; //other file names.

	vector<Tower> _towers;
	vector<BadPoint> _cross_points;
	vector<BadPoint> _dangerous_points;
	//std::size_type _current_layer;
	//std::vector< int > _roi_indices;
	//std::vector< ind > _new_indices;

  private:
	PointCloud *getXLayer(int index)
	{
		PointCloud *pcl = NULL;

		//if (index == 5)
		//{
		//	pcl = new PointCloud;

		//	for (int i=_vec_clouds.size()-1; i>=0; --i)
		//	{
		//		if (_vec_clouds[i] && !_vec_clouds[i]->empty())
		//		{
		//			for (int j=0; j<_vec_clouds[i]->size(); j++)
		//			{
		//				if (_vec_clouds[i]->at(j).classification == index)
		//				{
		//					pcl->push_back(_vec_clouds[i]->at(j));
		//				}
		//			}
		//		}
		//	}
		//}

		//else
		{
			for (int i = _vec_clouds.size() - 1; i >= 0; --i)
			{
				if (_vec_clouds[i] && !_vec_clouds[i]->empty() && index == _vec_clouds[i]->at(0).classification)
				{
					pcl = _vec_clouds[i];
					break;
				}
			}
		}

		return pcl;
	}
};
} // namespace wl

#endif
