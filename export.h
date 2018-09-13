
#ifndef _POINTCLOUD_EXPORT_H
#define _POINTCLOUD_EXPORT_H

#ifdef POINTCLOUD_MAKE_DLL
#define POINTCLOUD_API __declspec(dllexport)
#else
#define POINTCLOUD_API __declspec(dllimport)
#endif

//#include <config.h>

#endif // _POINTCLOUD_EXPORT_H