#pragma once
#include <iostream>
#include <string>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h> 

#include <OpenMesh/Core/Geometry/VectorT.hh>
//#include "QGLViewerWidget.h"
#include "MeshDefinition.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/bilateral.h> // Ë«±ßÂË²¨
//#include <pcl/filters/impl/bilateral.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>


inline double kernel(double x, double sigma)
{
	return (std::exp(-(x * x) / (2 * sigma * sigma)));
}

pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud);
int bilateralFilter(Mesh mesh);