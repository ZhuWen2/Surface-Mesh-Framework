#pragma once
#include <pcl/point_types.h>                  // 点类型定义相关头文件
#include <pcl/point_cloud.h>                  // 点类型定义相关头文件
#include <pcl/io/pcd_io.h>                    // PCD文件打开存储类相关头文件
#include <pcl/filters/voxel_grid.h>           // 体素滤波
#include <pcl/features/normal_3d.h>           // 法向量计算相关头文件
#include <pcl/registration/icp_nl.h>          // 非线性LM-ICP相关头文件
#include <pcl/registration/transforms.h>      // 变换矩阵类相关头文件
//#include <pcl/visualization/pcl_visualizer.h> // 可视化头文件
#include <boost/make_shared.hpp>              // boost指针相关头文件

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>//　随机采样一致性配准

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/time.h>//计算时间
#include <pcl/console/print.h>//PCL控制台输出

#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/gicp.h>  
#include <pcl/console/time.h>
#include <pcl/registration/icp.h> // icp算法

#include <pcl/registration/ia_kfpcs.h> //K4PCS算法头文件

using namespace std;
//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

// Types
typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<pcl::PointXYZ> PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimation<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


//简单类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

struct PCD
{
	PointCloud::Ptr cloud;           // 点云共享指针
	std::string f_name;              // 文件名称
	PCD() : cloud(new PointCloud) {};
};

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud);

int orin_align(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &OrinTransform);
int orin_align1(const pcl::PointCloud<PointNT>::Ptr object, const pcl::PointCloud<PointNT>::Ptr scene, Eigen::Matrix4f &OrinTransform);
int orin_align2(const pointcloud::Ptr source_cloud, const pointcloud::Ptr target_cloud, Eigen::Matrix4f &OrinTransform, double LeafSize);
int	orin_align3(const pointcloud::Ptr source_cloud, const pointcloud::Ptr target_cloud, Eigen::Matrix4f &OrinTransform, double LeafSize);
//在可视化窗口的第一视点显示未匹配源点云和目标点云
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);

//在可视化窗口的第二视点显示配准后源点云和目标点云
//void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
/**---------------------------------------------------------------
* 加载数据，迭代其他程序的参数，检查每一个参数是否指向一个pcd文件**
* 如果是，则创建一个添加到点云矢量data中的pcd对象。              **
* ---------------------------------------------------------------**/
void loadData(int argc, const vector<string>& argv, vector<PCD, Eigen::aligned_allocator<PCD> >& models);

/**--------------------------------------------
  * 进行实际配准，由子函数pairAlign具体实现。**
  *参数 cloud_src 是源点云                   **
  *参数 cloud_tgt 是目标点云                 **
  *参数output输出的配准结果的源点云          **
  *参数final_transform是在来源和目标之间的转换*
  -------------------------------------------*/
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false);


//--------------------------------------------------------------------------------
//主函数检查用户输入，在点云矢量data中加载了用户输入的所有点云数据，建立两个视口的
//可视化对象，左边显示未配准的源和目标点云，右边显示配准的源和目标点云。在为一对点
//云找到变换矩阵后，将目标点云变换到源点云坐标系下，并将源点云与变换后的目标点云存
//储到一点云文件，同时用此次找到的变换矩阵更新全局变换，用于将后续的点云都变换到与
//第一个输入点云同一坐标系下。
//--------------------------------------------------------------------------------
int alignPoint(int argc1, const vector<string>& argv1);

int icp(const string& file1, const string& file2);