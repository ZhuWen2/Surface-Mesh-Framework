#pragma once
#include <pcl/point_types.h>                  // �����Ͷ������ͷ�ļ�
#include <pcl/point_cloud.h>                  // �����Ͷ������ͷ�ļ�
#include <pcl/io/pcd_io.h>                    // PCD�ļ��򿪴洢�����ͷ�ļ�
#include <pcl/filters/voxel_grid.h>           // �����˲�
#include <pcl/features/normal_3d.h>           // �������������ͷ�ļ�
#include <pcl/registration/icp_nl.h>          // ������LM-ICP���ͷ�ļ�
#include <pcl/registration/transforms.h>      // �任���������ͷ�ļ�
//#include <pcl/visualization/pcl_visualizer.h> // ���ӻ�ͷ�ļ�
#include <boost/make_shared.hpp>              // boostָ�����ͷ�ļ�

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <boost/thread/thread.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>//���������һ������׼

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/common/time.h>//����ʱ��
#include <pcl/console/print.h>//PCL����̨���

#include <pcl/features/normal_3d_omp.h>//ʹ��OMP��Ҫ��ӵ�ͷ�ļ�
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/gicp.h>  
#include <pcl/console/time.h>
#include <pcl/registration/icp.h> // icp�㷨

#include <pcl/registration/ia_kfpcs.h> //K4PCS�㷨ͷ�ļ�

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


//�����Ͷ���
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

struct PCD
{
	PointCloud::Ptr cloud;           // ���ƹ���ָ��
	std::string f_name;              // �ļ�����
	PCD() : cloud(new PointCloud) {};
};

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud);

int orin_align(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &OrinTransform);
int orin_align1(const pcl::PointCloud<PointNT>::Ptr object, const pcl::PointCloud<PointNT>::Ptr scene, Eigen::Matrix4f &OrinTransform);
int orin_align2(const pointcloud::Ptr source_cloud, const pointcloud::Ptr target_cloud, Eigen::Matrix4f &OrinTransform, double LeafSize);
int	orin_align3(const pointcloud::Ptr source_cloud, const pointcloud::Ptr target_cloud, Eigen::Matrix4f &OrinTransform, double LeafSize);
//�ڿ��ӻ����ڵĵ�һ�ӵ���ʾδƥ��Դ���ƺ�Ŀ�����
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source);

//�ڿ��ӻ����ڵĵڶ��ӵ���ʾ��׼��Դ���ƺ�Ŀ�����
//void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source);
/**---------------------------------------------------------------
* �������ݣ�������������Ĳ��������ÿһ�������Ƿ�ָ��һ��pcd�ļ�**
* ����ǣ��򴴽�һ����ӵ�����ʸ��data�е�pcd����              **
* ---------------------------------------------------------------**/
void loadData(int argc, const vector<string>& argv, vector<PCD, Eigen::aligned_allocator<PCD> >& models);

/**--------------------------------------------
  * ����ʵ����׼�����Ӻ���pairAlign����ʵ�֡�**
  *���� cloud_src ��Դ����                   **
  *���� cloud_tgt ��Ŀ�����                 **
  *����output�������׼�����Դ����          **
  *����final_transform������Դ��Ŀ��֮���ת��*
  -------------------------------------------*/
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample = false);


//--------------------------------------------------------------------------------
//����������û����룬�ڵ���ʸ��data�м������û���������е������ݣ����������ӿڵ�
//���ӻ����������ʾδ��׼��Դ��Ŀ����ƣ��ұ���ʾ��׼��Դ��Ŀ����ơ���Ϊһ�Ե�
//���ҵ��任����󣬽�Ŀ����Ʊ任��Դ��������ϵ�£�����Դ������任���Ŀ����ƴ�
//����һ�����ļ���ͬʱ�ô˴��ҵ��ı任�������ȫ�ֱ任�����ڽ������ĵ��ƶ��任����
//��һ���������ͬһ����ϵ�¡�
//--------------------------------------------------------------------------------
int alignPoint(int argc1, const vector<string>& argv1);

int icp(const string& file1, const string& file2);