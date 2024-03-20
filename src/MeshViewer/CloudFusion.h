#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp算法
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//体素下采样滤波
#include <pcl/features/fpfh_omp.h> //fpfh加速计算的omp(多核并行计算)
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd.h> //奇异矩阵分解计算变换矩阵
#include <pcl/registration/ia_ransac.h>//sac_ia算法
#include <fstream>
#include <time.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <cstdlib>
#include <ctime>
#include <math.h>

//#include <pcl/registration/ia_kfpcs.h>
#include "MeshDefinition.h"
using namespace std;

class CF
{
public:
	clock_t whole_time;

	CF() {
		whole_time=0;
	};
	void PCS(int i);
	void sacia(int i);
	void getOverlappedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped_cloud2);
	void calaPointCloudCoincide(int m, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1);
	void find_corr(int m, int n);
	//void find_corr();


};

//笛卡尔坐标系中三维点坐标
typedef struct st_pointxyz
{
	float x;
	float y;
	float z;
}st_pointxyz;

typedef struct st_point
{
	st_pointxyz pnt;
	int groupID;
	st_point() {};
	st_point(st_pointxyz& p, int id)
	{
		pnt = p;
		groupID = id;
	}
}st_point;


class KMeans
{
public:


	int m_k;
	std::vector< st_pointxyz> pc_arr;

	typedef std::vector<st_point> VecPoint_t;

	VecPoint_t mv_pntcloud1, mv_pntcloud2;    //要聚类的点云
	std::vector<VecPoint_t> m_grp_pntcloud1, m_grp_pntcloud2, m_grp_pntcloud;    //K类，每一类存储若干点
	std::vector<st_pointxyz> mv_center;    //每个类的中心

	KMeans()
	{
		m_k = 0;
	}

	inline void SetK(int k_)
	{
		m_k = k_;
		m_grp_pntcloud.resize(m_k);
		m_grp_pntcloud1.resize(m_k);
		m_grp_pntcloud2.resize(m_k);
	}
	//设置输入点云
	bool SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2);

	//降采样获得种子点
	bool SetInitKCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2, pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal>::Ptr nss);
	//初始化最初的K个类的中心
	bool InitKCenter(std::vector< st_pointxyz> pc_arr);

	//聚类
	bool Cluster();

	//更新K类的中心
	bool UpdateGroupCenter(std::vector<VecPoint_t>& grp_pntcloud, std::vector<st_pointxyz>& center);

	//计算两个点间的欧氏距离
	double DistBetweenPoints(st_pointxyz& p1, st_pointxyz& p2);

	//是否存在中心点移动
	bool ExistCenterShift(std::vector<st_pointxyz>& prev_center, std::vector<st_pointxyz>& cur_center);

	//把聚类的点融合
	bool Mix(const string dir_name, const string prex_name, int a, int b);
	//bool Mix(const string dir_name, const string prex_name);
	//将聚类的点分别存到各自的pcd文件中
	//bool SaveFile(const char* prex_name);
	
	
	
	//将聚类的点分别存到各自的pcd文件中
	bool SaveFile(const string dir_name, const string prex_name);

	bool clearr();
};

void Cloud_Reg_Fus();