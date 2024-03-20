#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/icp.h> // icp�㷨
#include <pcl/io/ply_io.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>//�����²����˲�
#include <pcl/features/fpfh_omp.h> //fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <boost/thread/thread.hpp>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd.h> //�������ֽ����任����
#include <pcl/registration/ia_ransac.h>//sac_ia�㷨
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

//�ѿ�������ϵ����ά������
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

	VecPoint_t mv_pntcloud1, mv_pntcloud2;    //Ҫ����ĵ���
	std::vector<VecPoint_t> m_grp_pntcloud1, m_grp_pntcloud2, m_grp_pntcloud;    //K�࣬ÿһ��洢���ɵ�
	std::vector<st_pointxyz> mv_center;    //ÿ���������

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
	//�����������
	bool SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2);

	//������������ӵ�
	bool SetInitKCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2, pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal>::Ptr nss);
	//��ʼ�������K���������
	bool InitKCenter(std::vector< st_pointxyz> pc_arr);

	//����
	bool Cluster();

	//����K�������
	bool UpdateGroupCenter(std::vector<VecPoint_t>& grp_pntcloud, std::vector<st_pointxyz>& center);

	//������������ŷ�Ͼ���
	double DistBetweenPoints(st_pointxyz& p1, st_pointxyz& p2);

	//�Ƿ�������ĵ��ƶ�
	bool ExistCenterShift(std::vector<st_pointxyz>& prev_center, std::vector<st_pointxyz>& cur_center);

	//�Ѿ���ĵ��ں�
	bool Mix(const string dir_name, const string prex_name, int a, int b);
	//bool Mix(const string dir_name, const string prex_name);
	//������ĵ�ֱ�浽���Ե�pcd�ļ���
	//bool SaveFile(const char* prex_name);
	
	
	
	//������ĵ�ֱ�浽���Ե�pcd�ļ���
	bool SaveFile(const string dir_name, const string prex_name);

	bool clearr();
};

void Cloud_Reg_Fus();