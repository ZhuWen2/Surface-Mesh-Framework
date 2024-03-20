#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>//ͳ���˲�
#include <pcl/filters/convolution_3d.h>  // ��˹�˲�
#include <pcl/keypoints/uniform_sampling.h> // ���Ȳ���
#include <pcl/filters/voxel_grid.h>//���ز���
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
//#include <pcl/visualization/cloud_viewer.h>//���ӻ�


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h> 
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/bilateral.h> // ˫���˲�
//#include <pcl/filters/impl/bilateral.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include "PCL_ICP.h"

using namespace std;

pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud);

//txt�ļ�����
void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream file(file_path.c_str());
	std::string line;
	pcl::PointXYZ point;
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		cloud->push_back(point);
	}
	file.close();
}


//ͳ���˲�
bool Statisticalfiltering(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt(file_path, cloud);

	cout << "ԭʼ���Ƹ�����" << cloud->points.size() << endl;
	// -----------------ͳ���˲�-------------------
	// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
	// ����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //���ô��˲��ĵ���
	sor.setMeanK(50);           //�����ڽ���ͳ��ʱ���ǲ�ѯ���ڽ�����
	sor.setStddevMulThresh(1);  //�����ж��Ƿ�Ϊ��Ⱥ�����ֵ����ߵ����ֱ�ʾ��׼��ı�����1����׼�����Ͼ�����Ⱥ�㡣
	//�������жϵ��k����ƽ������(mean distance)����ȫ�ֵ�1����׼��+ƽ������(global distances mean and standard)����Ϊ��Ⱥ�㡣

	sor.filter(*cloud_filtered); //�洢�ڵ�
	cout << "ͳ���˲�֮����Ƶĸ�����" << cloud_filtered->points.size() << endl;
	// �������
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//Statisticalfiltering.pcd", *cloud_filtered, false);
	// ���ӻ�
	return true;
}


//��˹�˲�
bool Gaussianfiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {
	// -----------------------------���ڸ�˹�˺����ľ���˲�ʵ��---------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4);//��˹�����ı�׼������������Ŀ��
	kernel.setThresholdRelativeToSigma(4);//�������Sigma�����ľ�����ֵ
	kernel.setThreshold(0.05);//���þ�����ֵ���������������ֵ���迼��
	//cout << "Kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	//cout << "KdTree made" << endl;

	// -------------------------------����Convolution ��ز���-----------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//���þ����
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	//cout << "Convolution Start" << endl;
	convolution.convolve(*cloud_filtered);

	//----------------------������-------------------
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//Gaussianfiltering.pcd", *cloud_filtered, false);
	//pcl::io::savePCDFileASCII("GS.pcd", *gassFilter);

}



//���ز���
bool VoxelSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{

	cout << "ԭʼ���Ƹ�����" << cloud->points.size() << endl;
	// ----------------�������ز�������-------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);             // �������
	vg.setLeafSize(1.5f, 1.5f, 1.5f); // ������С���ر߳�
	vg.filter(*cloud_filtered);          // �����˲�
	cout << "���ز���֮����Ƶĸ�����" << cloud_filtered->points.size() << endl;

	// ---------------------------------������----------------------------------
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//VoxelSampling.pcd", *cloud_filtered, false);

	//---------------------��ʾ����-----------------------

	return true;
}


//int bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
//{
//	// -------------------------------��ȡ����ǿ����Ϣ��pcd����--------------------------------
//	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
//
//	//if (pcl::io::loadPCDFile<pcl::PointXYZI>("box - Cloud.pcd", *cloud) < 0)
//	//{
//	//	PCL_ERROR("Could not read file\n");
//	//	return (-1);
//	//}
//	//cout << "��������������������Ч�ֶ�Ϊ: " << pcl::getFieldsList(*cloud) << endl;
//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	// --------------------------------------����kdtree----------------------------------------
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//��һ��������Ϊ�βδ���
//	//---------------------------------------˫���˲�------------------------------------------
//	pcl::BilateralFilter<pcl::PointXYZ> bf;
//	bf.setInputCloud(cloud);
//	bf.setSearchMethod(tree);
//	bf.setHalfSize(0.1); // ���ø�˹˫���˲����ڵ�һ���С��
//	bf.setStdDev(0.03);  // ���ñ�׼�����
//	bf.filter(*target_cloud);
//
//	//pcl::io::savePCDFileBinary("box_filtered.pcd", *target_cloud);
//
//
//	return 1;
//}


inline double kernel(double x, double sigma)
{
	return (std::exp(-(x * x) / (2 * sigma * sigma)));
}

// ���㷨����
pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(target_cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.compute(*normals);

	return normals;
}

int bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//std::string incloudfile = "sphere_noisy.pcd";
	std::string outcloudfile = "sphere_noisy_bffilter.ply";

	// ---------------------------------���ص���---------------------------------------
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);
	// -------------------------------���ò�����ֵ-------------------------------------
	float sigma_s = 0.05;
	float sigma_r = 10;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BFcloud(new pcl::PointCloud<pcl::PointXYZ>);
	BFcloud = cloud;
	// -------------------------------����KD������----------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);

	std::vector<int>  k_indices;
	std::vector<float> k_distances;
	// ---------------------------���ڷ��ߵ�˫���˲�--------------------------------
	pcl::PointCloud< pcl::Normal>::Ptr normals_input = computeNormal(cloud);

	for (int point_id = 0; point_id < cloud->size(); ++point_id)
	{
		float BF = 0;
		float W = 0;

		tree->radiusSearch(point_id, 2 * sigma_s, k_indices, k_distances);
		Eigen::Vector3f normal = (*normals_input)[point_id].getNormalVector3fMap();
		// ����ÿһ����
		for (std::size_t n_id = 0; n_id < k_indices.size(); ++n_id)
		{
			int id = k_indices.at(n_id);
			float dist = sqrt(k_distances.at(n_id)); // ����ŷ�Ͼ���

			Eigen::Vector3f  point_p = cloud->points[point_id].getVector3fMap(),
				point_q = cloud->points[k_indices[n_id]].getVector3fMap();
			float normal_dist = normal.dot(point_q - point_p); // ���㷨�߾���
			// �����˹�˺���
			float w_a = kernel(dist, sigma_s);
			float w_b = kernel(normal_dist, sigma_r);
			float weight = w_a * w_b; // w

			BF += weight * normal_dist; //sum_l
			W += weight; //sum_w
		}
		// �˲�֮��ĵ�
		Eigen::Vector3f  point_filter = cloud->points[point_id].getVector3fMap() + (BF / W) * normal;
		BFcloud->points[point_id].x = point_filter[0];
		BFcloud->points[point_id].y = point_filter[1];
		BFcloud->points[point_id].z = point_filter[2];

	}

	pcl::io::savePLYFile(outcloudfile.c_str(), *BFcloud);

	return (0);
}

void xyz2pcd(const string& file, const string& pcd) {
	std::ifstream input_file(file);

	if (!input_file.is_open()) {
		std::cerr << "Error opening input file." << std::endl;
	}

	// ���� PCL ���ƶ���
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// ��ȡ�ı��ļ��е�ÿ���㲢��ӵ����ƶ�����
	double x, y, z;
	while (input_file >> x >> y >> z) {
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		cloud.points.push_back(point);
	}

	input_file.close();

	// �����������Ϊ PCD �ļ�
	pcl::PCDWriter writer;

	cloud.width = cloud.size();
	cloud.height = 1;
	cloud.is_dense = false;
	writer.write<pcl::PointXYZ>(pcd, cloud, false);
}


int main(int argc, char** argv) {

	vector<string> argv2;
	for (int i = 1; i < argc; i++)
	{
		const string tempFile = argv[i];
		const string tempFileS = tempFile.substr(0, tempFile.length() - 3) + (string)"pcd";
		xyz2pcd(tempFile, tempFileS);
		//argv1[i] = (char*)tempFileS.c_str();

		//argv2[i-1] = new char(tempFileS.length() + 1);
		//strcpy(argv2[i-1], tempFileS.c_str());
		argv2.emplace_back(tempFileS);
	}
	alignPoint(argc - 1, argv2);
	string path = "pnt.txt";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt(path, cloud);
	bilateralFilter(cloud);
	//pcl::visualization::CloudViewer viewer("Cloud Viewer"); //����viewer����
	//viewer.showCloud(cloud);

	//ͳ���˲�
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//Statisticalfiltering(path, cloud_filtered);

	////˫���˲�(��ʱ)
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//Bilateralfiltering(cloud_filtered, cloud_filtered2);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBilFilter(new pcl::PointCloud<pcl::PointXYZ>);
	//bilateralFilter(cloud, cloudBilFilter);



	///��˹�˲�'
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//Gaussianfiltering(cloud_filtered, cloud_filtered2);
	return 0;
}





