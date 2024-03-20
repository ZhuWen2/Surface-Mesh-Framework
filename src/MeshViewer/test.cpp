#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波
#include <pcl/filters/convolution_3d.h>  // 高斯滤波
#include <pcl/keypoints/uniform_sampling.h> // 均匀采样
#include <pcl/filters/voxel_grid.h>//体素采样
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
//#include <pcl/visualization/cloud_viewer.h>//可视化


#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h> 
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/bilateral.h> // 双边滤波
//#include <pcl/filters/impl/bilateral.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include "PCL_ICP.h"

using namespace std;

pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud);

//txt文件读入
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


//统计滤波
bool Statisticalfiltering(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	CreateCloudFromTxt(file_path, cloud);

	cout << "原始点云个数：" << cloud->points.size() << endl;
	// -----------------统计滤波-------------------
	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	// 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);   //设置待滤波的点云
	sor.setMeanK(50);           //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
	//即：当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。

	sor.filter(*cloud_filtered); //存储内点
	cout << "统计滤波之后点云的个数：" << cloud_filtered->points.size() << endl;
	// 保存点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//Statisticalfiltering.pcd", *cloud_filtered, false);
	// 可视化
	return true;
}


//高斯滤波
bool Gaussianfiltering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered) {
	// -----------------------------基于高斯核函数的卷积滤波实现---------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);//设置距离阈值，若点间距离大于阈值则不予考虑
	//cout << "Kernel made" << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	//cout << "KdTree made" << endl;

	// -------------------------------设置Convolution 相关参数-----------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(cloud);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);
	//cout << "Convolution Start" << endl;
	convolution.convolve(*cloud_filtered);

	//----------------------保存结果-------------------
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//Gaussianfiltering.pcd", *cloud_filtered, false);
	//pcl::io::savePCDFileASCII("GS.pcd", *gassFilter);

}



//体素采样
bool VoxelSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered)
{

	cout << "原始点云个数：" << cloud->points.size() << endl;
	// ----------------创建体素采样对象-------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);             // 输入点云
	vg.setLeafSize(1.5f, 1.5f, 1.5f); // 设置最小体素边长
	vg.filter(*cloud_filtered);          // 进行滤波
	cout << "体素采样之后点云的个数：" << cloud_filtered->points.size() << endl;

	// ---------------------------------保存结果----------------------------------
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("mult_construct//3Dpntdeal//VoxelSampling.pcd", *cloud_filtered, false);

	//---------------------显示点云-----------------------

	return true;
}


//int bilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
//{
//	// -------------------------------读取带有强度信息的pcd点云--------------------------------
//	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
//
//	//if (pcl::io::loadPCDFile<pcl::PointXYZI>("box - Cloud.pcd", *cloud) < 0)
//	//{
//	//	PCL_ERROR("Could not read file\n");
//	//	return (-1);
//	//}
//	//cout << "点云数据中所包含的有效字段为: " << pcl::getFieldsList(*cloud) << endl;
//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	// --------------------------------------建立kdtree----------------------------------------
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//用一个子类作为形参传入
//	//---------------------------------------双边滤波------------------------------------------
//	pcl::BilateralFilter<pcl::PointXYZ> bf;
//	bf.setInputCloud(cloud);
//	bf.setSearchMethod(tree);
//	bf.setHalfSize(0.1); // 设置高斯双边滤波窗口的一半大小。
//	bf.setStdDev(0.03);  // 设置标准差参数
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

// 计算法向量
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

	// ---------------------------------加载点云---------------------------------------
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);
	// -------------------------------设置参数阈值-------------------------------------
	float sigma_s = 0.05;
	float sigma_r = 10;
	pcl::PointCloud<pcl::PointXYZ>::Ptr BFcloud(new pcl::PointCloud<pcl::PointXYZ>);
	BFcloud = cloud;
	// -------------------------------建立KD树索引----------------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(cloud);

	std::vector<int>  k_indices;
	std::vector<float> k_distances;
	// ---------------------------基于法线的双边滤波--------------------------------
	pcl::PointCloud< pcl::Normal>::Ptr normals_input = computeNormal(cloud);

	for (int point_id = 0; point_id < cloud->size(); ++point_id)
	{
		float BF = 0;
		float W = 0;

		tree->radiusSearch(point_id, 2 * sigma_s, k_indices, k_distances);
		Eigen::Vector3f normal = (*normals_input)[point_id].getNormalVector3fMap();
		// 遍历每一个点
		for (std::size_t n_id = 0; n_id < k_indices.size(); ++n_id)
		{
			int id = k_indices.at(n_id);
			float dist = sqrt(k_distances.at(n_id)); // 计算欧氏距离

			Eigen::Vector3f  point_p = cloud->points[point_id].getVector3fMap(),
				point_q = cloud->points[k_indices[n_id]].getVector3fMap();
			float normal_dist = normal.dot(point_q - point_p); // 计算法线距离
			// 计算高斯核函数
			float w_a = kernel(dist, sigma_s);
			float w_b = kernel(normal_dist, sigma_r);
			float weight = w_a * w_b; // w

			BF += weight * normal_dist; //sum_l
			W += weight; //sum_w
		}
		// 滤波之后的点
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

	// 创建 PCL 点云对象
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// 读取文本文件中的每个点并添加到点云对象中
	double x, y, z;
	while (input_file >> x >> y >> z) {
		pcl::PointXYZ point;
		point.x = x;
		point.y = y;
		point.z = z;
		cloud.points.push_back(point);
	}

	input_file.close();

	// 保存点云数据为 PCD 文件
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
	//pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	//viewer.showCloud(cloud);

	//统计滤波
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//Statisticalfiltering(path, cloud_filtered);

	////双边滤波(耗时)
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//Bilateralfiltering(cloud_filtered, cloud_filtered2);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBilFilter(new pcl::PointCloud<pcl::PointXYZ>);
	//bilateralFilter(cloud, cloudBilFilter);



	///高斯滤波'
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	//Gaussianfiltering(cloud_filtered, cloud_filtered2);
	return 0;
}





