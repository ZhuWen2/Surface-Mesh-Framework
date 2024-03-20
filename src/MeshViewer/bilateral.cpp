#include "bilateral.h"

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

int bilateralFilter(Mesh mesh)
{
	//std::string incloudfile = "sphere_noisy.pcd";
	std::string outcloudfile = "outputdata/sphere_noisy_bffilter.ply";

	// ---------------------------------加载点云---------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(mesh.n_vertices());
	int pointInd = 0;
	for (auto it : mesh.vertices())
	{
		auto res = mesh.point(it);
		cloud->points[pointInd].x = res[0];
		cloud->points[pointInd].y = res[1];
		cloud->points[pointInd].z = res[2];
		pointInd++;
	}
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


