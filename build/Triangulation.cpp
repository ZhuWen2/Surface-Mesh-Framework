#include "pch.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <array>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/disable_warnings.h>
#include "Triangulation.h"
#include <chrono>

#include "pch.h"
#include "framework.h"

using namespace std::chrono;

typedef std::array<std::size_t, 3> Facet;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3  Point_3;
typedef CGAL::Surface_mesh<Point_3> Meshh;
struct Construct {
	Meshh& mesh;
	template < typename PointIterator>
	Construct(Meshh& mesh, PointIterator b, PointIterator e)
		: mesh(mesh)
	{
		for (; b != e; ++b) {
			boost::graph_traits<Meshh>::vertex_descriptor v;
			v = add_vertex(mesh);
			mesh.point(v) = *b;
		}
	}
	Construct& operator=(const Facet f)
	{
		typedef boost::graph_traits<Meshh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<Meshh>::vertices_size_type size_type;
		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
			vertex_descriptor(static_cast<size_type>(f[1])),
			vertex_descriptor(static_cast<size_type>(f[2])));
		return *this;
	}
	Construct&
		operator*() { return *this; }
	Construct&
		operator++() { return *this; }
	Construct
		operator++(int) { return *this; }
};
int TriangulationCloud()
{
	std::ifstream in(CGAL::data_file_path("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\测试数据\\cupeiz1.txt"));
	//std::ifstream in("./gangti.txt");

	auto start = steady_clock::now();

	std::vector<Point_3> points;
	Meshh m;
	std::copy(std::istream_iterator<Point_3>(in),
		std::istream_iterator<Point_3>(),
		std::back_inserter(points));

	int pointSize = points.size();
	//std::random_shuffle(points.begin(), points.end());

	Construct construct(m, points.begin(), points.begin()+ pointSize);
	CGAL::advancing_front_surface_reconstruction(points.begin(), points.begin() + pointSize,construct);

	auto end = steady_clock::now();

	std::string outputFile();
	std::ofstream out("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\测试数据\\cupeiz1.off");	
	out << m << std::endl;
	auto end1 = steady_clock::now();

	auto time1 = duration_cast<microseconds>(end - start);
	auto time2 = duration_cast<microseconds>(end1 - start);

	std::cout << "不加读文件网格封装时间是：" << time1.count()/1e6 << "s" <<std::endl;
	std::cout << "加读文件网格封装时间是：" << time2.count()/1e6 << "s" << std::endl;
	//std::cout << m << std::endl;
	return 0;
}











//
//
//#include <iostream>
//#include <fstream>
//#include <algorithm>
//#include <array>
//#include "Triangulation.h"
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h> // 贪婪投影三角化算法
////#include <pcl/visualization/pcl_visualizer.h>
//int TriangulationCloud()
//{
//
//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PCLPointCloud2 cloud_blob; // pcl::PCLPointCloud2是一种通用的点云数据表示格式，可以存储不同类型和不同结构的点云数据
//		pcl::io::loadPLYFile("D:/XTOP/XTOPwork/mesh/meshview/Surface-Mesh-Framework-main/测试数据/融合前降5.ply", cloud_blob);
//		pcl::fromPCLPointCloud2(cloud_blob, *cloud); // 转换为特定类型的点云数据
//
//		// Normal estimation
//		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; // 法线估计
//		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//		tree->setInputCloud(cloud);
//		n.setInputCloud(cloud);
//		n.setSearchMethod(tree);
//		n.setKSearch(20);
//		n.compute(*normals);
//
//		// Concatenate the XYZ and normal fields
//		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//
//		// 定义搜索树对象
//		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//		tree2->setInputCloud(cloud_with_normals); // 点云构建搜索树
//
//		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化对象
//		pcl::PolygonMesh triangles; // 用于存储最终三角化的模型
//
//		// 三角化操作
//		gp3.setSearchRadius(4);
//
//		gp3.setMu(2.5); // 表示距离参数，用于控制点云投影到三角化表面时的最近邻数量，值越大表示距离越近
//		gp3.setMaximumNearestNeighbors(100); // 设置点云的最大最近邻数量
//		gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置最大表面角度，用于控制三角化结果的平滑程
//		gp3.setMinimumAngle(M_PI / 180); // 设置最小角度，用于控制三角化结果的细节程度
//		gp3.setMaximumAngle(2 * M_PI / 3); // 设置最大角度，用于控制三角化结果的细节程度
//		// 以上都是默认参数，可以不设置
//
//		gp3.setNormalConsistency(true); //  设置法向一致性，表示是否保持法向一致；如果设置为true，则生成三角化结果时会对法向进行一致性检查和调整
//
//		gp3.setInputCloud(cloud_with_normals);  // 设置带有法向信息的输入点云数据
//		gp3.setSearchMethod(tree2); // 设置搜索方法，用于三角化操作时的点云搜索
//		gp3.reconstruct(triangles); // 进行三角化操作，生成三角化结果
//
//		std::vector<int> parts = gp3.getPartIDs(); // 获取三角化结果的部分ID信息
//		std::vector<int> states = gp3.getPointStates(); // 获取三角化结果的状态信息
//
//		pcl::io::savePLYFile("D:/XTOP/XTOPwork/mesh/meshview/Surface-Mesh-Framework-main/测试数据/融合前降5mesh.ply", triangles);
//
//		//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud viewer"));
//		//viewer->addPolygonMesh(triangles, "my");
//		//viewer->addCoordinateSystem(1.0);
//		//viewer->initCameraParameters();
//
//		//while (!viewer->wasStopped()) {
//		//	viewer->spinOnce();
//		//}
//
//
//	return 0;
//}