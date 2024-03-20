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
	std::ifstream in(CGAL::data_file_path("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\cupeiz1.txt"));
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
	std::ofstream out("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\cupeiz1.off");	
	out << m << std::endl;
	auto end1 = steady_clock::now();

	auto time1 = duration_cast<microseconds>(end - start);
	auto time2 = duration_cast<microseconds>(end1 - start);

	std::cout << "���Ӷ��ļ������װʱ���ǣ�" << time1.count()/1e6 << "s" <<std::endl;
	std::cout << "�Ӷ��ļ������װʱ���ǣ�" << time2.count()/1e6 << "s" << std::endl;
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
//#include <pcl/surface/gp3.h> // ̰��ͶӰ���ǻ��㷨
////#include <pcl/visualization/pcl_visualizer.h>
//int TriangulationCloud()
//{
//
//
//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::PCLPointCloud2 cloud_blob; // pcl::PCLPointCloud2��һ��ͨ�õĵ������ݱ�ʾ��ʽ�����Դ洢��ͬ���ͺͲ�ͬ�ṹ�ĵ�������
//		pcl::io::loadPLYFile("D:/XTOP/XTOPwork/mesh/meshview/Surface-Mesh-Framework-main/��������/�ں�ǰ��5.ply", cloud_blob);
//		pcl::fromPCLPointCloud2(cloud_blob, *cloud); // ת��Ϊ�ض����͵ĵ�������
//
//		// Normal estimation
//		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; // ���߹���
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
//		// ��������������
//		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//		tree2->setInputCloud(cloud_with_normals); // ���ƹ���������
//
//		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // �������ǻ�����
//		pcl::PolygonMesh triangles; // ���ڴ洢�������ǻ���ģ��
//
//		// ���ǻ�����
//		gp3.setSearchRadius(4);
//
//		gp3.setMu(2.5); // ��ʾ������������ڿ��Ƶ���ͶӰ�����ǻ�����ʱ�������������ֵԽ���ʾ����Խ��
//		gp3.setMaximumNearestNeighbors(100); // ���õ��Ƶ�������������
//		gp3.setMaximumSurfaceAngle(M_PI / 4); // ����������Ƕȣ����ڿ������ǻ������ƽ����
//		gp3.setMinimumAngle(M_PI / 180); // ������С�Ƕȣ����ڿ������ǻ������ϸ�ڳ̶�
//		gp3.setMaximumAngle(2 * M_PI / 3); // �������Ƕȣ����ڿ������ǻ������ϸ�ڳ̶�
//		// ���϶���Ĭ�ϲ��������Բ�����
//
//		gp3.setNormalConsistency(true); //  ���÷���һ���ԣ���ʾ�Ƿ񱣳ַ���һ�£��������Ϊtrue�����������ǻ����ʱ��Է������һ���Լ��͵���
//
//		gp3.setInputCloud(cloud_with_normals);  // ���ô��з�����Ϣ�������������
//		gp3.setSearchMethod(tree2); // ���������������������ǻ�����ʱ�ĵ�������
//		gp3.reconstruct(triangles); // �������ǻ��������������ǻ����
//
//		std::vector<int> parts = gp3.getPartIDs(); // ��ȡ���ǻ�����Ĳ���ID��Ϣ
//		std::vector<int> states = gp3.getPointStates(); // ��ȡ���ǻ������״̬��Ϣ
//
//		pcl::io::savePLYFile("D:/XTOP/XTOPwork/mesh/meshview/Surface-Mesh-Framework-main/��������/�ں�ǰ��5mesh.ply", triangles);
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