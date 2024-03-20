#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h> // icp算法
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>   // 利用控制台计算时间

using namespace std;

#include "PCL_ICP.h"

//pcl::visualization::PCLVisualizer* pVisualizer; // 创建可视化对象
int vp_1, vp_2;                       // 定义存储左右视点ID

//-------------------------------------------------------------------------
//声明一个结构体，方便对点云以及文件名和点云对象进行成对处理管理，在配准
//过程中，可以同时接受多个点云文件输入，程序从第一个文件开始，连续的两两
//配准处理，然后存储配准后的点云文件。
//-------------------------------------------------------------------------

struct PCDComparator                 // 文件比较处理
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};

//------------------------------------------------------------------------
//以< x, y, z, curvature >形式定义一个新的点
//pcl::PointRepresentation提供一组方法，用于将点结构体/对象转换为n维向量。
//------------------------------------------------------------------------
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //**************1****************
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		nr_dimensions_ = 4; //定义点的维度
	}
	//重载copyToFloatArray方法将点转化为4维数组
	virtual void copyToFloatArray(const PointNormalT& p01, float* out) const
	{
		// < x, y, z, curvature >
		out[0] = p01.x;
		out[1] = p01.y;
		out[2] = p01.z;
		out[3] = p01.curvature;
	}
};

fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//-------------------------法向量估计-----------------------
	pointnormal::Ptr normals(new pointnormal);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(input_cloud);
	n.setNumberOfThreads(8);        // 设置openMP的线程数
	n.setSearchMethod(tree);        // 搜索方式
	n.setKSearch(10);               // K近邻点个数
	//n.setRadiusSearch(0.01);      // 搜索半径
	n.compute(*normals);            // 计算法线
	//-------------------------FPFH估计-------------------------
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setNumberOfThreads(8);     //指定8核计算
	fest.setInputCloud(input_cloud);//输入点云
	fest.setInputNormals(normals);  //输入法线
	fest.setSearchMethod(tree);     //搜索方式
	fest.setKSearch(10);            //K近邻点个数
	//fest.setRadiusSearch(0.025);  //搜索半径
	fest.compute(*fpfh);            //计算FPFH

	return fpfh;
}




//在可视化窗口的第一视点显示未匹配源点云和目标点云
//void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
//{
//	pVisualizer->setWindowName("多幅点云配准");
//	pVisualizer->removePointCloud("vp1_target");
//	pVisualizer->removePointCloud("vp1_source");
//	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
//	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
//	pVisualizer->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
//	pVisualizer->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);
//	PCL_INFO("Press q to begin the registration.\n");
//	pVisualizer->addText("Red is the source point cloud and green is the target point cloud", 10, 10, "text", vp_1);
//	pVisualizer->spin();
//}

//在可视化窗口的第二视点显示配准后源点云和目标点云
//void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
//{
//	pVisualizer->removePointCloud("source");
//	pVisualizer->removePointCloud("target");
//	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
//	if (!tgt_color_handler.isCapable())
//		PCL_WARN("Cannot create curvature color handler!");
//	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
//	if (!src_color_handler.isCapable())
//		PCL_WARN("Cannot create curvature color handler!");
//	pVisualizer->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
//	pVisualizer->addPointCloud(cloud_source, src_color_handler, "source", vp_2);
//	pVisualizer->addText("The point cloud after registration", 10, 10, "text", vp_2);
//	pVisualizer->spinOnce();
//}
/**---------------------------------------------------------------
* 加载数据，迭代其他程序的参数，检查每一个参数是否指向一个pcd文件**
* 如果是，则创建一个添加到点云矢量data中的pcd对象。              **
* ---------------------------------------------------------------**/
void loadData(int argc, const vector<string>& argv, vector<PCD, Eigen::aligned_allocator<PCD> >& models)
{
	string extension(".pcd");
	//第一个参数是命令本身，所以要从第二个参数开始解析
	for (int i = 0; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// PCD文件名至少为5个字符大小字符串（因为后缀名.pcd就已经占了四个字符位置）
		if (fname.size() <= extension.size())
			continue;

		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);
		//检查参数是否为一个pcd后缀的文件
		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
		{
			//加载点云并保存在总体的点云列表中
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);
			//从点云中移除NAN点也就是无效点
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);
		}
	}
}

double CalAveragePointSpacing(const PointCloud::Ptr tgt) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(tgt);

	// 设置最近邻的数量 k
	int k = 5;

	// 计算每个点到其 k 个最近邻点的距离
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	double totalDistance = 0.0;
	int totalPairs = 0;
	for (size_t i = 0; i < tgt->size(); ++i) {
		if (kdtree.nearestKSearch(tgt->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			// 计算 k 个最近邻距离的平均值
			for (int j = 0; j < k; ++j) {
				totalDistance += std::sqrt(pointNKNSquaredDistance[j]);
				totalPairs++;
			}
		}
	}
	// 计算平均点间距
	double averagePointSpacing = totalDistance / totalPairs;
	return averagePointSpacing;
}

/**---------------------------------------------------------------
* 粗配准**
* RANSAC加FPFH              **
* ---------------------------------------------------------------**/

double RMSE(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt) {
	double RMSEDist = 0.0;
	int cloudSize = cloud_src0->size();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_tgt);
	double totalDist = 0.0;
	std::vector<int> pointIdxKNNSearch(1);
	std::vector<float> pointKNNSquaredDistance(1);
	int conNum = 0;
	for (int i = 0; i < cloud_src0->size(); i++) {
		kdtree.nearestKSearch((*cloud_src0)[i], 1, pointIdxKNNSearch, pointKNNSquaredDistance);
		if (pointKNNSquaredDistance[0] < 0.5) {
			totalDist += pointKNNSquaredDistance[0] * pointKNNSquaredDistance[0];
			conNum++;
		}

	}
	RMSEDist = sqrt(totalDist / conNum);
	return RMSEDist;
}


float caculateRMSE(const PointCloud::Ptr xyz_source, const PointCloud::Ptr xyz_target)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_source(new pcl::PointCloud<pcl::PointXYZ>());
	//fromPCLPointCloud2(*cloud_source, *xyz_source);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target(new pcl::PointCloud<pcl::PointXYZ>());
	//fromPCLPointCloud2(*cloud_target, *xyz_target);

	float rmse = 0.0f;

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	tree->setInputCloud(xyz_target);

	for (auto point_i : *xyz_source)
	{
		// 去除无效的点
		if (!pcl_isfinite(point_i.x) || !pcl_isfinite(point_i.y) || !pcl_isfinite(point_i.z))
			continue;
		vector<int> nn_indices(1);
		std::vector<float> nn_distances(1);
		if (!tree->nearestKSearch(point_i, 1, nn_indices, nn_distances)) // K近邻搜索获取匹配点对
			continue;
		/*dist的计算方法之一
		size_t point_nn_i = nn_indices.front();
		float dist = squaredEuclideanDistance(point_i, xyz_target->points[point_nn_i]);
		*/

		float dist = nn_distances[0]; // 获取最近邻对应点之间欧氏距离的平方
		rmse += dist;                 // 计算平方距离之和
	}
	rmse = std::sqrt(rmse / static_cast<float> (xyz_source->points.size())); // 计算均方根误差

	return rmse;
}



int orin_align(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &OrinTransform)
{

	////---------------------加载点云数据------------------------------
	//pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("1.pcd", *source);
	//pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("2.pcd", *target);

	//---------------计算源点云和目标点云的FPFH----------------------
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target);

	//--------------------RANSAC点云配准-----------------------------
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> r_sac;
	r_sac.setInputSource(source);            // 源点云
	r_sac.setInputTarget(target);            // 目标点云
	r_sac.setSourceFeatures(source_fpfh);    // 源点云FPFH特征
	r_sac.setTargetFeatures(target_fpfh);    // 目标点云FPFH特征
	r_sac.setCorrespondenceRandomness(5);    // 在选择随机特征对应时，设置要使用的邻居的数量,数值越大，特征匹配的随机性越大。
	r_sac.setInlierFraction(0.2f);           // 所需的(输入的)inlier分数
	r_sac.setNumberOfSamples(3);             // 每次迭代中使用的采样点数量
	r_sac.setSimilarityThreshold(0.5f);      // 将底层多边形对应拒绝器对象的边缘长度之间的相似阈值设置为[0,1]，其中1为完全匹配。
	r_sac.setMaxCorrespondenceDistance(1.0f);// 内点，阈值 Inlier threshold
	r_sac.setMaximumIterations(100);         // RANSAC 　最大迭代次数
	pointcloud::Ptr align(new pointcloud);
	r_sac.align(*align);

	if (!r_sac.hasConverged()) {
		cout << "配准失败：\n " << endl;
	}
	pcl::transformPointCloud(*source, *align, r_sac.getFinalTransformation());
	cout << "变换矩阵：\n" << r_sac.getFinalTransformation() << endl;
	OrinTransform = r_sac.getFinalTransformation();
	return 0;
}


int orin_align1(const pcl::PointCloud<PointNT>::Ptr object, const pcl::PointCloud<PointNT>::Ptr scene, Eigen::Matrix4f &OrinTransform)
{

	////---------------------加载点云数据------------------------------
	//pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("1.pcd", *source);
	//pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);
	//pcl::io::loadPCDFile("2.pcd", *target);

	// Point clouds
	PointCloudT::Ptr object_aligned(new PointCloudT);
	FeatureCloudT::Ptr object_features(new FeatureCloudT);
	FeatureCloudT::Ptr scene_features(new FeatureCloudT);

	// 下采样
	pcl::console::print_highlight("点云下采样\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 1.1f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	// 估计场景法线
	pcl::console::print_highlight("估计场景点云的法线\n");
	pcl::NormalEstimationOMP<PointNT, PointNT> n;
	n.setNumberOfThreads(4);//设置openMP的线程数
	n.setRadiusSearch(1.0);
	n.setInputCloud(scene);
	n.compute(*scene);

	// 特征估计
	pcl::console::print_highlight("计算FPFH特征\n");
	FeatureEstimationT f;
	f.setKSearch(10);
	//f.setRadiusSearch(0.025);
	f.setInputCloud(object);
	f.setInputNormals(object);
	f.compute(*object_features);
	f.setInputCloud(scene);
	f.setInputNormals(scene);
	f.compute(*scene_features);

	// 实施配准
	pcl::console::print_highlight("开始进行配准\n");
	pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(5000);      //  采样一致性迭代次数
	align.setNumberOfSamples(5);            //  创建假设所需的样本数
	align.setCorrespondenceRandomness(5);   //  使用的临近特征点的数目
	align.setSimilarityThreshold(0.8f);     //  多边形边长度相似度阈值
	align.setMaxCorrespondenceDistance(1.0f); //  判断是否为内点的距离阈值
	align.setInlierFraction(0.15f);         //  接受位姿假设所需的内点比例
	{
		pcl::ScopeTime t("执行配准");
		align.align(*object_aligned);
	}
	pcl::io::savePCDFileASCII("object_aligned.pcd", *object_aligned);

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

		// Show alignment
		//pcl::console::print_highlight("左侧为配准前的位置，右侧为配准后\n");
		//pcl::visualization::PCLVisualizer visu("鲁棒位姿估计");
		//int v1(0), v2(0);
		//visu.setWindowName("鲁棒位姿估计");
		//visu.createViewPort(0, 0, 0.5, 1, v1);
		//visu.createViewPort(0.5, 0, 1, 1, v2);
		//visu.setBackgroundColor(255, 255, 255, v2);
		//visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene", v2);
		//visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned", v2);

		//visu.addPointCloud(object, ColorHandlerT(object, 0.0, 255.0, 0.0), "object_before_aligned", v1);
		//visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 0.0, 255.0), "scene_v2", v1);
		//visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
		//visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_aligned");
		//visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object_before_aligned");
		//visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_v2");
		//visu.spin();
	}
	else
	{
		pcl::console::print_error("配准失败!\n");
		return (1);
	}
	OrinTransform = align.getFinalTransformation();
	return 0;
}

//--------------采样一致性SAC_IA初始配准------------------------
int orin_align2(const pointcloud::Ptr source_cloud, const pointcloud::Ptr target_cloud, Eigen::Matrix4f &OrinTransform, double LeafSize)
{
	int sampleNums = 4;
	if (source_cloud->size() < 1e5) sampleNums = 1;
	clock_t start, end, time;
	start = clock();
	//---------------------------去除源点云的NAN点------------------------
	vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
	cout << "remove *source_cloud nan" << endl;
	//-------------------------源点云下采样滤波-------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vs;
	vs.setLeafSize(LeafSize * sampleNums, LeafSize * sampleNums, LeafSize * sampleNums);
	vs.setInputCloud(source_cloud);
	pointcloud::Ptr source(new pointcloud);
	vs.filter(*source);
	cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;

	//--------------------------去除目标点云的NAN点--------------------
	vector<int> indices_tgt; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
	cout << "remove *target_cloud nan" << endl;
	//----------------------目标点云下采样滤波-------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vt;
	vt.setLeafSize(LeafSize * sampleNums, LeafSize * sampleNums, LeafSize * sampleNums);
	vt.setInputCloud(target_cloud);
	pointcloud::Ptr target(new pointcloud);
	vt.filter(*target);
	cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;
	//---------------计算源点云和目标点云的FPFH------------------------
	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
	fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target);

	//--------------采样一致性SAC_IA初始配准----------------------------
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setMinSampleDistance(LeafSize * sampleNums * 0.5);//设置样本之间的最小距离0.1*************************************
	sac_ia.setMinSampleDistance(1);//设置样本之间的最小距离0.1*************************************
	//sac_ia.setNumberOfSamples(200);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(6); //在选择随机特征对应时，设置要使用的邻居的数量;
	//也就是计算协方差时选择的近邻点个数，该值越大，协防差越精确，但是计算效率越低.(可省)
	//sac_ia.setErrorFunction();//这个调用是可选的
	pointcloud::Ptr align(new pointcloud);
	sac_ia.align(*align);
	end = clock();
	pcl::transformPointCloud(*source_cloud, *align, sac_ia.getFinalTransformation());
	// pcl::io::savePCDFile("crou_output.pcd", *align);
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
	cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
	cout << "变换矩阵：\n" << sac_ia.getFinalTransformation() << endl;
	OrinTransform = sac_ia.getFinalTransformation();
	return 0;
}

//--------------------------K4PCS算法进行配准------------------------------
int	orin_align3(const pointcloud::Ptr source, const pointcloud::Ptr target, Eigen::Matrix4f &OrinTransform, double LeafSize)
{
	pcl::console::TicToc time;
	//----------------------------读取点云数据----------------------------------
	time.tic();
	//--------------------------K4PCS算法进行配准------------------------------
	pcl::registration::KFPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> kfpcs;
	kfpcs.setInputSource(source);  // 源点云
	kfpcs.setInputTarget(target);  // 目标点云
	kfpcs.setApproxOverlap(0.5);   // 源和目标之间的近似重叠。
	kfpcs.setLambda(0.5);          // 平移矩阵的加权系数。(暂时不知道是干什么用的)
	kfpcs.setDelta(0.1, false);  // 配准后源点云和目标点云之间的距离
	//kfpcs.setNumberOfThreads(6);   // OpenMP多线程加速的线程数
	kfpcs.setNumberOfSamples(200); // 配准时要使用的随机采样点数量
	//kfpcs.setMaxComputationTime(1000);//最大计算时间(以秒为单位)。
	pcl::PointCloud<pcl::PointXYZ>::Ptr kpcs(new pcl::PointCloud<pcl::PointXYZ>);
	kfpcs.align(*kpcs);

	cout << "KFPCS配准用时： " << time.toc() << " ms" << endl;
	cout << "变换矩阵：\n" << kfpcs.getFinalTransformation() << endl;
	// 使用创建的变换对为输入的源点云进行变换
	pcl::transformPointCloud(*source, *kpcs, kfpcs.getFinalTransformation());
	// 保存转换后的源点云作为最终的变换输出
	//  pcl::io::savePCDFileASCII ("transformed.pcd", *kpcs);

	return (0);
}

/**--------------------------------------------
  * 进行实际配准，由子函数pairAlign具体实现。**
  *参数 cloud_src 是源点云                   **
  *参数 cloud_tgt 是目标点云                 **
  *参数output输出的配准结果的源点云          **
  *参数final_transform是在来源和目标之间的转换*
  -------------------------------------------*/
void pairAlign(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。
	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储滤波后的目标点云
	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	downsample = false;
	if (downsample)
	{
		grid.setLeafSize(10.0, 10.0, 10.0);//设置滤波时采用的体素大小
		grid.setInputCloud(cloud_src0);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src0;
		tgt = cloud_tgt;
	}

	Eigen::Matrix4f OrinTransform;
	orin_align(cloud_src0, cloud_tgt, OrinTransform);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *cloud_src, OrinTransform, false);

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//举例说明我们自定义点的表示（以上定义）
	MyPointRepresentation point_representation;
	//调整'curvature'尺寸权重以便使它和x, y, z平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);




	// 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 非线性LM-ICP配准
	reg.setTransformationEpsilon(1e-6);//设置收敛判断条件，越小精度越大，收敛也越慢
	//将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
	//注意：根据你的数据集大小来调整
	reg.setMaxCorrespondenceDistance(0.1);
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//**********3**************
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);//设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
	/*-------------------------------------------------------------------
	*手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示*
	*如果迭代N次找到的变换和迭代N-1次中找到的变换之间的差异小于传给ICP的**
	*变换收敛阈值，我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程
	*-------------------------------------------------------------------*/
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//为了可视化的目的保存点云
		points_with_normals_src = reg_result;
		//估计
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);
		//在每一个迭代之间累积转换
		Ti = reg.getFinalTransformation() * Ti;
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())//**************4***************
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//可视化当前状态
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	targetToSource = (OrinTransform * Ti).inverse();

	//把目标点云转换回源点云坐标系下
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	*output += *cloud_src;
	final_transform = targetToSource;
}

void pairAlign1(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{



	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。
	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储滤波后的目标点云
	pcl::VoxelGrid<PointT> grid;           //滤波处理对象
	if (downsample)
	{
		grid.setLeafSize(1.1, 1.1, 1.1);//设置滤波时采用的体素大小
		grid.setInputCloud(cloud_src0);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src0;
		tgt = cloud_tgt;
	}
	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align1(points_with_normals_src, points_with_normals_tgt, OrinTransform);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *cloud_src, OrinTransform, false);



	//举例说明我们自定义点的表示（以上定义）
	MyPointRepresentation point_representation;
	//调整'curvature'尺寸权重以便使它和x, y, z平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 非线性LM-ICP配准
	reg.setTransformationEpsilon(1e-6);//设置收敛判断条件，越小精度越大，收敛也越慢
	//将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
	//注意：根据你的数据集大小来调整
	reg.setMaxCorrespondenceDistance(0.1);
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//**********3**************
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);//设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
	/*-------------------------------------------------------------------
	*手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示*
	*如果迭代N次找到的变换和迭代N-1次中找到的变换之间的差异小于传给ICP的**
	*变换收敛阈值，我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程
	*-------------------------------------------------------------------*/
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//为了可视化的目的保存点云
		points_with_normals_src = reg_result;
		//估计
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);
		//在每一个迭代之间累积转换
		Ti = reg.getFinalTransformation() * Ti;
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())//**************4***************
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
		prev = reg.getLastIncrementalTransformation();
		//可视化当前状态
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	targetToSource = (OrinTransform * Ti).inverse();

	//把目标点云转换回源点云坐标系下
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	*output += *cloud_src;
	final_transform = targetToSource;
}
//点到面的LM-ICP
void pairAlign2(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。


	PointCloud::Ptr src1(new PointCloud);   //存储粗配准后的源点云

	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	double averagePointSpacing = CalAveragePointSpacing(cloud_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align2(cloud_src0, cloud_tgt, OrinTransform, averagePointSpacing);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *src1, OrinTransform, false);

	pcl::io::savePCDFile("../data/cupeiz1.pcd", *src1);
	pcl::io::savePCDFile("../data/cupeiz2.pcd", *cloud_tgt);

	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储粗配准后的目标点云

	//downsample = false;
	if (downsample && cloud_tgt->size() > 1e5)
	{
		grid.setLeafSize(5 * averagePointSpacing, 5 * averagePointSpacing, 5 * averagePointSpacing);//设置滤波时采用的体素大小
		grid.setInputCloud(src1);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = src1;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//举例说明我们自定义点的表示（以上定义）
	MyPointRepresentation point_representation;
	//调整'curvature'尺寸权重以便使它和x, y, z平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 非线性LM-ICP配准/******************  调setTransformationEpsilon和setMaxCorrespondenceDistance *******************/
	reg.setTransformationEpsilon(1e-10);//设置收敛判断条件，越小精度越大，收敛也越慢
	//将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
	//注意：根据你的数据集大小来调整
	reg.setMaxCorrespondenceDistance(averagePointSpacing * 20);
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//**********3**************
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations(2);//设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
	/*-------------------------------------------------------------------
	*手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示*
	*如果迭代N次找到的变换和迭代N-1次中找到的变换之间的差异小于传给ICP的**
	*变换收敛阈值，我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程
	*-------------------------------------------------------------------*/
	for (int i = 0; i < 50; ++i)
	{
		PCL_INFO("Iteration Nr. %d.\n", i);
		//为了可视化的目的保存点云
		points_with_normals_src = reg_result;
		//估计
		reg.setInputSource(points_with_normals_src);
		reg.align(*reg_result);
		//在每一个迭代之间累积转换
		Ti = reg.getFinalTransformation() * Ti;
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())//**************4***************
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.01 * averagePointSpacing * 1.5);
		prev = reg.getLastIncrementalTransformation();
		//可视化当前状态
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	//targetToSource = (OrinTransform * Ti).inverse();
	targetToSource = ( Ti * OrinTransform );

	//把目标点云转换回源点云坐标系下
	//pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	pcl::transformPointCloud(*cloud_src0, *output, targetToSource);
	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	//*output += *cloud_src;

	pcl::io::savePCDFile("../data/jingpingjie1.pcd", *output);
	pcl::io::savePCDFile("../data/jingpingjie2.pcd", *cloud_tgt);

	*output += *cloud_tgt;
	final_transform = targetToSource;

	pcl::io::savePCDFile("../data/outputalgin.pcd", *output);
}

//LMICP
void pairAlign3(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。


	PointCloud::Ptr src1(new PointCloud);   //存储粗配准后的源点云

	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	double averagePointSpacing = CalAveragePointSpacing(cloud_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align2(cloud_src0, cloud_tgt, OrinTransform, averagePointSpacing);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *src1, OrinTransform, false);

	pcl::io::savePCDFile("../data/cupeiz1.pcd", *src1);
	pcl::io::savePCDFile("../data/cupeiz2.pcd", *cloud_tgt);

	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储粗配准后的目标点云

	//downsample = false;
	if (downsample && cloud_tgt->size() > 1e5)
	{
		grid.setLeafSize(5 * averagePointSpacing, 5 * averagePointSpacing, 5 * averagePointSpacing);//设置滤波时采用的体素大小
		grid.setInputCloud(src1);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = src1;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//举例说明我们自定义点的表示（以上定义）
	MyPointRepresentation point_representation;
	//调整'curvature'尺寸权重以便使它和x, y, z平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	// 配准
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 非线性LM-ICP配准/******************  调setTransformationEpsilon和setMaxCorrespondenceDistance *******************/
	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));//**********3**************
	reg.setInputSource(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);
	//


	reg.setTransformationEpsilon(1e-8);//设置收敛判断条件，越小精度越大，收敛也越慢
	//将两个对应关系之间的(src<->tgt)最大距离设置为10厘米
	//注意：根据你的数据集大小来调整
	reg.setEuclideanFitnessEpsilon(1e-5);
	//reg.setRANSACOutlierRejectionThreshold(0.05);
	reg.setMaxCorrespondenceDistance(averagePointSpacing * 10);
	reg.setMaximumIterations(50);//设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
	/*-------------------------------------------------------------------
	*手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示*
	*如果迭代N次找到的变换和迭代N-1次中找到的变换之间的差异小于传给ICP的**
	*变换收敛阈值，我们选择源与目标之间更靠近的对应点距离阈值来改善配准过程
	*-------------------------------------------------------------------*/
	reg.align(*reg_result);

	//在每一个迭代之间累积转换
	Ti = reg.getFinalTransformation();
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	//targetToSource = (OrinTransform * Ti).inverse();
	targetToSource = (Ti * OrinTransform);

	//把目标点云转换回源点云坐标系下
	//pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	pcl::transformPointCloud(*cloud_src0, *output, targetToSource);
	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	//*output += *cloud_src;
	*output += *cloud_tgt;
	final_transform = targetToSource;

	pcl::io::savePCDFile("../data/outputalgin.pcd", *output);
}

//gICP
void pairAlign4(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。


	PointCloud::Ptr src1(new PointCloud);   //存储粗配准后的源点云

	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	double averagePointSpacing = CalAveragePointSpacing(cloud_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align2(cloud_src0, cloud_tgt, OrinTransform, averagePointSpacing);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *src1, OrinTransform, false);

	pcl::io::savePCDFile("../data/cupeiz1.pcd", *src1);
	pcl::io::savePCDFile("../data/cupeiz2.pcd", *cloud_tgt);

	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储粗配准后的目标点云

	//downsample = false;
	if (downsample && cloud_tgt->size() > 1e5)
	{
		grid.setLeafSize(5 * averagePointSpacing, 5 * averagePointSpacing, 5 * averagePointSpacing);//设置滤波时采用的体素大小
		grid.setInputCloud(src1);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = src1;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);


	/******************************* 精配准 ***************************/
	pcl::console::TicToc time;
	time.tic();
	//-----------------初始化GICP对象-------------------------
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	//-----------------KD树加速搜索---------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(tgt);
	gicp.setSearchMethodSource(tree1);
	gicp.setSearchMethodTarget(tree2);
	//-----------------设置GICP相关参数-----------------------
	gicp.setInputSource(src);  //源点云
	gicp.setInputTarget(tgt);  //目标点云
	gicp.setMaxCorrespondenceDistance(averagePointSpacing * 1); //设置对应点对之间的最大距离
	gicp.setTransformationEpsilon(1e-10);   //为终止条件设置最小转换差异
		/* gicp.setSourceCovariances(source_covariances);
	gicp.setTargetCovariances(target_covariances);*/
	gicp.setEuclideanFitnessEpsilon(1e-7);  //设置收敛条件是均方误差和小于阈值， 停止迭代
	gicp.setMaximumIterations(1000); //最大迭代次数  
	//gicp.setUseReciprocalCorrespondences(true);//使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	gicp.align(*icp_cloud);
	//---------------输出必要信息到显示--------------------
	if (gicp.hasConverged()) {
		cout << "Applied " << gicp.getRANSACIterations() << " GICP iterations in " << time.toc() / 1000 << " s" << endl;
		cout << "\nGICP has converged, score is " << gicp.getFitnessScore() << endl;
		cout << "变换矩阵：\n" << gicp.getFinalTransformation() << endl;
	}


	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;

	//在每一个迭代之间累积转换
	Ti = gicp.getFinalTransformation();
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	//targetToSource = (OrinTransform * Ti).inverse();
	targetToSource = (OrinTransform * Ti);

	//把目标点云转换回源点云坐标系下
	//pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	pcl::transformPointCloud(*cloud_src0, *output, targetToSource);

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	//*output += *cloud_src;
	*output += *cloud_tgt;
	final_transform = targetToSource;

	pcl::io::savePCDFile("../data/outputalgin.pcd", *output);
}

//点到面
void pairAlign5(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。


	PointCloud::Ptr src1(new PointCloud);   //存储粗配准后的源点云

	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	double averagePointSpacing = CalAveragePointSpacing(cloud_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align2(cloud_src0, cloud_tgt, OrinTransform, averagePointSpacing);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *src1, OrinTransform, false);

	pcl::io::savePCDFile("../data/cupeiz1.pcd", *src1);
	pcl::io::savePCDFile("../data/cupeiz2.pcd", *cloud_tgt);

	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储粗配准后的目标点云

	//downsample = false;
	if (downsample && cloud_tgt->size() > 1e5)
	{
		grid.setLeafSize(5 * averagePointSpacing, 5 * averagePointSpacing, 5 * averagePointSpacing);//设置滤波时采用的体素大小
		grid.setInputCloud(src1);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = src1;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);


	/******************************* 精配准 ***************************/
	pcl::console::TicToc time;
	// --------------------加载源点云-----------------------
	time.tic();
	//--------------------初始化ICP对象--------------------
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//---------------------KD树加速搜索--------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(tgt);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	//----------------------icp核心代码--------------------
	icp.setInputSource(src);            // 源点云
	icp.setInputTarget(tgt);            // 目标点云
	//icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(averagePointSpacing * 1);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	//icp.setEuclideanFitnessEpsilon(1e-7);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(1000);           // 最大迭代次数
	//icp.setUseReciprocalCorrespondences(true);//使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*icp_cloud);
	if (icp.hasConverged()) {
		cout << "Applied " << 100 << " ICP iterations in " << time.toc() << " ms" << endl;
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << "\t" << icp.getEuclideanFitnessEpsilon() << endl;
		cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
	}
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;
	//在每一个迭代之间累积转换
	Ti = icp.getFinalTransformation();
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	//targetToSource = (OrinTransform * Ti).inverse();
	targetToSource = (OrinTransform * Ti);

	//把目标点云转换回源点云坐标系下
	//pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	pcl::transformPointCloud(*cloud_src0, *output, targetToSource);

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	//*output += *cloud_src;
	*output += *cloud_tgt;
	final_transform = targetToSource;

	pcl::io::savePCDFile("../data/outputalgin.pcd", *output);
}

//点到点
void pairAlign6(const PointCloud::Ptr cloud_src0, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f& final_transform, bool downsample)
{
	//对点云进行下采样，这对于大型数据集是非常实用的，可以加快后期的处理速度。


	PointCloud::Ptr src1(new PointCloud);   //存储粗配准后的源点云

	pcl::VoxelGrid<PointT> grid;           //滤波处理对象

	double averagePointSpacing = CalAveragePointSpacing(cloud_tgt);

	Eigen::Matrix4f OrinTransform;
	orin_align2(cloud_src0, cloud_tgt, OrinTransform, averagePointSpacing);
	PointCloud::Ptr cloud_src(new PointCloud);
	pcl::transformPointCloud(*cloud_src0, *src1, OrinTransform, false);

	pcl::io::savePCDFile("../data/cupeiz1.pcd", *src1);
	pcl::io::savePCDFile("../data/cupeiz2.pcd", *cloud_tgt);

	PointCloud::Ptr src(new PointCloud);   //存储滤波后的源点云
	PointCloud::Ptr tgt(new PointCloud);   //存储粗配准后的目标点云

	//downsample = false;
	if (downsample && cloud_tgt->size() > 1e5)
	{
		grid.setLeafSize(5 * averagePointSpacing, 5 * averagePointSpacing, 5 * averagePointSpacing);//设置滤波时采用的体素大小
		grid.setInputCloud(src1);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = src1;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);
	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	//pcl::copyPointCloud(*src, *points_with_normals_src); //*****************************2*************************
	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	//pcl::copyPointCloud(*tgt, *points_with_normals_tgt);


	/******************************* 精配准 ***************************/
	pcl::console::TicToc time;
	// --------------------加载源点云-----------------------
	time.tic();
	//--------------------初始化ICP对象--------------------
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	//---------------------KD树加速搜索--------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(tgt);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	//----------------------icp核心代码--------------------
	icp.setInputSource(src);            // 源点云
	icp.setInputTarget(tgt);            // 目标点云
	icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(averagePointSpacing * 10);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(1e-4);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(100);           // 最大迭代次数
	//icp.setUseReciprocalCorrespondences(true);//使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*icp_cloud);
	if (icp.hasConverged()) {
		cout << "Applied " << 100 << " ICP iterations in " << time.toc() << " ms" << endl;
		cout << "\nICP has converged, score is " << icp.getFitnessScore() << "\t" << icp.getEuclideanFitnessEpsilon() << endl;
		cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;
	}
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;
	//在每一个迭代之间累积转换
	Ti = icp.getFinalTransformation();
	/*------------------------------------------------------------------------
	* 一旦找到最优的变换，ICP返回的变换是从源点云到目标点云的变换矩阵，求逆
	* 变换得到从目标点云到源点云的变换矩阵，并应用到目标点云，变换后的目标点云
	* 然后添加到源点云中，并且将点云和变换矩阵一起返回到主函数。
	*------------------------------------------------------------------------*/
	//得到目标点云到源点云的变换
	//targetToSource = (OrinTransform * Ti).inverse();
	targetToSource = (Ti * OrinTransform);

	//把目标点云转换回源点云坐标系下
	//pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
	pcl::transformPointCloud(*cloud_src0, *output, targetToSource);

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	//PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	//pVisualizer->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	//pVisualizer->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);
	//PCL_INFO("Press q to continue the registration.\n");
	//pVisualizer->spin();

	//pVisualizer->removePointCloud("source");
	//pVisualizer->removePointCloud("target");
	//添加源点云到转换目标
	//*output += *cloud_src;
	pcl::io::savePCDFile("../data/jingpingjie1.pcd", *output);
	pcl::io::savePCDFile("../data/jingpingjie2.pcd", *cloud_tgt);

	cout << "RMSE值为:" << RMSE(cloud_tgt, output) << "	调别人函数计算" << caculateRMSE(cloud_tgt, output) << endl;

	*output += *cloud_tgt;
	final_transform = targetToSource;

	pcl::io::savePCDFile("../data/outputalgin.pcd", *output);
}


//--------------------------------------------------------------------------------
//主函数检查用户输入，在点云矢量data中加载了用户输入的所有点云数据，建立两个视口的
//可视化对象，左边显示未配准的源和目标点云，右边显示配准的源和目标点云。在为一对点
//云找到变换矩阵后，将目标点云变换到源点云坐标系下，并将源点云与变换后的目标点云存
//储到一点云文件，同时用此次找到的变换矩阵更新全局变换，用于将后续的点云都变换到与
//第一个输入点云同一坐标系下。
//--------------------------------------------------------------------------------
int alignPoint(int argc1, const vector<string>& argv1)
{
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;// 存储管理所有打开的点云
	loadData(argc1, argv1, data);                           // 加载所有点云文件到data

	if (data.empty())                                     // 检查输入的点云是否为空
	{
		PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv1[0]);
		PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
		return (-1);
	}
	PCL_INFO("Loaded %d datasets.", (int)data.size());

	//创建一个PCL可视化对象
	//pVisualizer = new pcl::visualization::PCLVisualizer("Pairwise Incremental Registration example");
	//pVisualizer->createViewPort(0.0, 0, 0.5, 1.0, vp_1);          // 用左半窗口创建视口vp_1
	//pVisualizer->createViewPort(0.5, 0, 1.0, 1.0, vp_2);          // 用右半窗口创建视口vp_2
	PointCloud::Ptr result(new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
	for (size_t i = 1; i < data.size(); ++i)            // 循环处理所有点云
	{
		source = data[i - 1].cloud;                     // 连续配准
		target = data[i].cloud;                         // 相邻两组点云
		//添加可视化数据

		//showCloudsLeft(source, target);                 // 可视化为配准的源和目标点云

		PointCloud::Ptr temp(new PointCloud);
		PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
		//----------------------------------------------------------------------------
		//调用子函数完成一组点云的配准，temp返回配准后两组点云在第一组点云坐标下的点云
		//pairTransform返回从目标点云target到source的变换矩阵
		//----------------------------------------------------------------------------
		pairAlign3(source, target, temp, pairTransform, true);
		pairAlign6(source, target, temp, pairTransform, true);
		//把当前的两两准后的点云temp转化到全局坐标系下返回result
		pcl::transformPointCloud(*temp, *result, GlobalTransform);
		//用当前的两组点云之间的变换更新全局变换
		GlobalTransform = pairTransform * GlobalTransform;
		//保存转换到第一个点云坐标下的当前配准后的两组点云result到文件i.pcd
		stringstream ss;
		ss << "../data/" << i << ".pcd";
		pcl::io::savePCDFile(ss.str(), *result);
	}
}


int icp(const string& file1, const string& file2)
{
	pcl::console::TicToc time;
	// --------------------加载源点云-----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(file1, *source);//辅助点云

	cout << "从源点云中读取 " << source->size() << " 个点" << endl;

	// -------------------加载目标点云----------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(file2, *target);//主点云

	cout << "从目标点云中读取 " << target->size() << " 个点" << endl;

	time.tic();
	//--------------------初始化ICP对象--------------------
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//----------------------icp核心代码--------------------
	icp.setInputSource(source);            // 源点云
	icp.setInputTarget(target);            // 目标点云
	icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(1);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	icp.setMaximumIterations(35);           // 最大迭代次数
	icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*icp_cloud);
	cout << "Applied " << 35 << " ICP iterations in " << time.toc() << " ms" << endl;
	cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
	cout << "变换矩阵：\n" << icp.getFinalTransformation() << endl;



	// 使用创建的变换对为输入源点云进行变换
	pcl::transformPointCloud(*source, *icp_cloud, icp.getFinalTransformation());
	//pcl::io::savePCDFileASCII ("666.pcd", *icp_cloud);



	return 0;
}


//int main()
//{
//	icp((string)"sample2.pcd", (string)"sample20.pcd");
//	return 0;
//}

