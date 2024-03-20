#include"CloudFusion.h"
#include <pcl/registration/ia_fpcs.h>



using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointXYZ point;

void CF::PCS(int i)
{
	pointcloud::Ptr source_cloud(new pointcloud);
	pointcloud::Ptr target_cloud(new pointcloud);

	string fileName1, fileName2, fileName3;
	fileName3 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(i) + ".ply";
	fileName1 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(i) + ".ply";
	fileName2 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(i + 1) + ".ply";

	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointnormal::Ptr normals(new pointnormal);
	pointnormal::Ptr normals1(new pointnormal);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;


	pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *source_cloud);
	pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *target_cloud);
	//---------------------------ȥ��Դ���Ƶ�NAN��------------------------
	vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
	cout << "remove *source_cloud nan" << endl;

	vector<int> indices_tgt; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
	cout << "remove *target_cloud nan" << endl;


	n.setInputCloud(source_cloud);
	n.setNumberOfThreads(8);//����openMP���߳���
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normals);

	// ��������ռ������ģ�壩�����

	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
	const int kBinNum = 16;
	nss.setBins(kBinNum, kBinNum, kBinNum);
	// ����������������ƣ��˴����Գ�������Ϊtrue
	nss.setKeepOrganized(false);
	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
	nss.setSeed(0);   // random seed
	// ����������ĵ�������
	nss.setInputCloud(source_cloud);
	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
	nss.setNormals(normals);
	// ���ò�����������Ŀ����Ƶ���������
	const float kSampleRatio = 0.1f;
	nss.setSample(source_cloud->size() * kSampleRatio);
	// ִ�в����������������
	nss.filter(*source);

	n1.setInputCloud(target_cloud);
	n1.setNumberOfThreads(8);//����openMP���߳���
	n1.setSearchMethod(tree);
	n1.setKSearch(15);
	n1.compute(*normals1);
	// ��������ռ������ģ�壩�����
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss1;
	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���

	nss1.setBins(kBinNum, kBinNum, kBinNum);
	// ����������������ƣ��˴����Գ�������Ϊtrue
	nss1.setKeepOrganized(false);
	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
	nss1.setSeed(0);   // random seed
	// ����������ĵ�������
	nss1.setInputCloud(target_cloud);
	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
	nss1.setNormals(normals1);
	// ���ò�����������Ŀ����Ƶ���������

	nss1.setSample(target_cloud->size() * kSampleRatio);
	// ִ�в����������������
	nss1.filter(*target);

	//�ĵ㷨��׼
	pointcloud::Ptr pcs(new pointcloud);
	pcl::registration::FPCSInitialAlignment<pcl::PointXYZ, pcl::PointXYZ> fpcs;
	fpcs.setInputSource(source);//�������׼����
	fpcs.setInputTarget(target);//����Ŀ�����

	//��������
	fpcs.setApproxOverlap(0.1);//�������ص���
	fpcs.setDelta(0.1);//Bunny
	//fpcs.setDelta(0.5);//hippo
	fpcs.setMaxComputationTime(50);
	fpcs.setNumberOfSamples(int(source->size()));
	Eigen::Matrix4f tras;
	clock_t start = clock();
	fpcs.align(*pcs);
	clock_t end = clock();
	cout << "time:" << (double)(end - start) / (double)CLOCKS_PER_SEC << endl;
	cout << "score:" << fpcs.getFitnessScore() << endl;
	tras = fpcs.getFinalTransformation();
	cout << "matrix:" << endl << tras << endl << endl << endl;


	//PointCloud::Ptr cloud_end(new PointCloud);
	pcl::transformPointCloud(*source_cloud, *source_cloud, tras);
	//visualize_pcd(cloud_source, cloud_target, cloud_end);

	//----------------------------------icp----------------------------------------
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);
	icp.setTransformationEpsilon(1e-4);   // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(5);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(0.01);  // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(1000);           // ����������
	// ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����,
	// Ӧ�� ICP �㷨��, IterativeClosestPoint �ܹ����������Ƽ�,�������������ƥ����ȷ�Ļ�
	// ������������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ��ƣ�,��ô icp. hasConverged()= 1 (true),
	// Ȼ���������ձ任�����ƥ������ͱ任�������Ϣ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*Final);
	*Final += *target_cloud;
	pcl::transformPointCloud(*source_cloud, *source_cloud, icp.getFinalTransformation());
	//*source_cloud += *target_cloud;
	//pcl::io::savePLYFileASCII(fileName3, *source_cloud);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
	std::cout << matrix << std::endl;
	pcl::io::savePLYFileASCII(fileName3, *source_cloud);
}

void CF::sacia(int i)
{
	pointcloud::Ptr source_cloud(new pointcloud);
	pointcloud::Ptr target_cloud(new pointcloud);
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointnormal::Ptr normals(new pointnormal);
	pointnormal::Ptr normalss(new pointnormal);
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pointnormal::Ptr normals1(new pointnormal);
	pointnormal::Ptr normalss1(new pointnormal);
	fpfhFeature::Ptr fpfh1(new fpfhFeature);
	pointcloud::Ptr align(new pointcloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nn;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nn1;
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;




	clock_t start, end, time;
	start = clock();


	string fileName1, fileName2, fileName3;
	fileName3 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ��" + to_string(i + 1) + ".ply";
	fileName1 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ��" + to_string(i) + ".ply";
	fileName2 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\�ں�ǰ��" + to_string(i + 1) + ".ply";




	pcl::io::loadPLYFile<pcl::PointXYZ>(fileName2, *source_cloud);
	pcl::io::loadPLYFile<pcl::PointXYZ>(fileName1, *target_cloud);
	//---------------------------ȥ��Դ���Ƶ�NAN��------------------------
	vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
	cout << "remove *source_cloud nan" << endl;
	//-------------------------Դ�����²����˲�-------------------------
	//pcl::VoxelGrid<pcl::PointXYZ> vs;
	/*vs.setLeafSize(2, 2, 2);
	vs.setInputCloud(source_cloud);

	vs.filter(*source);
	cout << "down size *source_cloud from " << source_cloud->size() << " to " << source->size() << endl;*/
	n.setInputCloud(source_cloud);
	n.setNumberOfThreads(8);//����openMP���߳���
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normals);

	// ��������ռ������ģ�壩�����
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
	const int kBinNum = 16;
	nss.setBins(kBinNum, kBinNum, kBinNum);
	// ����������������ƣ��˴����Գ�������Ϊtrue
	nss.setKeepOrganized(false);
	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
	nss.setSeed(0);   // random seed
	// ����������ĵ�������
	nss.setInputCloud(source_cloud);
	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
	nss.setNormals(normals);
	// ���ò�����������Ŀ����Ƶ���������
	const float kSampleRatio = 0.1f;
	nss.setSample(source_cloud->size() * kSampleRatio);
	// ִ�в����������������
	nss.filter(*source);
	//pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����MLS��1.ply", *source);





	////--------------------------ȥ��Ŀ����Ƶ�NAN��--------------------
	vector<int> indices_tgt; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices_tgt);
	cout << "remove *target_cloud nan" << endl;
	//----------------------Ŀ������²����˲�-------------------------
	//pcl::VoxelGrid<pcl::PointXYZ> vt;
	/*vt.setLeafSize(2, 2, 2);
	vt.setInputCloud(target_cloud);

	vt.filter(*target);
	pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *target);
	cout << "down size *target_cloud from " << target_cloud->size() << " to " << target->size() << endl;*/
	//pcl::RandomSample<pcl::PointXYZ> vs1;    //�����˲�������
	//vs1.setInputCloud(target_cloud);                //���ô��˲�����
	//vs1.setSample(32000);                    //�����²������Ƶĵ���
	////vs.setSeed(1);                        //��������������ӵ�
	//vs1.filter(*target);                    //ִ���²����˲��������˲������cloud_sub
	//pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\������.ply", *target);

	n1.setInputCloud(target_cloud);
	n1.setNumberOfThreads(8);//����openMP���߳���
	n1.setSearchMethod(tree);
	n1.setKSearch(15);
	n1.compute(*normals1);
	// ��������ռ������ģ�壩�����
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss1;
	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���

	nss1.setBins(kBinNum, kBinNum, kBinNum);
	// ����������������ƣ��˴����Գ�������Ϊtrue
	nss1.setKeepOrganized(false);
	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
	nss1.setSeed(0);   // random seed
	// ����������ĵ�������
	nss1.setInputCloud(target_cloud);
	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
	nss1.setNormals(normals1);
	// ���ò�����������Ŀ����Ƶ���������

	nss1.setSample(target_cloud->size() * kSampleRatio);
	// ִ�в����������������
	nss1.filter(*target);
	//pcl::io::savePLYFileASCII("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����MLS��2.ply", *target);
	//---------------����Դ���ƺ�Ŀ����Ƶ�FPFH------------------------

	//fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source, tree);
	nn.setInputCloud(source);
	nn.setNumberOfThreads(8);//����openMP���߳���
	nn.setSearchMethod(tree);
	nn.setKSearch(15);
	nn.compute(*normalss);

	//------------------FPFH����-------------------------------


	f.setNumberOfThreads(8); //ָ��8�˼���
	f.setInputCloud(source);
	f.setInputNormals(normalss);
	f.setSearchMethod(tree);
	f.setKSearch(15);
	f.compute(*fpfh);

	//fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target, tree);
	nn1.setInputCloud(target);
	nn1.setNumberOfThreads(8);//����openMP���߳���
	nn1.setSearchMethod(tree);
	nn1.setKSearch(15);
	nn1.compute(*normalss1);

	//------------------FPFH����-------------------------------


	f1.setNumberOfThreads(8); //ָ��8�˼���
	f1.setInputCloud(target);
	f1.setInputNormals(normalss1);
	f1.setSearchMethod(tree);
	f1.setKSearch(15);
	f1.compute(*fpfh1);


	//--------------����һ����SAC_IA��ʼ��׼----------------------------
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source);
	sac_ia.setSourceFeatures(fpfh);
	sac_ia.setInputTarget(target);
	sac_ia.setTargetFeatures(fpfh1);
	sac_ia.setMinSampleDistance(3);//��������֮�����С����
	//setNumberOfSamples����3���͹������ö��˷����䲻׼��
	sac_ia.setNumberOfSamples(3);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
	//sac_ia.setCorrespondenceRandomness(5); //��ѡ�����������Ӧʱ������Ҫʹ�õ��ھӵ�����;
	//Ҳ���Ǽ���Э����ʱѡ��Ľ��ڵ��������ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
	//sac_ia.setErrorFunction();//��������ǿ�ѡ��
	sac_ia.setMaximumIterations(5000);
	sac_ia.align(*align);
	end = clock();
	pcl::transformPointCloud(*source_cloud, *source_cloud, sac_ia.getFinalTransformation());

	//pcl::io::savePLYFile("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\zhuaaa.ply", *align);
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
	cout << "\nSAC_IA has converged, score is " << sac_ia.getFitnessScore() << endl;
	cout << "�任����\n" << sac_ia.getFinalTransformation() << endl;
	//-------------------���ӻ�------------------------------------
	// visualize_pcd(source_cloud, target_cloud, align);






	//----------------------------------icp----------------------------------------
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);
	icp.setTransformationEpsilon(1e-4);   // Ϊ��ֹ����������Сת������
	icp.setMaxCorrespondenceDistance(5);  // ���ö�Ӧ���֮��������루��ֵ����׼���Ӱ��ϴ󣩡�
	icp.setEuclideanFitnessEpsilon(0.01);  // �������������Ǿ�������С����ֵ�� ֹͣ������
	icp.setMaximumIterations(1000);           // ����������
	// ����һ�� pcl::PointCloud<pcl::PointXYZ>ʵ�� Final ����,�洢��׼�任���Դ����,
	// Ӧ�� ICP �㷨��, IterativeClosestPoint �ܹ����������Ƽ�,�������������ƥ����ȷ�Ļ�
	// ������������һ��Ӧ��ĳ�ָ���任���Ϳ��Եõ�������ͬһ����ϵ����ͬ�ĵ��ƣ�,��ô icp. hasConverged()= 1 (true),
	// Ȼ���������ձ任�����ƥ������ͱ任�������Ϣ��

	icp.align(*Final);
	*Final += *target_cloud;
	pcl::transformPointCloud(*source_cloud, *source_cloud, icp.getFinalTransformation());
	//*source_cloud += *target_cloud;
	pcl::io::savePLYFileASCII(fileName3, *source_cloud);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4& matrix = icp.getFinalTransformation();
	std::cout << matrix << std::endl;

	//delete source_cloud;

}


void CF::getOverlappedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped_cloud2)
{
	double resolution = 7;
	pcl::octree::OctreePointCloudSearch<point> octree(resolution);
	octree.setInputCloud(cloud1);
	octree.addPointsFromInputCloud();

	//pcl::PointCloud<point> overlapped_2;
	for (size_t i = 0; i < cloud2->size(); ++i)
	{

		std::vector<int> indices;
		octree.voxelSearch(cloud2->points[i], indices);

		if (indices.size())
		{
			overlapped_cloud2->push_back(cloud2->points[i]);
		}

	}
}





void CF::calaPointCloudCoincide(int m, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float para1)
{



	pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr overlapped2(new pcl::PointCloud<pcl::PointXYZ>);

	getOverlappedCloud(cloud_src, cloud_target, overlapped2);


	//if (double(overlapped2->size())/ cloud_target->size() < 0.1) {
	//    return 0;
	//}
	getOverlappedCloud(cloud_target, cloud_src, overlapped1);

	//�ֱ���������Ƶ��ص����ֵĵ��Ʊ�������

	pcl::io::savePLYFile("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\overlapped" + to_string(m) + ".ply", *overlapped1);
	pcl::io::savePLYFile("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\overlapped" + to_string(n) + ".ply", *overlapped2);
	
}


void CF::find_corr(int m, int n) {

	for (int i = 1; i < 2; ++i) {
		clock_t s1 = clock();
		// --------------------����Դ����-----------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
		string fileName1 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(m) + ".ply";
		
		pcl::io::loadPLYFile(fileName1, *source);
		cout << "��Դ�����ж�ȡ " << source->size() << " ����" << endl;
		clock_t e1 = clock();
		whole_time += e1 - s1;
		for (int j = i + 1; j < 3; ++j) {

			clock_t s2 = clock();
			// -------------------����Ŀ�����----------------------
			pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
			string fileName2 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(n) + ".ply";
			
			pcl::io::loadPLYFile(fileName2, *target);
			cout << "��Ŀ������ж�ȡ " << target->size() << " ����" << endl;
			clock_t e2 = clock();
			whole_time += e2 - s2;
			calaPointCloudCoincide(m, n, source, target, 3);

		}
	}
}


//---------------------Kmeans-----------------------------
const float DIST_NEAR_ZERO = 0.1;

bool KMeans::SetInitKCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal>::Ptr nss)
{
	/*pointcloud::Ptr source_cloud(new pointcloud);
	pointcloud::Ptr target_cloud(new pointcloud);*/
	pointcloud::Ptr source(new pointcloud);
	//pointcloud::Ptr target(new pointcloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointnormal::Ptr normals(new pointnormal);
	//fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pointnormal::Ptr normals1(new pointnormal);
	//fpfhFeature::Ptr fpfh1(new fpfhFeature);
	//pointcloud::Ptr align(new pointcloud);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	//pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f;
	//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n1;
	//pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> f1;

	//---------------------------ȥ��Դ���Ƶ�NAN��------------------------
	vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices_src);
	cout << "remove *source_cloud nan" << endl;
	//-------------------------Դ�����²����˲�-------------------------

	n.setInputCloud(source_cloud);
	n.setNumberOfThreads(8);//����openMP���߳���
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normals);

	// ��������ռ������ģ�壩�����

	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
	const int kBinNum = 12;
	(*nss).setBins(kBinNum, kBinNum, kBinNum);
	// ����������������ƣ��˴����Գ�������Ϊtrue
	(*nss).setKeepOrganized(false);
	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
	(*nss).setSeed(0);   // random seed
	// ����������ĵ�������
	(*nss).setInputCloud(source_cloud);
	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
	(*nss).setNormals(normals);
	// ���ò�����������Ŀ����Ƶ���������
	const float kSampleRatio = 0.4f;
	(*nss).setSample(source_cloud->size() * kSampleRatio);
	// ִ�в����������������
	(*nss).filter(*source);
	cout << source->points.size() << endl;
	for (int i = 0; i < (*source).size(); ++i) {
		st_pointxyz p;
		p.x = source->points[i].x;
		p.y = source->points[i].y;
		p.z = source->points[i].z;
		pc_arr.push_back(p);

	}
	cout << pc_arr.size() << endl;
	return true;

}

bool KMeans::InitKCenter(std::vector< st_pointxyz> pnt_arr)
{
	if (m_k == 0)
	{
		PCL_ERROR("�ڴ�֮ǰ����Ҫ����setK()����\n");
		return false;
	}

	mv_center.resize(m_k);
	for (size_t i = 0; i < m_k; ++i)
	{
		mv_center[i] = pnt_arr[i];
	}
	return true;
}

bool KMeans::SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2)
{
	size_t pntCount1 = (size_t)pPntCloud1->points.size();
	//mv_pntcloud.resize(pntCount);
	for (size_t i = 0; i < pntCount1; ++i)
	{
		st_point point;
		point.pnt.x = pPntCloud1->points[i].x;
		point.pnt.y = pPntCloud1->points[i].y;
		point.pnt.z = pPntCloud1->points[i].z;
		point.groupID = 0;

		mv_pntcloud1.push_back(point);
	}


	size_t pntCount2 = (size_t)pPntCloud2->points.size();
	//mv_pntcloud.resize(pntCount);
	for (size_t i = 0; i < pntCount2; ++i)
	{
		st_point point;
		point.pnt.x = pPntCloud2->points[i].x;
		point.pnt.y = pPntCloud2->points[i].y;
		point.pnt.z = pPntCloud2->points[i].z;
		point.groupID = 0;

		mv_pntcloud2.push_back(point);
	}
	cout << "mv_pntcloud2.size()" << mv_pntcloud2.size() << endl;
	return true;
}

bool KMeans::Cluster()
{
	std::vector<st_pointxyz> v_center(mv_center.size());
	int k = 0;
	do
	{
		for (size_t i = 0, pntCount1 = mv_pntcloud1.size(); i < pntCount1; ++i)
		{
			double min_dist = DBL_MAX;
			int pnt_grp1 = 0;
			for (size_t j = 0; j < m_k; ++j)
			{
				double dist = DistBetweenPoints(mv_pntcloud1[i].pnt, mv_center[j]);
				if (min_dist - dist > 0.000001)
				{
					min_dist = dist;
					pnt_grp1 = j;
				}
			}
			m_grp_pntcloud1[pnt_grp1].push_back(st_point(mv_pntcloud1[i].pnt, pnt_grp1));
			m_grp_pntcloud[pnt_grp1].push_back(st_point(mv_pntcloud1[i].pnt, pnt_grp1));
		}

		for (size_t i = 0, pntCount2 = mv_pntcloud2.size(); i < pntCount2; ++i)
		{
			double min_dist = DBL_MAX;
			int pnt_grp2 = 0;
			for (size_t j = 0; j < m_k; ++j)
			{
				double dist = DistBetweenPoints(mv_pntcloud2[i].pnt, mv_center[j]);
				if (min_dist - dist > 0.000001)
				{
					min_dist = dist;
					pnt_grp2 = j;
				}
			}
			m_grp_pntcloud2[pnt_grp2].push_back(st_point(mv_pntcloud2[i].pnt, pnt_grp2));
			m_grp_pntcloud[pnt_grp2].push_back(st_point(mv_pntcloud2[i].pnt, pnt_grp2));
		}



		//������һ�ε��������ĵ�
		for (size_t i = 0; i < mv_center.size(); ++i)
		{
			v_center[i] = mv_center[i];
		}

		if (!UpdateGroupCenter(m_grp_pntcloud, mv_center))
		{
			return false;
		}
		if (!ExistCenterShift(v_center, mv_center))
		{
			break;
		}
		for (size_t i = 0; i < m_k; ++i) {
			m_grp_pntcloud[i].clear();
			m_grp_pntcloud1[i].clear();
			m_grp_pntcloud2[i].clear();
		}
		//cout << mv_center.size() << endl;


		///////////////////////////////////////////////////////////////////////////////////////////////////////
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr whole(new pcl::PointCloud<pcl::PointXYZ>());

		for (int i = 0; i < mv_center.size(); ++i) {
			pcl::PointXYZ p;
			p.x = mv_center[i].x;
			p.y = mv_center[i].y;
			p.z = mv_center[i].z;
			whole->points.push_back(p);
		}
		string newFileName0 = "C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\����"+to_string(k)+".ply";
		pcl::io::savePLYFile(newFileName0, *whole);
		++k;*/
		///////////////////////////////////////////////////////////////////////////////////////////////////////
	} while (true);

	return true;
}

double KMeans::DistBetweenPoints(st_pointxyz& p1, st_pointxyz& p2)
{
	double dist = 0;
	double x_diff = 0, y_diff = 0, z_diff = 0;

	x_diff = p1.x - p2.x;
	y_diff = p1.y - p2.y;
	z_diff = p1.z - p2.z;
	dist = sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

	return dist;
}

bool KMeans::UpdateGroupCenter(std::vector<VecPoint_t>& grp_pntcloud, std::vector<st_pointxyz>& center)
{
	if (center.size() != m_k)
	{
		PCL_ERROR("���ĸ�����ΪK\n");
		return false;
	}

	for (size_t i = 0; i < m_k; ++i)
	{
		float x = 0, y = 0, z = 0;
		size_t pnt_num_in_grp = grp_pntcloud[i].size();
		//������������û�е㣬��ô������ֱ�Ӽ�����һ������ȥ
		if (pnt_num_in_grp == 0) {
			continue;
		}
		for (size_t j = 0; j < pnt_num_in_grp; ++j)
		{
			x += grp_pntcloud[i][j].pnt.x;
			y += grp_pntcloud[i][j].pnt.y;
			z += grp_pntcloud[i][j].pnt.z;
		}
		x /= pnt_num_in_grp;
		y /= pnt_num_in_grp;
		z /= pnt_num_in_grp;
		center[i].x = x;
		center[i].y = y;
		center[i].z = z;
	}
	return true;
}

//�Ƿ�������ĵ��ƶ�
bool KMeans::ExistCenterShift(std::vector<st_pointxyz>& prev_center, std::vector<st_pointxyz>& cur_center)
{
	for (size_t i = 0; i < m_k; ++i)
	{
		double dist = DistBetweenPoints(prev_center[i], cur_center[i]);
		if (dist > DIST_NEAR_ZERO)
		{
			return true;
		}
	}

	return false;
}

//������ĵ�ֱ�浽���Ե�pcd�ļ���
//bool KMeans::SaveFile(const char* prex_name)
//{
//	for (size_t i = 0; i < m_k; ++i)
//	{
//		pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//
//		for (size_t j = 0, grp_pnt_count = m_grp_pntcloud[i].size(); j < grp_pnt_count; ++j)
//		{
//			pcl::PointXYZ pt;
//			pt.x = m_grp_pntcloud[i][j].pnt.x;
//			pt.y = m_grp_pntcloud[i][j].pnt.y;
//			pt.z = m_grp_pntcloud[i][j].pnt.z;
//
//			p_pnt_cloud->points.push_back(pt);
//		}
//
//		p_pnt_cloud->width = (int)m_grp_pntcloud[i].size();
//		p_pnt_cloud->height = 1;
//
//		/*char szFileName[256];
//		char newFileName[256] = { 0 };
//		char indexStr[16] = { 0 };*/
//		string newFileName;
//
//		/*strcat(newFileName, szFileName);
//		strcat(newFileName, "-");
//		strcat(newFileName, prex_name);
//		strcat(newFileName, "-");
//		sprintf(indexStr, "%d", i + 1);
//		strcat(newFileName, indexStr);
//		strcat(newFileName, ".pcd");*/
//
//		
//		pcl::io::savePCDFile(newFileName, *p_pnt_cloud);
//	}
//
//	return true;
//}

bool KMeans::SaveFile(const string dir_name, const string prex_name)
{
	//�����
	const int maxs = 200;
	//�������Ч��whole
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole(new pcl::PointCloud<pcl::PointXYZRGB>());


	for (size_t i = 0; i < m_k; ++i)
	{
		if (m_grp_pntcloud1[i].size() == 0 || m_grp_pntcloud2[i].size() == 0) {
			continue;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud2(new pcl::PointCloud<pcl::PointXYZ>());

		for (size_t j = 0, grp_pnt_count = m_grp_pntcloud1[i].size(); j < grp_pnt_count; ++j)
		{
			pcl::PointXYZ pt;
			pt.x = m_grp_pntcloud1[i][j].pnt.x;
			pt.y = m_grp_pntcloud1[i][j].pnt.y;
			pt.z = m_grp_pntcloud1[i][j].pnt.z;

			p_pnt_cloud1->points.push_back(pt);
		}

		p_pnt_cloud1->width = (int)m_grp_pntcloud1[i].size();
		p_pnt_cloud1->height = 1;

		for (size_t j = 0, grp_pnt_count = m_grp_pntcloud2[i].size(); j < grp_pnt_count; ++j)
		{
			pcl::PointXYZ pt;
			pt.x = m_grp_pntcloud2[i][j].pnt.x;
			pt.y = m_grp_pntcloud2[i][j].pnt.y;
			pt.z = m_grp_pntcloud2[i][j].pnt.z;

			p_pnt_cloud2->points.push_back(pt);
		}

		p_pnt_cloud2->width = (int)m_grp_pntcloud2[i].size();
		p_pnt_cloud2->height = 1;

		string newFileName1;

		newFileName1 = dir_name + "\\" + prex_name + "a" + "_" + to_string(i + 1) + ".ply";
		//*whole += *p_pnt_cloud1;
		//srand(time(NULL));
		int r = rand() % maxs;
		for (int i = 0; i < p_pnt_cloud1->size(); ++i) {
			pcl::PointXYZRGB p;
			p.x = p_pnt_cloud1->points[i].x;
			p.y = p_pnt_cloud1->points[i].y;
			p.z = p_pnt_cloud1->points[i].z;
			p.b = r * 2;
			p.g = r / 2;
			p.r = r;
			whole->points.push_back(p);
		}
		pcl::io::savePLYFile(newFileName1, *p_pnt_cloud1);
		string newFileName2;

		for (int i = 0; i < p_pnt_cloud2->size(); ++i) {
			pcl::PointXYZRGB p;
			p.x = p_pnt_cloud2->points[i].x;
			p.y = p_pnt_cloud2->points[i].y;
			p.z = p_pnt_cloud2->points[i].z;
			p.b = r * 2;
			p.g = r / 2;
			p.r = r;
			whole->points.push_back(p);
		}
		newFileName2 = dir_name + "\\" + prex_name + "b" + "_" + to_string(i + 1) + ".ply";
		pcl::io::savePLYFile(newFileName2, *p_pnt_cloud2);
	}

	/*for (size_t i = 0; i < m_k; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_pnt_cloud2(new pcl::PointCloud<pcl::PointXYZ>());

		for (size_t j = 0, grp_pnt_count = m_grp_pntcloud2[i].size(); j < grp_pnt_count; ++j)
		{
			pcl::PointXYZ pt;
			pt.x = m_grp_pntcloud2[i][j].pnt.x;
			pt.y = m_grp_pntcloud2[i][j].pnt.y;
			pt.z = m_grp_pntcloud2[i][j].pnt.z;

			p_pnt_cloud2->points.push_back(pt);
		}

		p_pnt_cloud2->width = (int)m_grp_pntcloud2[i].size();
		p_pnt_cloud2->height = 1;

		string newFileName2;

		newFileName2 = dir_name + "\\" + prex_name + "b" + "_" + to_string(i + 1) + ".pcd";
		pcl::io::savePCDFile(newFileName2, *p_pnt_cloud2);
	}*/
	string newFileName3 = dir_name + "\\" + prex_name + "w" + ".ply";
	pcl::io::savePLYFile(newFileName3, *whole);
	return true;
}

bool KMeans::Mix(const string dir_name, const string prex_name, int a, int b) {
	//�����ںϲ��ֵĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr whole(new pcl::PointCloud<pcl::PointXYZ>());

	/*for (int i = 0; i < mv_center.size(); ++i) {
		pcl::PointXYZ p;
		p.x = mv_center[i].x;
		p.y = mv_center[i].y;
		p.z = mv_center[i].z;
		whole->points.push_back(p);
	}
	string newFileName0 = dir_name + "\\" + prex_name + "rong" + ".ply";
	pcl::io::savePLYFile(newFileName0, *whole);*/

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudn(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pointnormal);

	//��������������
	pcl::PointCloud<pcl::PointXYZ>::Ptr centers1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr centers2(new pcl::PointCloud<pcl::PointXYZ>());

	for (int i = 0; i < m_k; ++i) {
		if (m_grp_pntcloud1[i].size() == 0 || m_grp_pntcloud2[i].size() == 0) {
			//centers1��centers2Ҳ����һ���㣬��֤���size��m_grp_pntcloud1��m_grp_pntcloud2һ��
			pcl::PointXYZ p1, p2;
			p1.x = 0; p1.y = 0; p1.z = 0;
			p2.x = 0; p2.y = 0; p2.z = 0;
			centers1->points.push_back(p1);
			centers2->points.push_back(p2);
			continue;
		}
		pcl::PointXYZ p1, p2;
		p1.x = 0; p1.y = 0; p1.z = 0;
		p2.x = 0; p2.y = 0; p2.z = 0;
		for (int j = 0; j < m_grp_pntcloud1[i].size(); ++j) {

			p1.x += m_grp_pntcloud1[i][j].pnt.x;
			p1.y += m_grp_pntcloud1[i][j].pnt.y;
			p1.z += m_grp_pntcloud1[i][j].pnt.z;

			//˳���m_grp_pntcloud1�����е�ÿ���㱣�浽cloudn���Ա�һ����㷨��
			pcl::PointXYZ np;
			np.x = m_grp_pntcloud1[i][j].pnt.x;
			np.y = m_grp_pntcloud1[i][j].pnt.y;
			np.z = m_grp_pntcloud1[i][j].pnt.z;
			cloudn->points.push_back(np);
		}
		p1.x /= (double)m_grp_pntcloud1[i].size();
		p1.y /= (double)m_grp_pntcloud1[i].size();
		p1.z /= (double)m_grp_pntcloud1[i].size();
		centers1->points.push_back(p1);

		for (int j = 0; j < m_grp_pntcloud2[i].size(); ++j) {

			p2.x += m_grp_pntcloud2[i][j].pnt.x;
			p2.y += m_grp_pntcloud2[i][j].pnt.y;
			p2.z += m_grp_pntcloud2[i][j].pnt.z;

		}
		p2.x /= (double)m_grp_pntcloud2[i].size();
		p2.y /= (double)m_grp_pntcloud2[i].size();
		p2.z /= (double)m_grp_pntcloud2[i].size();
		centers2->points.push_back(p2);


	}


	//���㷨��
	n.setInputCloud(cloudn);
	n.setNumberOfThreads(8);//����openMP���߳���
	n.setSearchMethod(tree);
	n.setKSearch(15);
	n.compute(*normal);





	//����������ÿ�����е������Ѿ��ҵ�������������������ļ����ںϵ�ʽ�Ӽ����ںϵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr mixed_points(new pcl::PointCloud<pcl::PointXYZ>());
	int k = 0;
	for (int i = 0; i < m_k; ++i) {
		if (m_grp_pntcloud1[i].size() == 0 || m_grp_pntcloud2[i].size() == 0) {
			continue;
		}
		//�������������ܶȺ���
		double w1 = 0, w2 = 0,sigma=5;

		for (int j = 0; j < m_grp_pntcloud1[i].size(); ++j) {
			double dist1 = sqrt(pow(centers1->points[i].x - m_grp_pntcloud1[i][j].pnt.x, 2) + pow(centers1->points[i].y - m_grp_pntcloud1[i][j].pnt.y, 2) + pow(centers1->points[i].z - m_grp_pntcloud1[i][j].pnt.z, 2));

			w1 += exp(-dist1 * dist1 / (2 * sigma * sigma));
		}
		for (int j = 0; j < m_grp_pntcloud2[i].size(); ++j) {
			double dist2 = sqrt(pow(centers2->points[i].x - m_grp_pntcloud2[i][j].pnt.x, 2) + pow(centers2->points[i].y - m_grp_pntcloud2[i][j].pnt.y, 2) + pow(centers2->points[i].z - m_grp_pntcloud2[i][j].pnt.z, 2));

			w2 += exp(-dist2 * dist2 / (2 * sigma * sigma));
		}
		//���ںϵ�λ��
		for (int j = 0; j < m_grp_pntcloud1[i].size(); ++j) {

			pcl::PointXYZ p;
			p.x = m_grp_pntcloud1[i][j].pnt.x;
			p.y = m_grp_pntcloud1[i][j].pnt.y;
			p.z = m_grp_pntcloud1[i][j].pnt.z;

			double dist_center12 = sqrt(pow(centers2->points[i].x - centers1->points[i].x, 2) + pow(centers2->points[i].y - centers1->points[i].y, 2) + pow(centers2->points[i].z - centers1->points[i].z, 2));
			pcl::PointXYZ p_mix;
			p_mix.x = p.x + w1 / (w1 + w2) * dist_center12 * (normal->points[k].normal_x);
			p_mix.y = p.y + w1 / (w1 + w2) * dist_center12 * (normal->points[k].normal_y);
			p_mix.z = p.z + w1 / (w1 + w2) * dist_center12 * (normal->points[k++].normal_z);
			mixed_points->points.push_back(p_mix);

		}
	}

	/*string newFileName0 = dir_name + "\\" + prex_name + "rong" + ".ply";
	pcl::io::savePLYFile(newFileName0, *mixed_points);*/

	//������ص�����ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	string fileName1 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(a) + ".ply";
	pcl::io::loadPLYFile(fileName1, *cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	string fileName2 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\�ں�ǰ��" + to_string(b) + ".ply";
	pcl::io::loadPLYFile(fileName2, *cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noverlapped_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr noverlapped_cloud1(new pcl::PointCloud<pcl::PointXYZ>());

	double resolution = 3;
	pcl::octree::OctreePointCloudSearch<point> octree(resolution);
	octree.setInputCloud(cloud1);
	octree.addPointsFromInputCloud();

	//pcl::PointCloud<point> overlapped_2;
	for (size_t i = 0; i < cloud2->size(); ++i)
	{

		std::vector<int> indices;
		octree.voxelSearch(cloud2->points[i], indices);

		if (!indices.size())
		{
			noverlapped_cloud2->push_back(cloud2->points[i]);
		}

	}





	pcl::octree::OctreePointCloudSearch<point> octree1(resolution);
	octree1.setInputCloud(cloud2);
	octree1.addPointsFromInputCloud();

	//pcl::PointCloud<point> overlapped_2;
	for (size_t i = 0; i < cloud1->size(); ++i)
	{

		std::vector<int> indices1;
		octree1.voxelSearch(cloud1->points[i], indices1);

		if (!indices1.size())
		{
			noverlapped_cloud1->push_back(cloud1->points[i]);
		}

	}

	*mixed_points += *noverlapped_cloud1 + *noverlapped_cloud2;
	string newFileName = dir_name + "\\" + prex_name + to_string(b) + ".ply";
	pcl::io::savePLYFile(newFileName, *mixed_points);

	return true;
}





bool KMeans::clearr() {
	m_k = 0;
	pc_arr.clear();
	mv_pntcloud1.clear();
	mv_pntcloud2.clear();
	m_grp_pntcloud1.clear();
	m_grp_pntcloud2.clear();
	m_grp_pntcloud.clear();
	mv_center.clear();

	return true;
}