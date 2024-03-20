#include"CloudFusion.h"

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointXYZ point;


void Cloud_Reg_Fus()
{
	CF cf;
	KMeans km;
	pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
	pointcloud::Ptr source(new pointcloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointnormal::Ptr normals(new pointnormal);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
	//�����forѭ����һ����������
	for (int i = 1; i < 6; ++i) {
		//cf.sacia(i);
		cf.PCS(i);
		cf.find_corr(i, i + 1);


		//---------------------------------------------���ղ����ɵ��ص�����-------------------------------------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud1(new pcl::PointCloud<pcl::PointXYZ>);
		string fileName1 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\overlapped" + to_string(i) + ".ply";
		pcl::io::loadPLYFile(fileName1, *pPntCloud1);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud2(new pcl::PointCloud<pcl::PointXYZ>);
		string fileName2 = "D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������\\overlapped" + to_string(i+1) + ".ply";
		pcl::io::loadPLYFile(fileName2, *pPntCloud2);



		//---------------------------------------------KMeans------------------------------------------------------------

		km.SetInputCloud(pPntCloud1, pPntCloud2);

		//---------------------------ȥ��Դ���Ƶ�NAN��------------------------
		vector<int> indices_src; //����ȥ���ĵ������
		pcl::removeNaNFromPointCloud(*pPntCloud1, *pPntCloud1, indices_src);
		cout << "remove *source_cloud nan" << endl;
		//-------------------------Դ�����²����˲�-------------------------

		n.setInputCloud(pPntCloud1);
		n.setNumberOfThreads(8);//����openMP���߳���
		n.setSearchMethod(tree);
		n.setKSearch(15);
		n.compute(*normals);

		//�ñ������Ľ�������ĵ���Ϊ�����ʼ�㣬��֤�˳�ʼλ�úã�Ҳ��֤�˱�������
		// ��������ռ������ģ�壩�����

		// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
		const int kBinNum = 12;
		nss.setBins(kBinNum, kBinNum, kBinNum);
		// ����������������ƣ��˴����Գ�������Ϊtrue
		nss.setKeepOrganized(false);
		// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
		nss.setSeed(0);   // random seed
		// ����������ĵ�������
		nss.setInputCloud(pPntCloud1);
		// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
		nss.setNormals(normals);
		// ���ò�����������Ŀ����Ƶ���������
		const float kSampleRatio = 0.5f;
		nss.setSample(pPntCloud1->size() * kSampleRatio);
		// ִ�в����������������
		nss.filter(*source);
		cout << source->points.size() << endl;
		for (int i = 0; i < (*source).size(); ++i) {
			st_pointxyz p;
			p.x = source->points[i].x;
			p.y = source->points[i].y;
			p.z = source->points[i].z;
			km.pc_arr.push_back(p);

		}
		//km.SetInitKCenter(pPntCloud1, pPntCloud2,nss);
		clock_t start = clock();
		km.SetK(km.pc_arr.size());
		km.InitKCenter(km.pc_arr);
		km.Cluster();
		//km.SaveFile("C:\\Users\\123456\\Desktop\\���Թ���\\�߼���\\����1��׼\\kmeans","����1");
		km.Mix("D:\\XTOP\\XTOPwork\\mesh\\meshview\\Surface-Mesh-Framework-main\\��������", "�ں�ǰ��", i, i + 1);
		clock_t end = clock();
		std::cout << "time = " << double(end - start) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����
		//cout << km.pc_arr.size() << endl;
		//std::cout << "time = " << double(cf.whole_time) / CLOCKS_PER_SEC << "s" << std::endl;  //���ʱ�䣨��λ����
	   // cout << "h" << endl;
		km.clearr();

	}

}