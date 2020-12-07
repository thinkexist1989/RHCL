#include <conio.h>
#include<vector>
#include<iostream>
#include<string>
#include<ctime>
// io��ȡ��ͷ�ļ�
#include <pcl/PolygonMesh.h>
#include <pcl/io/io.h>
#include<pcl/io/obj_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFile����ͷ�ļ���
#include<pcl/console/parse.h>

//
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>	
#include <pcl/filters/random_sample.h> //�������õ���ͷ�ļ�
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
using namespace std;
// Mutex: //������
boost::mutex source_point_cloud_mutex;
boost::mutex target_point_cloud_mutex;

void printUsage(const char* progName)
{
	cout << "�����������������֣��ټ�һ����������0���ֶ���1��sac_icp,������Ĭ��Ϊ0" << endl;
	cout << "���ӣ�"<<progName<<" source_point_cloud.pcd/obj/ply target_point_cloud.pcd/obj/ply 1" << endl;
	cout << "���԰汾" << endl;
}

//��ȡ�����ļ�//
void read_pointcloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud, int argc, char **argv)
{
	clock_t read_pc_start = clock();
	string pc1name = argv[1];
	string pc2name = argv[2];
	string pc1ext = pc1name.substr(pc1name.size() - 3);
	string pc2ext = pc2name.substr(pc2name.size() - 3);
	pcl::PolygonMesh mesh1;
	pcl::PolygonMesh mesh2;
	if (pc1ext.compare("ply") == 0)
		pcl::io::loadPLYFile(argv[1], *source_point_cloud);
	if (pc1ext.compare("obj") == 0)
	{
		cout << "��ʼ��source_point_cloud" <<  endl;
		pcl::io::loadPolygonFile(argv[1], mesh1);
		cout << "����source_point_cloud��תpcl" << endl;
		pcl::fromPCLPointCloud2(mesh1.cloud, *source_point_cloud);
		cout << "������source_point_cloud" << endl;
	}
		//pcl::io::loadOBJFile(argv[1], *source_point_cloud); //���ֶ���ķ�ʽ�ر���
	if (pc1ext.compare("pcd") == 0)
		pcl::io::loadPCDFile(argv[1], *source_point_cloud);
	if (pc2ext.compare("ply") == 0)
		pcl::io::loadPLYFile(argv[2], *target_point_cloud);
	if (pc2ext.compare("obj") == 0)		
	{
		cout << "��ʼ��target_point_cloud" << endl;
		pcl::io::loadPolygonFile(argv[2], mesh2);
		cout << "����target_point_cloud��תpcl" << endl;
		pcl::fromPCLPointCloud2(mesh2.cloud, *target_point_cloud);
		cout << "������target_point_cloud" << endl;
	}
		//pcl::io::loadOBJFile(argv[2], *target_point_cloud); //���ֶ���ķ�ʽ�ر���
	if (pc2ext.compare("pcd") == 0)
		pcl::io::loadPCDFile(argv[2], *target_point_cloud);
	
	clock_t read_pc_end = clock();
	cout << "read pointcloud time: " << (double)(read_pc_end - read_pc_start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "source_point_cloud size : " << source_point_cloud->points.size() << endl;
	cout << "target_point_cloud size : " << target_point_cloud->points.size() << endl;
}

//���ڸ��ص������Ľṹ�嶨��
// structure used to pass arguments to the callback function
struct callback_args
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pairpointselect_callback(const pcl::visualization::PointPickingEvent& event, void* args) //ѡ�㣬���
{
	int n;
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	pcl::PointXYZRGBA current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z); //�õ���ǰ������
	//data->clicked_points_3d->clear();//���ϴ�ѡ�ĵ����
	data->clicked_points_3d->points.push_back(current_point);//�����ѡ��ĵ�
	n = data->clicked_points_3d->points.size();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red(data->clicked_points_3d, 255, 0, 0); //��ѡ�е��ú�ɫ���
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->addText3D(to_string(n), current_point,0.01,1.0,0.0,0.0,to_string(n));
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << current_point.x << " " << current_point.y << " " << current_point.z << endl;
	//cout << data->viewerPtr->window_name() << "select " << n << " corresponding points" << endl;
}

//��������ܶ�
void compute_pointcloud_density(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud, double &delta) {
	int aa = point_cloud->points.size();
	double qxmax = point_cloud->points[0].x;
	double qxmin = point_cloud->points[0].x;
	double qymax = point_cloud->points[0].y;
	double qymin = point_cloud->points[0].y;
	double qzmax = point_cloud->points[0].z;
	double qzmin = point_cloud->points[0].z;
	for (int i = 0; i < aa - 1; i++)
	{
		double qx = point_cloud->points[i].x;
		qxmax = max(qx, qxmax);
		qxmin = min(qx, qxmin);
		double qy = point_cloud->points[i].y;
		qymax = max(qy, qymax);
		qymin = min(qy, qymin);
		double qz = point_cloud->points[i].z;
		qzmax = max(qz, qzmax);
		qzmin = min(qz, qzmin);

	}
	double pointnumber = pow(aa, 1.0 / 3);
	delta = max(max((qxmax - qxmin) / pointnumber, (qymax - qymin) / pointnumber), (qzmax - qzmin) / pointnumber);
}

//������
void DownSample(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	pcl::RandomSample<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(cloud);
	sor.setSample(10000);
	sor.filter(*cloud);
}
void method_icp(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutCloud, Eigen::Matrix4f &registration_matrix)
{
	if (source_point_cloud->points.size() > 10000)
		DownSample(source_point_cloud);
	if (target_point_cloud->points.size() > 10000)
		DownSample(target_point_cloud);
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setInputSource(source_point_cloud);
	icp.setInputTarget(target_point_cloud);
	pcl::PointCloud<pcl::PointXYZRGBA> Final;
	icp.align(Final);
	cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;
	pcl::transformPointCloud(*source_point_cloud, *OutCloud, icp.getFinalTransformation());
	registration_matrix = icp.getFinalTransformation();
}

void method_sac_icp(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutCloud, Eigen::Matrix4f &registration_matrix)
{
	double sourcepoint_leafsize;
	compute_pointcloud_density(source_point_cloud, sourcepoint_leafsize);
	double targetpoint_leafsize;
	compute_pointcloud_density(target_point_cloud, targetpoint_leafsize);
	vector<int> indices_src; //����ȥ���ĵ������
	pcl::removeNaNFromPointCloud(*source_point_cloud, *source_point_cloud, indices_src);
	cout << "remove *source_point_cloud nan" << endl;
	//�²����˲�
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setLeafSize(sourcepoint_leafsize*2, sourcepoint_leafsize * 2, sourcepoint_leafsize * 2);
	voxel_grid.setInputCloud(source_point_cloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	cout << "down size *source_point_cloud from " << source_point_cloud->size() << "to" << cloud_src->size() << endl;
	//������淨��
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_src;
	ne_src.setInputCloud(cloud_src);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZRGBA>());
	ne_src.setSearchMethod(tree_src);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(sourcepoint_leafsize * 6);
	ne_src.compute(*cloud_src_normals);

	vector<int> indices_tgt;
	pcl::removeNaNFromPointCloud(*target_point_cloud, *target_point_cloud, indices_tgt);
	cout << "remove *target_point_cloud nan" << endl;

	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid_2;
	voxel_grid_2.setLeafSize(targetpoint_leafsize * 2, targetpoint_leafsize * 2, targetpoint_leafsize * 2);
	voxel_grid_2.setInputCloud(target_point_cloud);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGBA>);
	voxel_grid_2.filter(*cloud_tgt);
	cout << "down size *target_point_cloud.pcd from " << target_point_cloud->size() << "to" << cloud_tgt->size() << endl;

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(cloud_tgt);
	pcl::search::KdTree< pcl::PointXYZRGBA>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZRGBA>());
	ne_tgt.setSearchMethod(tree_tgt);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(targetpoint_leafsize * 6);
	ne_tgt.compute(*cloud_tgt_normals);

	//����FPFH
	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(cloud_src);
	fpfh_src.setInputNormals(cloud_src_normals);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_src_fpfh(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	fpfh_src.setSearchMethod(tree_src_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(sourcepoint_leafsize * 10);
	fpfh_src.compute(*fpfhs_src);
	cout << "compute *cloud_src fpfh" << endl;

	pcl::FPFHEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(cloud_tgt);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree_tgt_fpfh(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(targetpoint_leafsize * 10);
	fpfh_tgt.compute(*fpfhs_tgt);
	cout << "compute *cloud_tgt fpfh" << endl;

	//SAC��׼
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	scia.setNumberOfSamples(20);
	scia.setRANSACIterations(30);
	scia.setRANSACOutlierRejectionThreshold(targetpoint_leafsize * 10);
	//scia.setCorrespondenceRandomness(20);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sac_result(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	cout << sac_trans << endl;
	clock_t sac_time = clock();

	//icp��׼
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setMaxCorrespondenceDistance(0.04);
	// ����������
	icp.setMaximumIterations(50);
	// ���α仯����֮��Ĳ�ֵ
	icp.setTransformationEpsilon(1e-10);
	// �������
	icp.setEuclideanFitnessEpsilon(0.2);
	icp.setInputSource(sac_result);
	icp.setInputTarget(cloud_tgt);
	pcl::PointCloud<pcl::PointXYZRGBA> Final;
	icp.align(Final);
	cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;
	Eigen::Matrix4f icp_trans= icp.getFinalTransformation();
	pcl::transformPointCloud(*source_point_cloud, *OutCloud, icp_trans*sac_trans);
	registration_matrix = icp_trans * sac_trans;
}
//��׼  ����ط����Ի����ַ���
void registration(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutCloud, Eigen::Matrix4f &registration_matrix, int method)
{
	clock_t registration_start = clock();
	switch (method) {
	case 0: {
		cout << "choose icp method" << endl;
		method_icp(source_point_cloud, target_point_cloud, OutCloud,registration_matrix);
		break;
	}
	case 1: {
		cout << "choose sac_icp method" << endl;
		method_sac_icp(source_point_cloud, target_point_cloud, OutCloud,registration_matrix);
		break;
	}
	default: {
		cout << "no this method" << endl;
		break;
	}
	}
	clock_t registration_end = clock();
	cout << "registartiond time: " << (double)(registration_end - registration_start) / (double)CLOCKS_PER_SEC << " s" << endl;
}


int
 main (int argc, char** argv)
{
	if (argc < 3)
	{
		PCL_ERROR("��������������㣡\n");
		printUsage(argv[0]);
		return (0);
	}
  //������������������������������������������������������������������������������������������������������������	
  //�����������������
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // Դ����
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // Ŀ�����
  //������������������������������������������������������������������������������������������������������������	
  //��ȡ�������������
  read_pointcloud(source_point_cloud, target_point_cloud, argc, argv);
  //������������������������������������������������������������������������������������������������������������
  //������
  source_point_cloud_mutex.lock();
  target_point_cloud_mutex.lock();
  //������������������������������������������������������������������������������������������������������������
  //��ʾ����
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
  //source���ƴ���
  viewer1->setBackgroundColor(255, 255, 255);//���ñ���ɫΪ��ɫ
  viewer1->addText("source_point_cloud_image", 10, 10,1.0,0.0,0.0);
  viewer1->addPointCloud(source_point_cloud,"source_point_cloud");
  //viewer1->addCoordinateSystem(1.0); //����������
  //target���ƴ���
  viewer2->setBackgroundColor(255, 255, 255);//���ñ���ɫΪ��ɫ
  viewer2->addText("target_point_cloud_image", 10, 10, 1.0, 0.0, 0.0);
  viewer2->addPointCloud(target_point_cloud, "target_point_cloud");
  //viewer2->addCoordinateSystem(1.0);

  //������������������������������������������������������������������������������������������������������������
  // ѡ��
  struct callback_args cb_args1;
  struct callback_args cb_args2;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d1(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cb_args1.clicked_points_3d = clicked_points_3d1;
  cb_args2.clicked_points_3d = clicked_points_3d2;
  cb_args1.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer1);
  cb_args2.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer2);
  viewer1->registerPointPickingCallback(pairpointselect_callback, (void*)&cb_args1);  //ע����Ļѡ���¼�
  viewer2->registerPointPickingCallback(pairpointselect_callback, (void*)&cb_args2);  //ע����Ļѡ���¼�
  //Shfit+������ѡ���
  cout << "Shift+click on four corresponding points,then press 'Enter'..." << endl;//Shfit+������ѡ���,����Enter��������
  //������������������������������������������������������������������������������������������������������������
  //�ͷŻ�����
  source_point_cloud_mutex.unlock();
  target_point_cloud_mutex.unlock();
  //������������������������������������������������������������������������������������������������������������
  //����������
  int ch = 0;//�����������ֵ
  while (!viewer1->wasStopped())
  {
	  viewer1->spinOnce(100);   //100??
	  viewer2->spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	  if (_kbhit()) {//����а������£���_kbhit()����������
		  ch = _getch();//ʹ��_getch()������ȡ���µļ�ֵ
	  }
	  if (ch == 13) { break; }//������Enterʱֹͣѭ����Enter���ļ�Ϊ13.
  }
  ch = 0;//�����������ֵ
  //������������������������������������������������������������������������������������������������������������
  //���ö�Ӧ���������任
  cout << "source_point_cloud select " << cb_args1.clicked_points_3d->points.size() << " corresponding points" << endl;
  cout << "target_point_cloud select " << cb_args2.clicked_points_3d->points.size() << " corresponding points" << endl;
  int correspond_point_number = 4;
  //����SVD�������任����  
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> TESVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Matrix4 correspond_transformation;
  TESVD.estimateRigidTransformation(*cb_args1.clicked_points_3d, *cb_args2.clicked_points_3d, correspond_transformation);
  //����任������Ϣ  
  cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << endl;
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", correspond_transformation(0, 0), correspond_transformation(0, 1), correspond_transformation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", correspond_transformation(1, 0), correspond_transformation(1, 1), correspond_transformation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", correspond_transformation(2, 0), correspond_transformation(2, 1), correspond_transformation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", correspond_transformation(0, 3), correspond_transformation(1, 3), correspond_transformation(2, 3));
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*source_point_cloud, *trans_source_point_cloud, correspond_transformation);
  //��ʾ���ݶ�Ӧ��任��ĵ���
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("��Ӧ�������׼ǰ����ӻ�"));
  int v1(0);
  int v2(0);
  viewer3->createViewPort(0.0, 0.0, 0.5, 1.0, v1);//(Xmin,Ymin,Xmax,Ymax)���ò�ͬ�ӽǴ�������
  viewer3->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer3->setBackgroundColor(255, 255, 255, v1);//���ñ���ɫΪ��ɫ
  viewer3->setBackgroundColor(255, 255, 255, v2);//���ñ���ɫΪ��ɫ
  viewer3->addText("correspond_transform_cloud_image", 10, 10,1.0,0.0,0.0, "v1 text",v1);
  viewer3->addText("final_transform_cloud_image", 10, 10, 1.0, 0.0, 0.0, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>  trans_source_point_cloud_handler(trans_source_point_cloud, 255, 255, 0);//��ɫ
  viewer3->addPointCloud(trans_source_point_cloud, trans_source_point_cloud_handler, "trans_source_point_cloud",v1);
  viewer3->addPointCloud(target_point_cloud,  "target_point_cloud", v1); //����ԭ��ɫ��ʾ����
  //������������������������������������������������������������������������������������������������������������
  //��������������ʾת�ƺ�����Ӿ���Ϳ��Խ��ע��
  /*
  cout << "press 'Enter'to start registration" << endl;
  //������������������������������������������������������������������������������������������������������������
  //����������
  while (!viewer3->wasStopped())
  {
	  viewer3->spinOnce(100);   //100??
	  if (_kbhit()) {//����а������£���_kbhit()����������
		  ch = _getch();//ʹ��_getch()������ȡ���µļ�ֵ
	  }
	  if (ch == 13) { break; }//������Enterʱֹͣѭ����Enter���ļ�Ϊ13.
  }  
  */
  //������������������������������������������������������������������������������������������������������������
  //��׼
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutCloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // Ŀ�����
  int method = 0;
  if (argc == 4) {
	  string methodname = argv[3];
	  method = stoi(methodname);
  }
  Eigen::Matrix4f registration_matrix;
  registration(trans_source_point_cloud, target_point_cloud, OutCloud,registration_matrix,method);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_registered_source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // �����任���source����
  pcl::transformPointCloud(*source_point_cloud, *trans_registered_source_point_cloud, registration_matrix*correspond_transformation);//pcl::transformPointCloud���ҳˣ����������еľ���任д���Ҳ�
  //������������������������������������������������������������������������������������������������������������
  //����׼��ĵ��Ƽ�����ʾ
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>  Final_handler(OutCloud, 255, 255, 0);//��ɫ
  viewer3->addPointCloud(trans_registered_source_point_cloud, Final_handler, "trans_registered_source_point_cloud1", v2); //����ָ����ɫ��ʾ�任���source����
  viewer3->addPointCloud(target_point_cloud, "target_point_cloud1", v2); //����ԭ��ɫ��ʾtarget����
  while (!viewer3->wasStopped())
  {
	  viewer3->spinOnce(100);   //
  }
 return (0);
}
