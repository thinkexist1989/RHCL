#include <conio.h>
#include<vector>
#include<iostream>
#include<string>
#include<ctime>
// io读取的头文件
#include <pcl/PolygonMesh.h>
#include <pcl/io/io.h>
#include<pcl/io/obj_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFile所属头文件；
#include<pcl/console/parse.h>

//
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>	
#include <pcl/filters/random_sample.h> //降采样用到的头文件
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
using namespace std;
// Mutex: //进程锁
boost::mutex source_point_cloud_mutex;
boost::mutex target_point_cloud_mutex;

void printUsage(const char* progName)
{
	cout << "先输入两个点云名字，再加一个方法数字0是手动，1是sac_icp,不输入默认为0" << endl;
	cout << "例子："<<progName<<" source_point_cloud.pcd/obj/ply target_point_cloud.pcd/obj/ply 1" << endl;
	cout << "测试版本" << endl;
}

//读取点云文件//
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
		cout << "开始读source_point_cloud" <<  endl;
		pcl::io::loadPolygonFile(argv[1], mesh1);
		cout << "读完source_point_cloud，转pcl" << endl;
		pcl::fromPCLPointCloud2(mesh1.cloud, *source_point_cloud);
		cout << "处理完source_point_cloud" << endl;
	}
		//pcl::io::loadOBJFile(argv[1], *source_point_cloud); //这种读入的方式特别慢
	if (pc1ext.compare("pcd") == 0)
		pcl::io::loadPCDFile(argv[1], *source_point_cloud);
	if (pc2ext.compare("ply") == 0)
		pcl::io::loadPLYFile(argv[2], *target_point_cloud);
	if (pc2ext.compare("obj") == 0)		
	{
		cout << "开始读target_point_cloud" << endl;
		pcl::io::loadPolygonFile(argv[2], mesh2);
		cout << "读完target_point_cloud，转pcl" << endl;
		pcl::fromPCLPointCloud2(mesh2.cloud, *target_point_cloud);
		cout << "处理完target_point_cloud" << endl;
	}
		//pcl::io::loadOBJFile(argv[2], *target_point_cloud); //这种读入的方式特别慢
	if (pc2ext.compare("pcd") == 0)
		pcl::io::loadPCDFile(argv[2], *target_point_cloud);
	
	clock_t read_pc_end = clock();
	cout << "read pointcloud time: " << (double)(read_pc_end - read_pc_start) / (double)CLOCKS_PER_SEC << " s" << endl;
	cout << "source_point_cloud size : " << source_point_cloud->points.size() << endl;
	cout << "target_point_cloud size : " << target_point_cloud->points.size() << endl;
}

//用于给回调函数的结构体定义
// structure used to pass arguments to the callback function
struct callback_args
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void pairpointselect_callback(const pcl::visualization::PointPickingEvent& event, void* args) //选点，标红
{
	int n;
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	pcl::PointXYZRGBA current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z); //得到当前点坐标
	//data->clicked_points_3d->clear();//将上次选的点清空
	data->clicked_points_3d->points.push_back(current_point);//添加新选择的点
	n = data->clicked_points_3d->points.size();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red(data->clicked_points_3d, 255, 0, 0); //将选中点用红色标记
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->addText3D(to_string(n), current_point,0.01,1.0,0.0,0.0,to_string(n));
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	cout << current_point.x << " " << current_point.y << " " << current_point.z << endl;
	//cout << data->viewerPtr->window_name() << "select " << n << " corresponding points" << endl;
}

//计算点云密度
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

//降采样
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
	vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*source_point_cloud, *source_point_cloud, indices_src);
	cout << "remove *source_point_cloud nan" << endl;
	//下采样滤波
	pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
	voxel_grid.setLeafSize(sourcepoint_leafsize*2, sourcepoint_leafsize * 2, sourcepoint_leafsize * 2);
	voxel_grid.setInputCloud(source_point_cloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//pcl::PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_src);
	cout << "down size *source_point_cloud from " << source_point_cloud->size() << "to" << cloud_src->size() << endl;
	//计算表面法线
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

	//计算FPFH
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

	//SAC配准
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

	//icp配准
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
	icp.setMaxCorrespondenceDistance(0.04);
	// 最大迭代次数
	icp.setMaximumIterations(50);
	// 两次变化矩阵之间的差值
	icp.setTransformationEpsilon(1e-10);
	// 均方误差
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
//配准  这个地方可以换多种方法
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
		PCL_ERROR("输入变量数量不足！\n");
		printUsage(argv[0]);
		return (0);
	}
  //——————————————————————————————————————————————————————	
  //定义输入的两个点云
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // 源点云
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // 目标点云
  //——————————————————————————————————————————————————————	
  //读取输入的两个点云
  read_pointcloud(source_point_cloud, target_point_cloud, argc, argv);
  //——————————————————————————————————————————————————————
  //进程锁
  source_point_cloud_mutex.lock();
  target_point_cloud_mutex.lock();
  //——————————————————————————————————————————————————————
  //显示点云
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("viewer1"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
  //source点云窗口
  viewer1->setBackgroundColor(255, 255, 255);//设置背景色为白色
  viewer1->addText("source_point_cloud_image", 10, 10,1.0,0.0,0.0);
  viewer1->addPointCloud(source_point_cloud,"source_point_cloud");
  //viewer1->addCoordinateSystem(1.0); //加入坐标轴
  //target点云窗口
  viewer2->setBackgroundColor(255, 255, 255);//设置背景色为白色
  viewer2->addText("target_point_cloud_image", 10, 10, 1.0, 0.0, 0.0);
  viewer2->addPointCloud(target_point_cloud, "target_point_cloud");
  //viewer2->addCoordinateSystem(1.0);

  //——————————————————————————————————————————————————————
  // 选点
  struct callback_args cb_args1;
  struct callback_args cb_args2;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d1(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clicked_points_3d2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  cb_args1.clicked_points_3d = clicked_points_3d1;
  cb_args2.clicked_points_3d = clicked_points_3d2;
  cb_args1.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer1);
  cb_args2.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer2);
  viewer1->registerPointPickingCallback(pairpointselect_callback, (void*)&cb_args1);  //注册屏幕选点事件
  viewer2->registerPointPickingCallback(pairpointselect_callback, (void*)&cb_args2);  //注册屏幕选点事件
  //Shfit+鼠标左键选择点
  cout << "Shift+click on four corresponding points,then press 'Enter'..." << endl;//Shfit+鼠标左键选择点,按“Enter”键结束
  //——————————————————————————————————————————————————————
  //释放互斥体
  source_point_cloud_mutex.unlock();
  target_point_cloud_mutex.unlock();
  //——————————————————————————————————————————————————————
  //检测键盘输入
  int ch = 0;//检测键盘输入键值
  while (!viewer1->wasStopped())
  {
	  viewer1->spinOnce(100);   //100??
	  viewer2->spinOnce(100);
	  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	  if (_kbhit()) {//如果有按键按下，则_kbhit()函数返回真
		  ch = _getch();//使用_getch()函数获取按下的键值
	  }
	  if (ch == 13) { break; }//当按下Enter时停止循环，Enter键的键为13.
  }
  ch = 0;//检测键盘输入键值
  //——————————————————————————————————————————————————————
  //利用对应点进行坐标变换
  cout << "source_point_cloud select " << cb_args1.clicked_points_3d->points.size() << " corresponding points" << endl;
  cout << "target_point_cloud select " << cb_args2.clicked_points_3d->points.size() << " corresponding points" << endl;
  int correspond_point_number = 4;
  //利用SVD方法求解变换矩阵  
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA> TESVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Matrix4 correspond_transformation;
  TESVD.estimateRigidTransformation(*cb_args1.clicked_points_3d, *cb_args2.clicked_points_3d, correspond_transformation);
  //输出变换矩阵信息  
  cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << endl;
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", correspond_transformation(0, 0), correspond_transformation(0, 1), correspond_transformation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", correspond_transformation(1, 0), correspond_transformation(1, 1), correspond_transformation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", correspond_transformation(2, 0), correspond_transformation(2, 1), correspond_transformation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", correspond_transformation(0, 3), correspond_transformation(1, 3), correspond_transformation(2, 3));
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::transformPointCloud(*source_point_cloud, *trans_source_point_cloud, correspond_transformation);
  //显示根据对应点变换后的点云
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("对应后点云配准前后可视化"));
  int v1(0);
  int v2(0);
  viewer3->createViewPort(0.0, 0.0, 0.5, 1.0, v1);//(Xmin,Ymin,Xmax,Ymax)设置不同视角窗口坐标
  viewer3->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer3->setBackgroundColor(255, 255, 255, v1);//设置背景色为白色
  viewer3->setBackgroundColor(255, 255, 255, v2);//设置背景色为白色
  viewer3->addText("correspond_transform_cloud_image", 10, 10,1.0,0.0,0.0, "v1 text",v1);
  viewer3->addText("final_transform_cloud_image", 10, 10, 1.0, 0.0, 0.0, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>  trans_source_point_cloud_handler(trans_source_point_cloud, 255, 255, 0);//黄色
  viewer3->addPointCloud(trans_source_point_cloud, trans_source_point_cloud_handler, "trans_source_point_cloud",v1);
  viewer3->addPointCloud(target_point_cloud,  "target_point_cloud", v1); //按照原颜色显示点云
  //——————————————————————————————————————————————————————
  //这块如果打算先显示转移后的兔子矩阵就可以解除注释
  /*
  cout << "press 'Enter'to start registration" << endl;
  //——————————————————————————————————————————————————————
  //检测键盘输入
  while (!viewer3->wasStopped())
  {
	  viewer3->spinOnce(100);   //100??
	  if (_kbhit()) {//如果有按键按下，则_kbhit()函数返回真
		  ch = _getch();//使用_getch()函数获取按下的键值
	  }
	  if (ch == 13) { break; }//当按下Enter时停止循环，Enter键的键为13.
  }  
  */
  //——————————————————————————————————————————————————————
  //配准
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr OutCloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // 目标点云
  int method = 0;
  if (argc == 4) {
	  string methodname = argv[3];
	  method = stoi(methodname);
  }
  Eigen::Matrix4f registration_matrix;
  registration(trans_source_point_cloud, target_point_cloud, OutCloud,registration_matrix,method);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_registered_source_point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>); // 经过变换后的source点云
  pcl::transformPointCloud(*source_point_cloud, *trans_registered_source_point_cloud, registration_matrix*correspond_transformation);//pcl::transformPointCloud是右乘，所以先制行的矩阵变换写在右侧
  //——————————————————————————————————————————————————————
  //将配准后的点云加入显示
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>  Final_handler(OutCloud, 255, 255, 0);//黄色
  viewer3->addPointCloud(trans_registered_source_point_cloud, Final_handler, "trans_registered_source_point_cloud1", v2); //按照指定颜色显示变换后的source点云
  viewer3->addPointCloud(target_point_cloud, "target_point_cloud1", v2); //按照原颜色显示target点云
  while (!viewer3->wasStopped())
  {
	  viewer3->spinOnce(100);   //
  }
 return (0);
}
