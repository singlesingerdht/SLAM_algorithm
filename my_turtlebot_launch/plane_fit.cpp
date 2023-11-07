#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> // 拟合平面
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 pointcloud;
sensor_msgs::PointCloud2 pubpointcloud;

void rotationCal(Eigen::Vector3d norm_vec)
{
    norm_vec.normalize();
    if(norm_vec[0] < 0){
        norm_vec = -norm_vec;
    }

    Eigen::Vector3d dest_vec(1, 0, 0);
    Eigen::Vector3d axis = (dest_vec + norm_vec)/2;
	Eigen::Vector3d x_axis(1,0,0);
    axis.normalize();
    Eigen::AngleAxisd rotation_vector(M_PI, axis);
	Eigen::AngleAxisd x_rotation_vector(M_PI, x_axis);
    Eigen::Matrix3d rotation_matrix = rotation_vector.matrix() * x_rotation_vector.matrix();
    cout<< rotation_matrix << endl;
    Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2,1,0);
    cout<< eulerAngle << endl;
}

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
	pointcloud = *laserCloudMsg;
	pubpointcloud.header = laserCloudMsg->header;
	pcl::moveFromROSMsg<pcl::PointXYZ>(pointcloud, *cloud);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"plane_rviz");
    ros::NodeHandle nh;

	std::string lidar_topic;
	std::string plane_topic;
	std::string filter_topic;
	double min_x, max_x, min_y, max_y, min_z, max_z;
	nh.param<std::string>("lidar_topic", lidar_topic, "velodyne_points");
	nh.param<std::string>("plane_topic", plane_topic, "plane_points");	
	nh.param<std::string>("filter_topic", filter_topic, "filter_points");
	nh.param<double>("min_x", min_x, 0);
	nh.param<double>("max_x", max_x, 5);
	nh.param<double>("min_y", min_y, -5);
	nh.param<double>("max_y", max_y, 5);
	nh.param<double>("min_z", min_z, -5);
	nh.param<double>("max_z", max_z, -0.3);

	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, &cloudHandler);
	ros::Publisher filter_pub = nh.advertise<sensor_msgs::PointCloud2>(filter_topic, 3, true);
	ros::Publisher plane_pub = nh.advertise<sensor_msgs::PointCloud2>(plane_topic, 3, true);
	auto first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(lidar_topic, nh);
	pcl::PassThrough<pcl::PointXYZ> passx, passy, passz;//创建滤波器对象
	passx.setInputCloud(cloud);
	passx.setFilterFieldName ("x");	
	passx.setFilterLimits (min_x, max_x);	
	passy.setInputCloud(cloud1);
	passy.setFilterFieldName ("y");	
	passy.setFilterLimits (min_y, max_y);	
	passz.setInputCloud (cloud2);			//设置待滤波的点云
	passz.setFilterFieldName ("z");		//设置在Z轴方向上进行滤波
	passz.setFilterLimits (min_z, max_z);		//设置滤波范围为0~1,在范围之外的点会被剪除
	//pass.setFilterLimitsNegative (true);//是否反向过滤，默认为false

	//cloud = cloud_filtered;	
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("拟合结果"));
	//-----------------------------读取点云----------------------------
	// std::string pcdfile;
    // nh.param<std::string>("pcdfile", pcdfile, "points.pcd");
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// if (pcl::io::loadPCDFile(pcdfile, *cloud) < 0)
	// {
	// 	PCL_ERROR("点云读取失败！\n");
	// 	return -1;
	// }
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
		do{
			ROS_INFO("No Filtered Points");
			passx.filter(*cloud1);
			passy.filter(*cloud2);
			passz.filter(*cloud_filtered);
			ros::spinOnce();
		}while(cloud_filtered->size()==0);
		pcl::toROSMsg(*cloud_filtered,pubpointcloud);
		filter_pub.publish(pubpointcloud);
		//--------------------------RANSAC拟合平面--------------------------
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	
		ransac.setDistanceThreshold(0.01);	//设置距离阈值，与平面距离小于0.1的点作为内点
		ransac.computeModel();				//执行模型估计
		//-------------------------根据索引提取内点--------------------------

		vector<int> inliers;				//存储内点索引的容器
		ransac.getInliers(inliers);			//提取内点索引
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered, inliers, *cloud_plane);
		//----------------------------输出模型参数---------------------------
		Eigen::VectorXf coefficient;
		ransac.getModelCoefficients(coefficient);
		cout << "平面方程为：\n" << coefficient[0] << "x + " << coefficient[1] << "y + " << coefficient[2] << "z + "
			<< coefficient[3] << " = 0" << endl;
		rotationCal(Eigen::Vector3d(coefficient[0], coefficient[1], coefficient[2]));
		//-----------------------------结果可视化----------------------------
		// viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");													
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");	
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");	

		// viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "plane");												
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "plane");	
		// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");	
		// // while (!viewer->wasStopped())
		// // {
		// // 	viewer->spinOnce(100);
		// // }
		// viewer->spinOnce(100);
		pcl::toROSMsg(*cloud_plane,pubpointcloud);
		plane_pub.publish(pubpointcloud);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}