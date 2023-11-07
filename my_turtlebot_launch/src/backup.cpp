#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

sensor_msgs::PointCloud pointcloud;
pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);

void cloudHandler(const sensor_msgs::PointCloudConstPtr& laserCloudMsg)
{
	pointcloud = *laserCloudMsg;
	pcl::PointXYZ point;
	for(int i=0; i<laserCloudMsg->points.size();i++)
	{
		point.x = laserCloudMsg->points[i].x;
		point.y = laserCloudMsg->points[i].y;
		point.z = laserCloudMsg->points[i].z;

		cloud->points.emplace_back(point);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"cylinder_fit");
    ros::NodeHandle nh;
	ros::Rate loop_rate(1);

	std::string lidar_topic;
	std::string plane_topic;
	std::string filter_topic;
	double min_x, max_x, min_y, max_y, min_z, max_z;
	nh.param<std::string>("lidar_topic", lidar_topic, "livox_points");
	nh.param<std::string>("plane_topic", plane_topic, "plane_points");	
	nh.param<std::string>("filter_topic", filter_topic, "filter_points");
	// nh.param<double>("min_x", min_x, 0);
	// nh.param<double>("max_x", max_x, 5);
	// nh.param<double>("min_y", min_y, -5);
	// nh.param<double>("max_y", max_y, 5);
	// nh.param<double>("min_z", min_z, -5);
	// nh.param<double>("max_z", max_z, -0.3);

	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud>(lidar_topic, 10, &cloudHandler);
	ros::Publisher filter_pub = nh.advertise<sensor_msgs::PointCloud2>(filter_topic, 3, true);
	ros::Publisher plane_pub = nh.advertise<sensor_msgs::PointCloud2>(plane_topic, 3, true);
	while(ros::ok())
	{
		//------------------------------读取点云数据---------------------------------
		if( cloud->size() == 0)
		{
			//auto first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(lidar_topic, nh);
			cout<<cloud->size()<<endl;
			cloud->clear();
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		//--------------------------------直通滤波-----------------------------------
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(cloud);
		cout<<cloud_filtered->size()<<endl;
		// pcl::PassThrough<pcl::PointXYZ > pass;
		// pass.setInputCloud(cloud);
		// pass.setFilterFieldName("z");//将Z轴不在（0，1.5）范围内的点过滤掉
		// pass.setFilterLimits(0, 1.5);
		// pass.filter(*cloud_filtered);//剩余的点储存在cloud_filtered中后续使用
		// cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << endl;
		//--------------------------------计算法线-----------------------------------
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
		normal_est.setSearchMethod(tree);
		normal_est.setInputCloud(cloud_filtered);
		normal_est.setKSearch(50);
		normal_est.compute(*cloud_normals);
		//------------------------------创建分割对象---------------------------------
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
		// ------------------------点云分割，提取平面上的点--------------------------
		// seg.setOptimizeCoefficients(true);
		// seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		// seg.setNormalDistanceWeight(0.1);
		// seg.setMethodType(pcl::SAC_RANSAC);
		// seg.setMaxIterations(100);
		// seg.setDistanceThreshold(0.03);
		// seg.setInputCloud(cloud_filtered);
		// seg.setInputNormals(cloud_normals);
		// seg.segment(*inliers_plane, *coefficients_plane);//获取平面模型系数和平面上的点
		// cout << "Plane coefficients: " << *coefficients_plane << endl;
		// //----------------------------------提取平面---------------------------------
		// pcl::ExtractIndices<pcl::PointXYZ > extract;
		// extract.setInputCloud(cloud_filtered);
		// extract.setIndices(inliers_plane);
		// extract.setNegative(false);
		// pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ >());
		// extract.filter(*cloud_plane);
		// cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << endl;
		//-----------------------------存储点云到输出文件----------------------------
		// pcl::PCDWriter writer;
		// writer.write("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);
		//-------------------------------提取圆柱体模型------------------------------
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
		pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
		//获取平面以外的点和点的法线
		// extract.setNegative(true);
		// extract.filter(*cloud_filtered2);
		// extract_normals.setNegative(true);
		// extract_normals.setInputCloud(cloud_normals);
		// extract_normals.setIndices(inliers_plane);
		// extract_normals.filter(*normals2);
		//为圆柱体分割创建分割对象，并设置参数
		seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
		seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
		seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
		seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
		seg.setMaxIterations(5000);               //设置迭代的最大次数
		seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值 
		seg.setRadiusLimits(0, 0.1);              //设置估计出圆柱模型的半径范围
		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normals);
		//获取圆柱模型系数和圆柱上的点
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
		cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
		//-----------------------------存储点云到输出文件----------------------------
		extract.setInputCloud(cloud_filtered2);
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
		extract.filter(*cloud_cylinder);
		if (cloud_cylinder->points.empty())
			cout << "Can't find the cylindrical component." << endl;
		else
		{
			cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
			//writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}