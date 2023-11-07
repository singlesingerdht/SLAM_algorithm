#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

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

#include <Eigen/Geometry>

using namespace std;

sensor_msgs::PointCloud pointcloud;
pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);

void cloudHandler(const sensor_msgs::PointCloudConstPtr& laserCloudMsg)
{
	cloud->clear();
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
	std::string tnl_topic;
	std::vector<double> extRotV, extRPYV, extTransV;
	nh.param<std::string>("lidar_topic", lidar_topic, "livox_points");
	nh.param<std::string>("plane_topic", plane_topic, "plane_points");	
	nh.param<std::string>("filter_topic", filter_topic, "filter_points");
	nh.param<std::string>("tunnel_param", tnl_topic, "tunnel_param");
	nh.param<vector<double>>("extrinsicRot", extRotV, vector<double>{0,0,-1,0,1,0,1,0,0});
	nh.param<vector<double>>("extrinsicRPY", extRPYV, vector<double>());
	nh.param<vector<double>>("extrinsicTrans", extTransV, vector<double>());
	auto extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
	auto extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
	auto extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);

	ros::Publisher param_pub = nh.advertise<geometry_msgs::PoseStamped>("tunnel_param", 10);
	ros::Publisher eular_pub = nh.advertise<geometry_msgs::Point32>("angles",10);
	ros::Publisher filter_pub = nh.advertise<sensor_msgs::PointCloud2>(filter_topic, 3, true);
	ros::Publisher plane_pub = nh.advertise<sensor_msgs::PointCloud2>(plane_topic, 3, true);
	ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud>(lidar_topic, 10, &cloudHandler);

	while(ros::ok())
	{
		//------------------------------读取点云数据---------------------------------
		if( cloud->size() == 0)
		{
			//auto first_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(lidar_topic, nh);
			//cout<<cloud->size()<<endl;
			cloud->clear();
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		//--------------------------------直通滤波-----------------------------------
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered(cloud);
		//cout<<cloud_filtered->size()<<endl;
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
		pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
		pcl::ExtractIndices<pcl::PointXYZ > extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ >);
		pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
		seg.setOptimizeCoefficients(true);        //设置对估计的模型系数需要进行优化
		seg.setModelType(pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
		seg.setMethodType(pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
		seg.setNormalDistanceWeight(0.1);         //设置表面法线权重系数
		seg.setMaxIterations(5000);               //设置迭代的最大次数
		seg.setDistanceThreshold(0.05);           //设置内点到模型的距离允许最大值 
		seg.setRadiusLimits(9,11 );              //设置估计出圆柱模型的半径范围
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals);
		//获取圆柱模型系数和圆柱上的点
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
		//cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
		//-----------------------------存储点云到输出文件----------------------------
		extract.setInputCloud(cloud);
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
		extract.filter(*cloud_cylinder);
		if (cloud_cylinder->points.empty())
			cout << "Can't find the cylindrical component." << endl;
		else
		{
			//cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
			//writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
		}

		// point in lidar frame
		Eigen::Vector3d tnl_pnt(coefficients_cylinder->values[0],coefficients_cylinder->values[1],coefficients_cylinder->values[2]);
		// tunnel vector in lidar frame
		Eigen::Vector3d tnl_vec(coefficients_cylinder->values[3],coefficients_cylinder->values[4],coefficients_cylinder->values[5]);
		
		// point in imu frame
		Eigen::Vector3d tnl_pnt_truck = extRot*tnl_pnt;
		// tunnel vector in imu frame
		Eigen::Vector3d tnl_vec_truck = extRot*tnl_vec;

		// make sure tunnel axis direction is along with truck direction
		if(tnl_vec_truck[1] < 0)
			tnl_vec_truck = -tnl_vec_truck;
		Eigen::Vector3d dest(0,1,0);
		
		// angle between directions of imu and tunnel 
		double theta = acos(tnl_vec_truck.dot(dest)/tnl_vec_truck.norm());
		// rotating vector is vertical to imu_vec and tnl_vec
		auto rot_vec = tnl_vec_truck.cross(dest);
		rot_vec.normalize();
		
		Eigen::AngleAxisd rotation_vector(theta, rot_vec);
		Eigen::Vector3d ori_in_tnl = rotation_vector.matrix() * tnl_pnt_truck;
		Eigen::Quaterniond quaternion(rotation_vector);
		Eigen::Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(2,1,0);
		geometry_msgs::Point32 point; 
		point.x = eulerAngle[0];
		point.y = eulerAngle[1];
		point.z = eulerAngle[2];
		eular_pub.publish(point);

		geometry_msgs::PoseStamped msg;
		msg.header = pointcloud.header;
		msg.pose.orientation.w = quaternion.w();
		msg.pose.orientation.x = quaternion.x();
		msg.pose.orientation.y = quaternion.y();
		msg.pose.orientation.z = quaternion.z();
		msg.pose.position.x = ori_in_tnl.x();
		msg.pose.position.y = ori_in_tnl.y();
		msg.pose.position.z = ori_in_tnl.z();

		param_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}