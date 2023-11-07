#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions//pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"pcl_rviz");
    ros::NodeHandle nh;
    std::string pcdfile;
    nh.param<std::string>("pcdfile", pcdfile, "points.pcd");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 output;

    pcl::io::loadPCDFile(pcdfile,cloud);
    //读取文件绝对路径.
    //对点云坐标做变换，绕x轴旋转90度，将z轴指向上方
    //octomap_server，它的坐标系是定义z轴向上的，得到栅格地图是默认投影到xy平面
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)));
    pcl::transformPointCloud(cloud, cloud, transform);
    	
    pcl::toROSMsg(cloud,output);
    output.header.frame_id = "odom1";   
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
