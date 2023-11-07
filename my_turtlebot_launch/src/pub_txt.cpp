#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <iomanip>
#include <std_msgs/Float32.h>

using namespace std;

void distCallback(const std_msgs::Float32::ConstPtr &msg)
{
    std::ofstream out("/home/thy/catkin_ws/src/my_turtlebot_launch/dist.txt",std::ios::app);
   //<<fixed<<setprecision(2)的作用就是将输出结果转化为小数点后两位 
   // out<<fixed<<setprecision(2)<<way_points(0)<<"\t"<<way_points(1)<<"\t"<<way_points(2)<<std::endl;
   out<<fixed<<setprecision(2)<<msg->data<<std::endl; 
   // out<<fixed<<setprecision(2)<<x<<" "<<y<<" "<<z<<std::endl;
   out.close();
}

void subCallback(const geometry_msgs::PointStamped &msg)
{
   Eigen::Vector3d way_points;
   way_points(0)  = msg.point.x * 100 / 100;
   way_points(1)  = msg.point.y * 100 / 100;
   way_points(2)  = msg.point.z * 100 / 100;
//    double x = msg.point.x * 100 / 100;
//    double y = msg.point.y * 100 / 100;
//    double z = msg.point.z * 100 / 100;

   std::ofstream out("/home/ubuntu/catkin_ws/src/my_turtlebot_launch/point.txt",std::ios::app);
   //<<fixed<<setprecision(2)的作用就是将输出结果转化为小数点后两位 
   // out<<fixed<<setprecision(2)<<way_points(0)<<"\t"<<way_points(1)<<"\t"<<way_points(2)<<std::endl;
   out<<fixed<<setprecision(2)<<way_points(0)<<"\t\t"<<way_points(1)<<"\t\t"<<way_points(2)<<std::endl; 
   // out<<fixed<<setprecision(2)<<x<<" "<<y<<" "<<z<<std::endl;
   out.close();
}
 
int main(int argc, char **argv){

    ros::init(argc,argv,"pub_txt"); //初始化节点
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("odom", 1000, subCallback);
    ros::Subscriber dist_pub = nh.subscribe("odom_dist",1,distCallback);
    ros::spin();
    return 0;
}
