#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <iomanip>
#include <std_msgs/Float32.h>

using namespace std;

std::ofstream out("/home/thy/catkin_ws/src/my_turtlebot_launch/dist.txt",std::ios::app);
int key;
void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
   //<<fixed<<setprecision(2)的作用就是将输出结果转化为小数点后两位 
   // out<<fixed<<setprecision(2)<<way_points(0)<<"\t"<<way_points(1)<<"\t"<<way_points(2)<<std::endl;
   out<<msg->angular_velocity.x<<" "<<msg->angular_velocity.y<<" "<<msg->angular_velocity.z<<" "<<
   msg->linear_acceleration.x<<" "<<msg->linear_acceleration.y<<" "<<msg->linear_acceleration.z<<std::endl; 
   key ++;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"pub_txt"); //初始化节点
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("odom", 1000, subCallback);
    ros::Subscriber dist_pub = nh.subscribe<sensor_msgs::Imu>("imu",10000, &imuCallback);
    ros::Rate loop(1);
    while(ros::ok())
    {
        ROS_INFO("%d msg has been saved.", key);
        ros::spinOnce();
        loop.sleep();
    }
    
    out.close();
    return 0;
}
