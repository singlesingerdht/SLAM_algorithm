#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
// #include <Eigen/Eigen>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <iostream>

geometry_msgs::Pose global;
std_msgs::Float32 global_dist;
nav_msgs::OccupancyGrid global_map;
ros::Publisher ratial_pub;
ros::Publisher size_pub;
std_msgs::Float32 ratial;
std_msgs::Float32 size;
double height = 10;
double width = 20;
std::ofstream out("/home/thy/catkin_ws/src/my_turtlebot_launch/dist.txt",std::ios::app);

void time_callback(const std_msgs::Float64ConstPtr &msg) {
    out<<std::fixed<<std::setprecision(2)<<size.data<<"\t\t"<<
        msg->data<<std::endl; 
    ratial_pub.publish(ratial);
    size_pub.publish(size);
}

void map_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
    global_map = *msg;
}

void map_count() {
    unsigned int count = 0;
    double res = global_map.info.resolution;
    // double height = global_map.info.height;
    // double width = global_map.info.width;

    for(auto i = global_map.data.begin(); i != global_map.data.end(); i++) {
        if(*i != -1) {
            count ++;
        }
    }
    if(height != 0 && width != 0) {
        ratial.data = 100*count*res*res/height/width;
        size.data = count*res*res;
    }
}

void odom_callback(const nav_msgs::OdometryConstPtr &msg) {
    double x,y,z;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    if(global.position.z == -1000)
    {
        global.position.x = x;
        global.position.y = y;
        global.position.z = z;
    }
    else
    {
        global_dist.data += sqrt((x - global.position.x)*(x - global.position.x)\
         + (y - global.position.y)* (y - global.position.y));
        global.position.x = x;
        global.position.y = y;
        global.position.z = z;
    }
}

int main(int argc, char ** argv)
{
    if( !out.is_open()){
        ROS_ERROR("Faile to open file");
        ROS_BREAK();
    }
    global.position.z = -1000;
    ros::init(argc, argv, "path_odom");
    ros::NodeHandle nh;
    
    ratial_pub = nh.advertise<std_msgs::Float32>("explored_ratial",1,true);
    size_pub = nh.advertise<std_msgs::Float32>("explored_size", 5, true);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1,true);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("odom_dist", 1,true);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, odom_callback);
    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, map_callback);
    ros::Subscriber time_sub = nh.subscribe<std_msgs::Float64>("/explore/search_time", 1, time_callback);
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.orientation.w = 1;
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    ros::Time current_time = ros::Time::now();
    ros::Rate loop_rate(200);

    while( ros::ok() && (global.position.z == -1000) ) {
        ros::spinOnce();
    }

    while(ros::ok()) {
        
        current_time = ros::Time::now();
        pose_stamped.pose.position.x = global.position.x;
        pose_stamped.pose.position.y = global.position.y;
        pose_stamped.pose.position.z = global.position.z;
       
        pose_stamped.header.stamp = current_time;
        pose_stamped.header.frame_id = "odom";
        
        path.header.stamp = current_time;
        path.header.frame_id = "odom";
        path.poses.push_back(pose_stamped);
        path_pub.publish(path);
        dist_pub.publish(global_dist);
        map_count();
        // Eigen::Matrix3d mtx_map_to_base, mtx_wld_to_base, mtx_base_to_map, mtx_wld_to_map;
        // Eigen::Vector3d vec_map_to_base, vec_wld_to_base, vec_base_to_map, vec_wld_to_map;
        // Eigen::Quaterniond q_map_to_base, q_wld_to_base;
        // q_map_to_base = Eigen::Quaterniond(qmap_to_base.getW(),qmap_to_base.getX(), qmap_to_base.getY(), qmap_to_base.getZ());  //w x y z
        // q_wld_to_base = Eigen::Quaterniond(qworld_to_base.getW(),qworld_to_base.getX(), qworld_to_base.getY(), qworld_to_base.getZ());  //w x y z
        // mtx_map_to_base = q_map_to_base.matrix();
        // mtx_wld_to_base = q_wld_to_base.matrix();
        // mtx_base_to_map = mtx_map_to_base.transpose();
        // mtx_wld_to_map = mtx_wld_to_base * mtx_base_to_map;
        // vec_wld_to_base = Eigen::Vector3d(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
        // vec_map_to_base = Eigen::Vector3d(maptobase.getOrigin());
        // vec_base_to_map = -mtx_base_to_map * vec_map_to_base;
        // vec_wld_to_map = mtx_wld_to_base * vec_base_to_map + vec_wld_to_base;
        // Eigen::Quaterniond rotation(mtx_wld_to_map);
        // tf::Quaternion q(rotation.w(),rotation.x(),rotation.y(),rotation.z());
        // worldtomap.setOrigin(tf::Vector3(vec_wld_to_map[0], vec_wld_to_map[1], vec_wld_to_map[2]));
        // worldtomap.setRotation(q);
        ratial_pub.publish(ratial);
        size_pub.publish(size);
        // pose_br.sendTransform(tf::StampedTransform(worldtomap, ros::Time::now(), "world", "map"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    out.close();
    double time = ros::Time::now().toSec() - current_time.toSec();
    ROS_INFO("dist:%f time:%f", global_dist.data, time);
    return 0;
}