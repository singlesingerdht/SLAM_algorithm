#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "path_wld");
    ros::NodeHandle nh;

    // ros::service::waitForService("gazebo");
    ros::ServiceClient state_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1,true);

    tf::TransformBroadcaster pose_br;
    tf::Transform worldtomap;
    tf::StampedTransform maptobase;
    tf::StampedTransform basetomap;
    tf::Quaternion qmap_to_base, qworld_to_base;
    tf::TransformListener map_ls;
    ros::Duration(10.0).sleep();

    nav_msgs::Path path;
    gazebo_msgs::GetModelState model_state;
    geometry_msgs::PoseStamped pose_stamped;
    ros::Time current_time;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {

        current_time = ros::Time::now();

        model_state.request.model_name = "turtlebot3";
        model_state.request.relative_entity_name = "world";
        state_client.call(model_state);

        path.header.stamp = current_time;
        path.header.frame_id = "world";

        pose_stamped.pose = model_state.response.pose;
        pose_stamped.header.stamp = current_time;
        pose_stamped.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        path_pub.publish(path);

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

        // pose_br.sendTransform(tf::StampedTransform(worldtomap, ros::Time::now(), "world", "map"));
        loop_rate.sleep();
    }
    return 0;
}