#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>

class CircleFit
{
    private:
        ros::NodeHandle nh;
        pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
        ros::Subscriber subLaserCloud;
        std::string pointCloudTopic;
    public:
        CircleFit(){
            nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
            subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &CircleFit::cloudHandler, this, ros::TransportHints().tcpNoDelay());


        };

        void fit_circle(double* center_x, double* center_y, double* radius)
        {
            double sum_x, sum_y, sum_x2, sum_y2, sum_x3, sum_y3, sum_xy, sumx1y2, sumx2y1;
            int N = laserCloudIn->points

        }

        void pointcallback(sensor_msgs::PointCloud2ConstPtr& msg)
        {
            // points filter
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);

        }

}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "yaw_cal");
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
        ("velodyne", 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());nh.subscribe<sensor_msgs::PointCloud2ConstPtr>("velodyne", 5, &pointcallback);
    // ros::service::waitForService("gazebo");
    ros::Time current_time;

    ros::Rate loop_rate(10);
    ros::spin();
    return 0;

}





