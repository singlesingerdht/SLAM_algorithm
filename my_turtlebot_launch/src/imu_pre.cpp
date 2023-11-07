#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

class imuPreintegration
{
    private:
    ros::NodeHandle nh;
    std::string imu_topic;
    ros::Subscriber inte_sub;
    ros::Publisher pubImuOdometry;

    std::vector<sensor_msgs::Imu> imuVec;
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;    

    bool doneInitial;
    gtsam::Vector3 gravityOri;
    size_t key;
    double imuAccNoise;
    double imuGyrNoise;
    double imuGravity;
    double lastImuT_imu;
    std::string pub_type;

  public:
    imuPreintegration()
    {
      nh.param<std::string>("type",pub_type,"Velocity");
      
      imu_topic = "imu";
      key = 0;
      inte_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic,150, &imuPreintegration::ImuHandler, this);
      pubImuOdometry = nh.advertise<nav_msgs::Odometry> ("imu_odom", 2000);
      imuAccNoise = 0.001;
      imuGyrNoise = 0.001;
      imuGravity = 9.81;
      lastImuT_imu = -1;
      gravityOri << 0, 0, 0;
      doneInitial = false;
      gtsam::Pose3 pose;
      gtsam::Vector3 vel;
      prevStateOdom = gtsam::NavState(pose, vel);     
    };

    void perform()
    {
      ros::Rate loop(10);
      for(int i=0; i<1000; i++)
      {
        ros::spinOnce();
        loop.sleep();
        if(!doneInitial)
        {
          initialAlign();
        }
      }
      if(doneInitial)
        ros::spin();
      else
        ROS_INFO("\033[1;32m----> IMU Preintegration Stopped.\033[0m");
    }

    void initialAlign()
    {
      for(key;key<imuVec.size();key++)
      {
        sensor_msgs::Imu msg =  imuVec[key];
        double x, y, z;
        x = msg.linear_acceleration.x;
        y =  msg.linear_acceleration.y;
        z =  msg.linear_acceleration.z;
        if(sqrt(x*x + y*y + z*z) <= imuGravity-0.1 || sqrt(x*x + y*y + z*z) >= imuGravity+0.1 || key>=1000)
        {
          boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
          gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
          p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
          p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
          p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
          
          p->n_gravity = -gravityOri;
          // //p->body_P_sensor;
          // std::cout<<gravityOri<<std::endl;
          imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);              
          // ROS_INFO("\033[1;32m----> Initialization done.\033[0m");
          doneInitial = true;
          break;
        }
        gravityOri = (gravityOri*key + gtsam::Vector3(x,y,z))/(key+1);
      }
    }

    void ImuHandler(const sensor_msgs::Imu::ConstPtr& msg)
    {
      sensor_msgs::Imu thisImu = *msg;
      imuVec.push_back(thisImu);
      //std::cout<<imuVec.size()<<std::endl;
      if(!doneInitial)
        return;
      double imuTime = msg->header.stamp.toSec();
        //lastImuT_imu变量初始被赋值为-1
        // 获得时间间隔, 第一次为1/500,之后是两次imuTime间的差
      double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
      //std::cout<<dt<<std::endl;
      lastImuT_imu = imuTime;
      imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                        gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);
      gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        // 发布imu里程计（转到lidar系，与激光里程计同一个系）
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = "imu_odom";
        odometry.child_frame_id = "imu";

        // transform imu pose to ldiar
        //预测值currentState获得imu位姿, 再由imu到雷达变换, 获得雷达位姿
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose;//.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
        //prevStateOdom = currentState;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integration");
  imuPreintegration iP;
  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
  iP.perform();
  return 0;
}
