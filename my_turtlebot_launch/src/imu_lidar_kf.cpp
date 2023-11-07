#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

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

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using Eigen::MatrixXd;
using Eigen::VectorXd;

using gtsam::Vector9;

static Eigen::Block<const Vector9, 3, 1> dR(const Vector9& v) {
return v.segment<3>(0);
}
static Eigen::Block<const Vector9, 3, 1> dP(const Vector9& v) {
return v.segment<3>(3);
}
static Eigen::Block<const Vector9, 3, 1> dV(const Vector9& v) {
return v.segment<3>(6);
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

class imuPreintegration
{
    private:
    std::mutex mtx;
    ros::NodeHandle nh;
    std::string imu_topic;
    std::string tnl_topic;
    ros::Subscriber inte_sub;
    ros::Subscriber tnl_param_sub;
    ros::Publisher imu_odom_pub;
    ros::Publisher tnl_odom_pub;
    ros::Publisher imu_kf_pub;

    std::deque<sensor_msgs::Imu> imuVec;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    gtsam::NavState curr_state;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool imuInitial=false;
    bool lidarInitial=false;    
    bool systemInitialized = false;
    gtsam::Vector3 gravityOri;
    unsigned int key;
    double imuAccNoise, imuGyrNoise, imuGravity;
    double imuAccBiasN, imuGyrBiasN;
    double lastImuT_imu;
    double delta_t;
    double currentCorrectionTime;
    double surroundingkeyframeAddingAngleThreshold;
    double surroundingkeyframeAddingDistThreshold;
    //gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3::AxisAngle(gtsam::Point3(0, 1, 0), 3.1416/2), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3::AxisAngle(gtsam::Point3(0, 1, 0), 0), gtsam::Point3(0, 0, 0));
    std::string pub_type;

    double lastImuT_opt = -1;
    gtsam::Pose3 lidarPose;
    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::NavState last_state;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    Eigen::Matrix3d Rot;
    Eigen::Vector3d origin;

  public:
    imuPreintegration()
    {
      key = 0;

      nh.param<std::string>("type",pub_type,"Velocity");
      nh.param<std::string>("tunnel_param", tnl_topic, "tunnel_param");
      nh.param<std::string>("imu", imu_topic, "imu");
      nh.param<double>("imuAccBiasN", imuAccBiasN, 0.0001);
      nh.param<double>("imuGyrBiasN", imuGyrBiasN, 0.00001);
      nh.param<double>("angleThreshold", surroundingkeyframeAddingAngleThreshold, 3.1416/5);
      nh.param<double>("angleThreshold", surroundingkeyframeAddingDistThreshold, 4);

      inte_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic,150, &imuPreintegration::ImuHandler, this);
      tnl_param_sub = nh.subscribe<geometry_msgs::PoseStamped>(tnl_topic, 1, &imuPreintegration::ParamHandler, this);
      imu_odom_pub = nh.advertise<geometry_msgs::Point32> ("acc_wld", 20);
      tnl_odom_pub = nh.advertise<nav_msgs::Odometry> ("tnl_odom", 20);
      imu_kf_pub = nh.advertise<nav_msgs::Odometry>("imu_tf", 20);
      imuAccNoise = 0.001; imuGyrNoise = 0.001; imuGravity = 9.81;
      gravityOri << 0, 0, 0;
      delta_t = 0.1;
      lastImuT_imu = -1;
      
      Rot << -5, -5, -5,
                    -5, -5, -5,
                    -5, -5, -5;
      imuInitial = false;
      curr_state = gtsam::NavState();     
      last_state = gtsam::NavState();

      priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 10, 1e-2).finished()); // rad,rad,rad,m, m, m
      priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
      priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
      correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
      correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
      noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    };

    void perform()
    {
      ros::Rate loop(10);
      while(!imuInitial)
      {
        ros::spinOnce();
        initialAlign();
        loop.sleep();
      }
      if(imuInitial) 
      {
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(gravityOri.norm());
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0.00, 0.00, 0.00, 0.00, 0.00, 0.00).finished()); // assume zero initial bias
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        auto msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(tnl_topic, nh);
        while(Rot(0) < -2) { 
          ros::spinOnce();
        }
        //  lack of provement of Rot
        //p->n_gravity = -Rot*gravityOri;
        //p->n_gravity = -gravityOri;

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
        ROS_INFO("\033[1;32m----> Start Navigating.\033[0m");
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
      }
      else
        ROS_INFO("\033[1;32m----> IMU Preintegration Stopped.\033[0m");
    }

    void ImuHandler(const sensor_msgs::Imu::ConstPtr& msg)
    {
      sensor_msgs::Imu thisImu = *msg;
      imuVec.push_back(thisImu);
      imuQueImu.push_back(thisImu);
      if(!imuInitial || !lidarInitial)
        return;
      
      double imuTime = msg->header.stamp.toSec();
        //lastImuT_imu变量初始被赋值为-1
        // 获得时间间隔, 第一次为1/500,之后是两次imuTime间的差
      double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
      //std::cout<<dt<<std::endl;
      lastImuT_imu = imuTime;
    
      Eigen::Vector3d a_I(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z);
      Eigen::Vector3d w_I(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z);
      Eigen::Vector3d a_wld = Rot*a_I;

      imuIntegratorImu_->integrateMeasurement(a_I, w_I, dt);
      curr_state = imuIntegratorImu_->predict(prevState_, prevBiasOdom);

      // 发布imu里程计（转到lidar系，与激光里程计同一个系)
      nav_msgs::Odometry odometry;
      odometry.header.stamp = thisImu.header.stamp;
      odometry.header.frame_id = "imu_odom";
      odometry.child_frame_id = "imu";

      // transform imu pose to ldiar
      //预测值currentState获得imu位姿, 再由imu到雷达变换, 获得雷达位姿
      gtsam::Pose3 imuPose = gtsam::Pose3(curr_state.quaternion(), curr_state.position());
      //gtsam::Pose3 lidar_Pose = imuPose;//.compose(imu2Lidar);
      odometryPublish(odometry, curr_state, tnl_odom_pub);
      
      geometry_msgs::Point32 point;
      point.x = a_wld.x();
      point.y = a_wld.y();
      point.z = a_wld.z();

      imu_odom_pub.publish(point);
    }

    void odometryPublish(nav_msgs::Odometry& odometry, gtsam::NavState currentState, ros::Publisher pub)
    {
      odometry.pose.pose.position.x = currentState.pose().translation().x();
      odometry.pose.pose.position.y = currentState.pose().translation().y();
      odometry.pose.pose.position.z = currentState.pose().translation().z();

      odometry.pose.pose.orientation.x = currentState.pose().rotation().toQuaternion().x();
      odometry.pose.pose.orientation.y = currentState.pose().rotation().toQuaternion().y();
      odometry.pose.pose.orientation.z = currentState.pose().rotation().toQuaternion().z();
      odometry.pose.pose.orientation.w = currentState.pose().rotation().toQuaternion().w();
      
      odometry.twist.twist.linear.x = currentState.velocity().x();
      odometry.twist.twist.linear.y = currentState.velocity().y();
      odometry.twist.twist.linear.z = currentState.velocity().z();

      odometry.twist.twist.angular.x = 0;
      odometry.twist.twist.angular.y = 0;
      odometry.twist.twist.angular.z = 0;
      pub.publish(odometry);
    }

    void odometryPublish(nav_msgs::Odometry& odometry, Vector9 currentState, ros::Publisher pub)
    {
      odometry.pose.pose.orientation.x = currentState(0);
      odometry.pose.pose.orientation.y = currentState(1);
      odometry.pose.pose.orientation.z = currentState(2);

      odometry.pose.pose.position.x = currentState(3);
      odometry.pose.pose.position.y = currentState(4);
      odometry.pose.pose.position.z = currentState(5);
      
      odometry.twist.twist.linear.x = currentState(6);
      odometry.twist.twist.linear.y = currentState(7);
      odometry.twist.twist.linear.z = currentState(8);
      pub.publish(odometry);
    }

    void initialAlign()
    {
      for(key;key<imuVec.size();key++) {
        sensor_msgs::Imu msg =  imuVec[key];
        double x, y, z;
        x = msg.linear_acceleration.x;
        y =  msg.linear_acceleration.y;
        z =  msg.linear_acceleration.z;
        if(key>=1000) {
          imuInitial = true;
          std::cout<<gravityOri<<std::endl;                 
          ROS_INFO("\033[1;32m----> Initialization done.\033[0m");
          break;
        }
        gravityOri = (gravityOri*key + gtsam::Vector3(x,y,z))/(key+1);
      }
    }

    void ParamHandler(const geometry_msgs::PoseStampedConstPtr &msg)
    {
      double x, y, z, w;
      x = msg->pose.orientation.x;
      y = msg->pose.orientation.y;
      z = msg->pose.orientation.z;
      w = msg->pose.orientation.w;

      Eigen::Quaternion<double> q(w, x, y, z);
      Rot = q.matrix();

      origin = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      std::lock_guard<std::mutex> lock(mtx);
      currentCorrectionTime = ROS_TIME(msg);
        
      if (imuVec.empty() || !imuInitial)
        return; 

      for (auto t = imuVec.begin(); t != imuVec.end(); t++)
      {
        if(t->header.stamp.toSec() < currentCorrectionTime - 0.05)
        {
          imuVec.pop_front();
        }
        else{
          break;
        }
      }

      // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
      lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(w, x, y, z), gtsam::Point3(-msg->pose.position.x, 0,-msg->pose.position.z));
      lidarInitial = true;
      nav_msgs::Odometry odometry;
      odometry.header.stamp = msg->header.stamp;
      odometry.header.frame_id = "imu_odom";
      odometry.child_frame_id = "imu";
      Vector9 dstate = imuIntegratorImu_->preintegrated();
      odometryPublish(odometry, dstate, imu_kf_pub);
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);       
      factorUpdate();
    }

    void factorUpdate( )
    {
       // 0. initialize system
      if (systemInitialized == false) {
        resetOptimization();
        // pop old IMU message
        while (!imuVec.empty())
        {
            if (ROS_TIME(&imuVec.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = ROS_TIME(&imuVec.front());
                imuVec.pop_front();
            }
            else
                break;
        }
        ROS_DEBUG("Insert first factor");
        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        key = 1;
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        systemInitialized = true;
        ROS_DEBUG("Insert first factor finished");
        return;
    }
    if(!keyFrame())
      return ;
    
    // ROS_DEBUG("Insert No. %d factor", key+1);
    // const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    // gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    // graphFactors.add(imu_factor);
    // // add imu bias between factor
    // graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
    //                   gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // // add pose factor
    // gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    // gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, priorPoseNoise);
    // graphFactors.add(pose_factor);
    // // insert predicted values
    // gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    // graphValues.insert(X(key), propState_.pose());
    // graphValues.insert(V(key), propState_.v());
    // graphValues.insert(B(key), prevBias_);
    // // optimize
    // optimizer.update(graphFactors, graphValues);
    // optimizer.update();
    // graphFactors.resize(0);
    // graphValues.clear();
    // // Overwrite the beginning of the preintegration for the next step.
    // gtsam::Values result = optimizer.calculateEstimate();

    // prevPose_  = result.at<gtsam::Pose3>(X(key));
    // prevVel_   = result.at<gtsam::Vector3>(V(key));
    // prevState_ = gtsam::NavState(prevPose_, prevVel_);
    // prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // // Reset the optimization preintegration object.
    // imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    // // 2. after optiization, re-propagate imu odometry preintegration
    //   prevStateOdom = prevState_;
    //   prevBiasOdom  = prevBias_;
    //   // first pop imu message older than current correction data
    //   double lastImuQT = -1;
    //   while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
    //   {
    //     lastImuQT = ROS_TIME(&imuQueImu.front());
    //     imuQueImu.pop_front();
    //   }
    //   // repropogate
    //   if (!imuQueImu.empty())
    //   {
    //     // reset bias use the newly optimized bias
    //     imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
    //     // integrate imu message from the beginning of this optimization
    //     for (int i = 0; i < (int)imuQueImu.size(); ++i)
    //     {
    //         sensor_msgs::Imu *thisImu = &imuQueImu[i];
    //         double imuTime = ROS_TIME(thisImu);
    //         double dt = (lastImuQT < 0) ? (1.0 / 500.0):(imuTime - lastImuQT);

    //         imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(Rot*Eigen::Vector3d(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z)),
    //                                                 gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,  thisImu->angular_velocity.z), dt);
    //         lastImuQT = imuTime;
    //     }
    //   }
    //   key++;
    //   ROS_DEBUG("Insert No. %d factor finished", key+1);
    }

    bool keyFrame()
    {
      auto prev_pose = prevState_.position();
      auto prev_att = prevState_.attitude();

      auto curr_pose = curr_state.position();
      auto curr_att = curr_state.attitude();

      Eigen::Affine3f transStart = pcl::getTransformation(prev_pose.x(), prev_pose.y(), prev_pose.z() ,
                                                                                                    prev_att.roll(), prev_att.pitch(), prev_att.yaw() );
      Eigen::Affine3f transEnd = pcl::getTransformation(curr_pose.x(), curr_pose.y(), curr_pose.z() ,
                                                                                                    curr_att.roll(), curr_att.pitch(), curr_att.yaw() );
      Eigen::Affine3f transBetween = transStart.inverse() * transEnd;
      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

      std::cout<<"x = "<<x<<"  y = "<<y<<" z = "<<z<<" roll = "<<roll<< " pitch = "<<pitch<<" yaw = "<<yaw <<std::endl;
      std::cout << "deltV = "<<prevState_.attitude()*imuIntegratorImu_->deltaVij() <<std::endl;
      std::cout << "deltV = "<<Rot*imuIntegratorImu_->deltaVij() <<std::endl;
      
      if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
          abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
          abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
          sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
        return false;

      return true; 
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integration");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  imuPreintegration iP;
  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
  iP.perform();
  return 0;
}
