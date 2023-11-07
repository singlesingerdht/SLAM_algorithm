#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/navigation/NavState.h>

#include <queue>
using Eigen::VectorXd;
using Eigen::MatrixXd;
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

class KF
{
public:
  KF(MatrixXd A_, MatrixXd B_, MatrixXd I_, MatrixXd L_, MatrixXd H_,MatrixXd Q_, MatrixXd R_, MatrixXd P0, VectorXd x_):
  A(A_), B(B_), I(I_), L(L_), H(H_), Q(Q_), R(R_), P_state(P0), x(x_), key(0) 
  { }

  KF(){}
  
  virtual ~KF(){}

  void stateUpdate(VectorXd z, VectorXd u)
  {
    MatrixXd At, Bt, Ht, Lt; 
    At = A.transpose();
    Bt = B.transpose();
    Ht = H.transpose();
    Lt  = L.transpose();
    MatrixXd E = H*P_iter*Ht + L*R*Lt;
    P_iter = A*P_state*At + B*Q*Bt;
    K = P_iter*Ht*E.inverse();
    P_state = P_iter - K*H*P_iter;
    VectorXd x_imu;
    x_imu = A*x + B*u;
    x = x_imu + K*(z-H*x_imu);
    key ++;
  }
  
  MatrixXd getA()
  {
    return A;
  }

  MatrixXd getB()
  {
    return B;
  }

  VectorXd getX()
  {
    return x;
  }

  void setA(MatrixXd A_)
  {
    A = A_;
  }

  void setB(MatrixXd B_)
  {
    B = B_;
  }
  virtual void perform() { }

private:
    Eigen::MatrixXd A, B, I;
    Eigen::MatrixXd H, L;
    // Q and R are continueous variance, discrete variance equals to P/delt
    Eigen::MatrixXd Q, R;
    Eigen::MatrixXd K;
    Eigen::MatrixXd P_iter, P_state;
    Eigen::VectorXd x;

    int key;
};

class imu_KF : public KF
{
  public:
    imu_KF(MatrixXd A_, MatrixXd B_, MatrixXd I_, MatrixXd L_, MatrixXd H_,MatrixXd Q_, MatrixXd R_, MatrixXd P0, VectorXd x_)
    : KF(A_, B_, I_, L_, H_, Q_, R_, P0, x_)
    {
      nh.param<std::string>("tunnel_param", tnl_topic, "tunnel_param");
      nh.param<std::string>("imu_tf", imu_topic, "imu_tf");
      imu_sub = nh.subscribe<nav_msgs::Odometry>(imu_topic, 150, &imu_KF::ImuHandler, this);
      param_sub = nh.subscribe<geometry_msgs::PoseStamped>(tnl_topic, 1, &imu_KF::ParamHandler, this);
    }

  void ImuHandler(const nav_msgs::OdometryConstPtr &msg) {
    imu_que.push( *msg );
  }

  void ParamHandler(const geometry_msgs::PoseStampedConstPtr &msg) {
    tnl_que.push( *msg );
  }

  void recalAB(double timebetween)
  {
    MatrixXd idt = MatrixXd::Identity(3,3);
    MatrixXd timeMatrix = timebetween * idt;
    MatrixXd timeMatrix2 = 0.5*timebetween*timebetween * idt / N;
    MatrixXd A_ = getA();
    MatrixXd B_ = getB();
    VectorXd x_ = getX();
    A_.block(3,6,3,3) = timeMatrix;
    gtsam::so3::DexpFunctor local(dR(x_));
    B_.block(0,0,3,3) = local.dexp();
    B_.block(3,3,3,3) = local.expmap();
    B_.block(6,6,3,3) = local.expmap();
    B_.block(3,9,3,3) = timeMatrix;
    setA(A_);
    setB(B_);
  }

  virtual void perform()
  {
    ros::Rate loop(10);
    while(ros::ok())
    {
      VectorXd z, u;
      auto imu_msg = imu_que.front();
      auto tnl_msg = tnl_que.front();
      double msg_sec = abs(imu_msg.header.stamp.toSec()-tnl_msg.header.stamp.toSec());
      if( msg_sec > 0.05 )
        ROS_BREAK();
      
      u = imu2vec(imu_msg);
      z = tnl2vec(tnl_msg);
      stateUpdate(z, u);
      ros::spinOnce();
      loop.sleep();
    }    
  }

  Vector9 imu2vec(nav_msgs::Odometry &msg)
  {
    Vector9 state;
    state(0) = msg.pose.pose.orientation.x;
    state(1) = msg.pose.pose.orientation.y;
    state(2) = msg.pose.pose.orientation.z;

    state(3) = msg.pose.pose.position.x;
    state(4) = msg.pose.pose.position.y;
    state(5) = msg.pose.pose.position.z;

    state(6) = msg.twist.twist.linear.x;
    state(7) = msg.twist.twist.linear.x;
    state(8) = msg.twist.twist.linear.x;
    return state;
  }

  VectorXd tnl2vec(geometry_msgs::PoseStamped msg)
  {
    VectorXd state;
    state(0) = msg.pose.orientation.x;
    state(1) = msg.pose.orientation.y;
    state(2) = msg.pose.orientation.z;
    state(3) = msg.pose.position.x;
    state(4) = msg.pose.position.z;
    return state;
  }

  private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber param_sub;
    std::string imu_topic, tnl_topic;
    int N;
    std::queue<nav_msgs::Odometry>  imu_que;
    std::queue<geometry_msgs::PoseStamped>  tnl_que;
    geometry_msgs::PoseStamped tnl_msg;
    
};

void initialize(MatrixXd &A, MatrixXd &B,MatrixXd &I, MatrixXd &L, 
                            MatrixXd &H,MatrixXd &Q,  MatrixXd &R, MatrixXd &P0, VectorXd &x0)
{
  double dt = 0.1;
  A << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 1, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 1, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 1, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 1, 0, 0, dt,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
  B << 0, 0, 0, 0, 0, 0,
      dt, 0, 0, 0, 0, 0,
      0, dt, 0, 0, 0, 0,
      0, 0, dt, 0, 0, 0;
  H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
  Q << 0.0001, 0, 0, 0, 0, 0,
              0, 0.0001, 0, 0, 0, 0,
              0.0, 0.0001, 0, 0, 0, 0,
              0, 0, 0, 0.0001, 0, 0,
              0.0, 0, 0, 0, 0.0001, 0,
              0.0, 0, 0, 0, 0, 0.0001;
  R << 0.0001, 0, 0, 0, 0,
          0, 0.0001, 0, 0, 0,
          0, 0, 0.0001, 0, 0,
          0, 0, 0, 0.0001, 0,
          0, 0, 0, 0, 0.0001;
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  P0 << 0.01, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0.01, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0.01, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0.01, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0.01, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0, 0;
}                         

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_kf");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  MatrixXd A(9,9), B(9,6), I(9,6), L(5,5), H(5,9);
  MatrixXd Q(6,6), R(5,5);
  MatrixXd P0(9,9);
  VectorXd x0(9);
  initialize(A, B, I, L, H, Q, R, P0, x0);
  imu_KF imu_kf(A, B, B, I, H, Q, R, P0, x0);
  ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

  return 0;
}
