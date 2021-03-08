#include <sensor_msgs/Imu.h>

#include <string>
#include "ros/ros.h"
#include "Eigen/Geometry"
// Returns a quaternion representing the same rotation as the given 'angle_axis'
// vector.
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

sensor_msgs::Imu last_imu_data;
Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
double imu_gravity_time_constant_ = 10;
Eigen::Vector3d gravity_vector_ = Eigen::Vector3d::UnitZ();

ros::Publisher imu_pub ;
int init_start= 0;
Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
  return Eigen::Quaterniond::FromTwoVectors(a, b);
}
  void imu_callback(sensor_msgs::Imu::ConstPtr msg) {
    static  bool is_first  =true;
    if (is_first) {
      last_imu_data = *msg;
      is_first = false;
      return;
    }
    sensor_msgs::Imu imu =  *msg;
    double temp  = imu.angular_velocity.x;
    imu.angular_velocity.x = imu.angular_velocity.z;
    imu.angular_velocity.z = -imu.angular_velocity.y;
    imu.angular_velocity.y = -temp;

    temp  = imu.linear_acceleration.x;
    imu.linear_acceleration.x = imu.linear_acceleration.z;
    imu.linear_acceleration.z = -imu.linear_acceleration.y;
    imu.linear_acceleration.y = -temp;

    const double delta_t =
        last_imu_data.header.stamp.toSec() - msg->header.stamp.toSec();
    Eigen::Vector3d imu_linear_acceleration;
    Eigen::Vector3d imu_angular_velocity;


    imu_angular_velocity.x() =imu.angular_velocity.x;
    imu_angular_velocity.y() =imu.angular_velocity.y;
    imu_angular_velocity.z() =imu.angular_velocity.z;

    imu_linear_acceleration.x() =imu.linear_acceleration.x;
    imu_linear_acceleration.y() =imu.linear_acceleration.y;
    imu_linear_acceleration.z() =imu.linear_acceleration.z;

    const double alpha = 0.8;//1. - std::exp(-delta_t / imu_gravity_time_constant_);
    gravity_vector_ =
        (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;

    Eigen::Quaterniond rotation = FromTwoVectors(
        gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
    orientation_ = (orientation_ * rotation).normalized();

    rotation = AngleAxisVectorToRotationQuaternion(
        Eigen::Vector3d(imu_angular_velocity * delta_t));

    orientation_ = (orientation_ * rotation).normalized();
    gravity_vector_ = rotation.conjugate() * gravity_vector_;


    last_imu_data = imu;
    imu.orientation.w = orientation_.w(); 
    imu.orientation.x = orientation_.x(); 
    imu.orientation.y = orientation_.y(); 
    imu.orientation.z = orientation_.z(); 

    if(init_start<1000){
      init_start++;
      return ;
    }
    imu_pub.publish(imu);

  }

  int main(int argc, char** argv) {
   
    ros::init(argc,argv,"realsen_imutoimu");
    ros::NodeHandle nh("");
    imu_pub= nh.advertise<sensor_msgs::Imu>("imu_topic",1);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/camera/imu",1,imu_callback);
    ros::spin();
    
    

  }