#include <string>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#define PI   3.14159265358979323846


class TrackAngle{

private:
  // Node Handle
  ros::NodeHandle n_;

  // Publishers and Subscribers
  ros::Publisher joint_pub;
  ros::Subscriber sub_lambda;

  // Messages
  sensor_msgs::JointState joint_state;

  // Wheelchair Parameters

  // Parameters for track angle computation
  double l;
  double t;
  double t_x;
  double t_z;
  double f;
  double f_x;
  double f_z;
  double phi;
  // Parameters for support system angle computation
  double b_0;
  double b_1;
  double a_1;
  double c_1;

  double sup_angle;

 public:
  TrackAngle(ros::NodeHandle n);
  void lambdaCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void setData();
  double pythagoras(double a_x, double a_z) {return sqrt(a_x*a_x + a_z*a_z);}

};

TrackAngle::TrackAngle(ros::NodeHandle n): l(.375), n_(n), phi(0)
{
  setData();
  sub_lambda = n_.subscribe("/Lambda",10, &TrackAngle::lambdaCallback, this);
  joint_pub = n_.advertise<sensor_msgs::JointState>("/joint_states", 1);
}

void TrackAngle::lambdaCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {

  // compute track angle
  l = (msg->data[0] + msg->data[1])/2/1000;
  l += .375;

  phi = acos((t*t + f*f - l*l)/(2*t*f)) - atan(t_z/t_x) - atan(f_z/f_x);

  // compute support system angle
  b_1 = b_0 + (msg->data[2] + msg->data[3])/2/1000;

  sup_angle = acos( (b_1*b_1 - a_1*a_1 - c_1*c_1)/(-2*a_1*c_1) ) - acos( (b_0*b_0 - a_1*a_1 - c_1*c_1)/(-2*a_1*c_1) );

  // Fill joint state message
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(3);
  joint_state.position.resize(3);

  joint_state.name[0] = "track_forks";
  joint_state.position[0] = phi;

  joint_state.name[1] = "base_to_laser_mount";
  joint_state.position[1] = 0;

  joint_state.name[2] = "support_pivot";
  joint_state.position[2] = -sup_angle;


  joint_pub.publish(joint_state);
}

void TrackAngle::setData() {
  // Track-Axle to Lambda-Track-Mount
  t_x = .681;
  t_z = .0752;

  // Track-Axle to Lambda-Frame-Mount
  f_x = .33;
  f_z = .0648;

  t = pythagoras(t_x, t_z);
  f = pythagoras(f_x, f_z);

  // Support System Parameters
  a_1 = .135;
  b_0 = .325;
  b_1 = b_0;
  c_1 = .39275;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  TrackAngle track_angle(n);
  ros::spin();
  return 0;
}