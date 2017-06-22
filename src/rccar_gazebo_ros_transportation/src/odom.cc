#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetJointProperties.h>
#include "rampage_msgs/WhlOdom.h"
#include "rampage_msgs/ImuSimple.h"


#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <math.h>

#include <iostream>


tf::TransformBroadcaster *odom_broadcaster;
int ros_odom_pub_seq;
std::string ros_odom_frame;
std::string ros_child_frame;
std::string gz_model_name;
double ros_odom_tf_future_date;

ros::Time t_prev(0);
double g_th;
double g_omega;
geometry_msgs::Transform state_current;


void cbWheelOdom(const rampage_msgs::WhlOdom::ConstPtr &msg)
{ 

  if(t_prev.toSec() == 0)
  {
    t_prev = msg->t_epoch;
    return;
  }
  double t = msg->t_epoch.toSec() - t_prev.toSec();
  t_prev = msg->t_epoch;

  if(t != t) {
       return;
  }
  //Estimate the current state of the car
  double x  = state_current.translation.x;
  double y  = state_current.translation.y;
  double th = g_th;
//  double th = 2*acos(state_current.basepose.rotation.w);


  double wt = g_omega*t;
  double ut = msg->delta_angle*0.0061*2*M_PI/1024;

  std::cout<<x<<" "<<y<<" "<<g_th<<" "<<t<<" "<< ut <<" "<< wt <<std::endl;



  if(abs(wt)<1e-6)//handle small w
  {
    x = x + cos(th+wt/2)*ut;
    y = y + sin(th+wt/2)*ut;
  } else {
    x = x - ( sin( th ) - sin( th + wt) )*ut/wt;
    y = y + ( cos( th ) - cos( th + wt) )*ut/wt;
  }

  th = th + wt;
  state_current.translation.x = x;
  state_current.translation.y = y;

  geometry_msgs::TransformStamped odom_trans;


  ros::Duration future_date(ros_odom_tf_future_date);
  odom_trans.header.stamp             = t_prev;
  odom_trans.header.frame_id          = ros_odom_frame;
  odom_trans.child_frame_id           = ros_child_frame;
  odom_trans.transform.translation.x  = x;
  odom_trans.transform.translation.y  = y;
  odom_trans.transform.translation.z  = 0.0;
  odom_trans.transform.rotation       = tf::createQuaternionMsgFromYaw(th);
  odom_broadcaster->sendTransform(odom_trans);

  odom_trans.header.stamp             = t_prev;
  odom_trans.header.frame_id          = ros_child_frame;
  odom_trans.child_frame_id           = "/laser";
  odom_trans.transform.translation.x  = .2;
  odom_trans.transform.translation.y  = 0;
  odom_trans.transform.translation.z  = 0.35;
  odom_trans.transform.rotation       = tf::createQuaternionMsgFromYaw(0);
  odom_broadcaster->sendTransform(odom_trans);


  g_th = th;

}


void cbImuSimple(const rampage_msgs::ImuSimple::ConstPtr &msg)
{
  if(msg->gz != msg->gz) {
      return;
  }
 g_omega = msg->gz;
}

int main( int argc, char* argv[] )
{
   state_current.translation.x = 0;
   state_current.translation.y = 0;
   state_current.translation.z = 0;
   g_th = 0;
   g_omega = 0;
  // Initialize ROS
  ros::init(argc, argv, "Odom_tf");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  n_.param<std::string>("ros_odom_frame", ros_odom_frame,   "/odom");
  n_.param<std::string>("ros_child_frame", ros_child_frame, "/base_link");
  n_.param<double>("ros_odom_tf_future_date", ros_odom_tf_future_date,  0);
  
  n_.param<std::string>("gz_model_name", gz_model_name,   "Rampage");
  
  odom_broadcaster = new tf::TransformBroadcaster;
  ros_odom_pub_seq = 0;

  // When launched from a launch file we need to give Gazebo time to load
  ros::Duration(5.0).sleep();
   // Subscribers

  ros::Subscriber vel_sub = n.subscribe<rampage_msgs::WhlOdom>("/whl_odom", 1, cbWheelOdom);
  ros::Subscriber vel_sub1 = n.subscribe<rampage_msgs::ImuSimple>("/imu_simple", 1, cbImuSimple);

  // Spin
  ros::spin();

  return 0;

}
