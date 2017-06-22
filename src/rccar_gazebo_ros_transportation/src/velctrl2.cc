#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"

#include "gcop_comm/CtrlTraj.h"
//#include "gcop_comm/CurrPose.h"
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>

/**
 * GCOP integration with Gazebo
 */

geometry_msgs::Twist cmd_vel;
double kp = 6;//Proportional gain
double ki = 0;//Integration gain DIsABLED	
double int_e = 0;//Integration for error in velocity

float lin_vel;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg) {

   cmd_vel.linear.x = msg->linear.x;

   cmd_vel.angular.z = msg->angular.z;

  // ROS_INFO("I heard: [%f]", msg->linear.x);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "velctrl");

  std::string WheelName   = (std::string)"base_to_backwheels";

  std::string steering1   = (std::string)"Unicycle::steeringblock1";
  std::string steering2   = (std::string)"Unicycle::steeringblock2";

  std::string unicycle   = (std::string)"Unicycle";

  ros::Time start(0);
  ros::Duration duration(0.01);


  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  ros::Duration(5.0).sleep();

  gazebo_msgs::ApplyJointEffort applyJointEffort;
  gazebo_msgs::ApplyJointEffort applySteering1Effort;
  gazebo_msgs::ApplyJointEffort applySteering2Effort;
  gazebo_msgs::GetJointProperties steeringjoint1;
  gazebo_msgs::GetJointProperties steeringjoint2;
  gazebo_msgs::GetJointProperties basewheels;

  gazebo_msgs::GetModelState model;
  geometry_msgs::Pose curr_pose;

  float steer_des= 0;

  double vel_set=0;


  gazebo_msgs::GetLinkState getsteering1;
  gazebo_msgs::LinkState localsteering1;
  gazebo_msgs::SetLinkState setsteering1;

  gazebo_msgs::GetLinkState getsteering2;
  gazebo_msgs::LinkState localsteering2;
  gazebo_msgs::SetLinkState setsteering2;

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("/rccar/cmd_vel1", 1000, chatterCallback);


   while (ros::ok())
   {


      vel_set = cmd_vel.linear.x;

      steer_des = cmd_vel.angular.z;



     //joint properties
      ros::ServiceClient JointClient = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

      basewheels.request.joint_name = WheelName;

      JointClient.call(basewheels);


      float curr_vel_wheels = basewheels.response.rate[0]; //speed of backwheels

      lin_vel = curr_vel_wheels *.05;


     // set steering angles

     ros::ServiceClient gls_client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
     ros::ServiceClient sls_client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");


      getsteering1.request.link_name = steering1;
      getsteering1.request.reference_frame = "carbody";
      gls_client.call(getsteering1);


      localsteering1 = getsteering1.response.link_state;
      localsteering1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,steer_des);
      localsteering1.reference_frame = "carbody";



      setsteering1.request.link_state = localsteering1;
      sls_client.call(setsteering1);


      getsteering2.request.link_name = steering2;
      getsteering2.request.reference_frame = "carbody";
      gls_client.call(getsteering2);


      localsteering2 = getsteering2.response.link_state;
      localsteering2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,steer_des);
      localsteering2.reference_frame = "carbody";

      setsteering2.request.link_state = localsteering2;
      sls_client.call(setsteering2);

//speed control

      double e3 =  vel_set-lin_vel;
      int_e += e3;		
      double torque3 = kp*e3 + ki*int_e;


      ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
      
      applyJointEffort.request.joint_name = WheelName;
      applyJointEffort.request.effort = torque3;
      applyJointEffort.request.start_time= start;
      applyJointEffort.request.duration = duration;
      client.call(applyJointEffort);


      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
