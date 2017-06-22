#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>

#include <sstream>

/**
 * Simple keyboard joint control
 */

geometry_msgs::Twist cmd_vel;

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg) {

   cmd_vel.linear.x = msg->linear.x;

   cmd_vel.angular.z = msg->angular.z;

  // ROS_INFO("I heard: [%f]", msg->linear.x);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "movewheel");

  std::string WheelName   = (std::string)"base_to_backwheels";
  std::string steering1   = (std::string)"base_to_steeringblock1";
  std::string steering2   = (std::string)"base_to_steeringblock2";

  ros::Time start(0);
  ros::Duration duration(0.01);


  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  gazebo_msgs::ApplyJointEffort applyJointEffort;
  gazebo_msgs::ApplyJointEffort applySteering1Effort;
  gazebo_msgs::ApplyJointEffort applySteering2Effort;
  gazebo_msgs::GetJointProperties steeringjoint1;
  gazebo_msgs::GetJointProperties steeringjoint2;
  gazebo_msgs::GetJointProperties basewheels;

  double steer_des=0;

  double vel_set=0;

  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("/rccar/cmd_vel", 1000, chatterCallback);

   while (ros::ok())
   {

      //ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("/rccar/cmd_vel", 1000, chatterCallback);


      vel_set = cmd_vel.linear.x;

      steer_des = cmd_vel.angular.z/3;

      

      ros::ServiceClient JointClient = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

      steeringjoint1.request.joint_name = steering1;
      steeringjoint2.request.joint_name = steering2;

      basewheels.request.joint_name = WheelName;

      JointClient.call(steeringjoint1);

      JointClient.call(steeringjoint2);

      JointClient.call(basewheels);

      float curr_steer1 = steeringjoint1.response.position[0];
      float curr_steer2 = steeringjoint2.response.position[0];

      float curr_vel1 = steeringjoint1.response.rate[0];
      float curr_vel2 = steeringjoint2.response.rate[0];

      float curr_vel_wheels = basewheels.response.rate[0];

      float lin_vel = curr_vel_wheels *.05;

      double e1 = steer_des - curr_steer1;
      double dedt1 = 0 - curr_vel1;
      double torque1 = 5*e1 + 2*(dedt1);

      double e2 = steer_des - curr_steer2;
      double dedt2 = 0 - curr_vel2;
      double torque2 = 5*e2 + 2*(dedt2);

      double e3 =  vel_set-lin_vel;
      double torque3 = 5*e3;



      ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");


      
      applyJointEffort.request.joint_name = WheelName;
      applyJointEffort.request.effort = torque3;
      applyJointEffort.request.start_time= start;
      applyJointEffort.request.duration = duration;
      client.call(applyJointEffort);

      applySteering1Effort.request.joint_name = steering1;
      applySteering1Effort.request.effort = torque1;
      applySteering1Effort.request.start_time= start;
      applySteering1Effort.request.duration = duration;
      client.call(applySteering1Effort);

      applySteering2Effort.request.joint_name = steering2;
      applySteering2Effort.request.effort = torque2;
      applySteering2Effort.request.start_time= start;
      applySteering2Effort.request.duration = duration;
      client.call(applySteering2Effort);

      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
