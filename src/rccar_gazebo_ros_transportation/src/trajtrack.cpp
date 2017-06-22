#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"

#include "rc_car/Car_control.h"
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
 * Trajectory tracking integration with Gazebo
 */

geometry_msgs::Twist cmd_vel;

void chatterCallback(const rc_car::Car_control::ConstPtr& msg) {

   cmd_vel.linear.x = msg->u_velocity;

   cmd_vel.angular.z = msg->u_steering;

  // ROS_INFO("I heard: [%f]", msg->linear.x);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "trajtrack");

  std::string WheelName   = (std::string)"base_to_backwheels";
  std::string steering1   = (std::string)"base_to_steeringblock1";
  std::string steering2   = (std::string)"base_to_steeringblock2";

  ros::Time start(0);
  ros::Duration duration(0.01);


  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  gazebo_msgs::ApplyJointEffort applyJointEffort;
  gazebo_msgs::ApplyJointEffort applySteering1Effort;
  gazebo_msgs::ApplyJointEffort applySteering2Effort;
  gazebo_msgs::GetJointProperties steeringjoint1;
  gazebo_msgs::GetJointProperties steeringjoint2;
  gazebo_msgs::GetJointProperties basewheels;

  gazebo_msgs::GetModelState model;
  geometry_msgs::Pose curr_pose;

  double steer_des=0;

  double vel_set=0;

  ros::Subscriber vel_sub = n.subscribe<rc_car::Car_control>("/pctrl/velctrl", 1, chatterCallback);

  ros::Publisher pose_pub = n.advertise<geometry_msgs::TransformStamped>("/pctrl/pose", 10);

  ros::Publisher goal_pub = n.advertise<geometry_msgs::Vector3>("/pctrl/goal", 10);

  geometry_msgs::Vector3 goal;

  goal.x = 2;

  goal.y = 3;

  goal.z = 0;


   while (ros::ok())
   {

      goal_pub.publish(goal);

     vel_set = cmd_vel.linear.x;

      steer_des = cmd_vel.angular.z;

      //pose

      ros::ServiceClient ModelClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

      model.request.model_name = "Unicycle";

      ModelClient.call(model);

      curr_pose = model.response.pose;

      ros::Time currtime = ros::Time::now();

      geometry_msgs::TransformStamped transform1;

      geometry_msgs::Transform t1;

      geometry_msgs::Vector3 e;

      e.x = curr_pose.position.x;
      e.y = curr_pose.position.y;
      e.z = curr_pose.position.z;

      t1.translation = e;
      t1.rotation = curr_pose.orientation;

      transform1.transform = t1;

      transform1.header.stamp = currtime;

      pose_pub.publish(transform1);

//Joint control      
  /*    
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

      double e1 = steer_des - curr_steer1;
      double dedt1 = 0 - curr_vel1;
      double torque1 = 5*e1 + 2*(dedt1);

      double e2 = steer_des - curr_steer2;
      double dedt2 = 0 - curr_vel2;
      double torque2 = 5*e2 + 2*(dedt2);

      double e3 =  vel_set-curr_vel_wheels;
      double torque3 = 2*e3;


    //  ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
      
      applyJointEffort.request.joint_name = WheelName;
      applyJointEffort.request.effort = torque3;
      applyJointEffort.request.start_time= start;
      applyJointEffort.request.duration = duration;
  //    client.call(applyJointEffort);

      applySteering1Effort.request.joint_name = steering1;
      applySteering1Effort.request.effort = torque1;
      applySteering1Effort.request.start_time= start;
      applySteering1Effort.request.duration = duration;
 //     client.call(applySteering1Effort);

      applySteering2Effort.request.joint_name = steering2;
      applySteering2Effort.request.effort = torque2;
      applySteering2Effort.request.start_time= start;
      applySteering2Effort.request.duration = duration;
  //    client.call(applySteering2Effort);*/

      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
