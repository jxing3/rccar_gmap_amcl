#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"

#include "gcop_comm/CtrlTraj.h"
#include "gcop_comm/CurrPose.h"
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

void chatterCallback(const gcop_comm::CtrlTraj::ConstPtr& msg) {

   cmd_vel.linear.x = msg->statemsg[1].statevector[3];

   cmd_vel.angular.z = msg->ctrl[0].ctrlvec[1];

   if(cmd_vel.angular.z > M_PI/8) {
       cmd_vel.angular.z = M_PI/8;

   } else if(cmd_vel.angular.z < -M_PI/8) {

       cmd_vel.angular.z = -M_PI/8;
   }

   
   /*ROS_INFO("State velocities: [%f][%f][%f]",msg->statemsg[0].statevector[3],msg->statemsg[1].statevector[3],msg->statemsg[2].statevector[3]);
   ROS_INFO("States: [%f][%f][%f][%f]",msg->statemsg[0].statevector[0],msg->statemsg[0].statevector[1], msg->statemsg[0].statevector[2],msg->statemsg[0].statevector[3]);*/
   ROS_INFO("Cmd_Linearvel: [%f] Cmd_Angularvel: [%f] Actual_Linearvel: [%f]", cmd_vel.linear.x, cmd_vel.angular.z, lin_vel);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "intergcop1");


  std::string WheelName1   = (std::string)"rear_left_wheel_joint";
  std::string WheelName2   = (std::string)"rear_right_wheel_joint";
  //std::string WheelName   = (std::string)"base_to_backwheels";
 //std::string steering1   = (std::string)"base_to_steeringblock1";
 // std::string steering2   = (std::string)"base_to_steeringblock2";

  std::string steering1   = (std::string)"Unicycle::steeringblock1";
  std::string steering2   = (std::string)"Unicycle::steeringblock2";

  std::string unicycle   = (std::string)"Unicycle";

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
  geometry_msgs::Twist vel_ctrl;

  float steer_des= 0;

  double vel_set=0;

  ros::Subscriber vel_sub = n.subscribe<gcop_comm::CtrlTraj>("/dmoc/ctrltraj", 1, chatterCallback);

  ros::Publisher pose_pub = n.advertise<gcop_comm::CurrPose>("/dmoc/mocap", 10);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/rccar/cmd_vel",10);

    tf::TransformListener listener;

   while (ros::ok())
   {


      //pose



    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

     vel_pub.publish(cmd_vel);



      ros::Time currtime = ros::Time::now();

      geometry_msgs::TransformStamped sttrans;

      geometry_msgs::Transform t1;


      t1.translation.x = transform.getOrigin().x();
      t1.translation.y = transform.getOrigin().y();
      t1.translation.z = transform.getOrigin().z();
      t1.rotation.w = transform.getRotation().w();
      t1.rotation.x = transform.getRotation().x();
      t1.rotation.y = transform.getRotation().y();
      t1.rotation.z = transform.getRotation().z();

      sttrans.transform = t1;

      sttrans.header.stamp = currtime;

      gcop_comm::CurrPose posemsg;

      posemsg.pose = sttrans;



//joint properties
      ros::ServiceClient JointClient = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
      basewheels.request.joint_name = WheelName1;

      JointClient.call(basewheels);


     /* float curr_vel_wheels = basewheels.response.rate[0]; //speed of backwheels

      lin_vel = curr_vel_wheels *.05;

 basewheels.request.joint_name = WheelName1;

      JointClient.call(basewheels);*/


      float curr_vel_wheels1 = basewheels.response.rate[0]; //speed of backwheels

      float lin_vel1 = curr_vel_wheels1 *0.05;



      basewheels.request.joint_name = WheelName2;

      JointClient.call(basewheels);


      float curr_vel_wheels2 = basewheels.response.rate[0]; //speed of backwheels

      float lin_vel2 = curr_vel_wheels2 *0.05;


      float lin_vel3 = (lin_vel1+lin_vel2)/2;

      
      posemsg.velocity = lin_vel3;

      pose_pub.publish(posemsg); //publish pose and velocity


  



      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
