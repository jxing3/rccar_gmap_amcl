#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo/msgs/MessageTypes.hh"
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ApplyJointEffort.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "movewheel");

 std::string WheelName   = (std::string)"base_to_backwheels";

 std::string steering1   = (std::string)"Unicycle::steeringblock1";

 std::string steering2   = (std::string)"Unicycle::steeringblock2";

ros::Time start(0);
ros::Duration duration(0.01);


  ros::NodeHandle n;

  ros::Rate loop_rate(10);

geometry_msgs::Pose start_pose1;
   start_pose1.position.x = 0.3;
   start_pose1.position.y = -0.12;
   start_pose1.position.z = 0;
   start_pose1.orientation.x = 0;
   start_pose1.orientation.y = 0;
   start_pose1.orientation.z = 0;
   start_pose1.orientation.w = 1;

geometry_msgs::Pose start_pose;
   start_pose.position.x = 0.3;
   start_pose.position.y = 0.12;
   start_pose.position.z = 0;
   start_pose.orientation.x = 0;
   start_pose.orientation.y = 0;
   start_pose.orientation.z = 0;
   start_pose.orientation.w = 1;

  geometry_msgs::Twist twist;
    twist.linear.x  = 0;
    twist.linear.y  = 0;
    twist.linear.z  = 0;
    twist.angular.x = 0;
    twist.angular.y = 0; //6.28/10;
    twist.angular.z = 0;

   gazebo_msgs::ApplyJointEffort applyJointEffort;


  gazebo_msgs::GetLinkState getLinkState;
  gazebo_msgs::LinkState localLinkState;
   gazebo_msgs::SetLinkState setLinkState;

  gazebo_msgs::GetLinkState getLinkState1;
  gazebo_msgs::LinkState localLinkState1;
   gazebo_msgs::SetLinkState setLinkState1;


   while (ros::ok())
   {

        ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
        applyJointEffort.request.joint_name = WheelName;
        applyJointEffort.request.effort = 0.5;
        applyJointEffort.request.start_time= start;
        applyJointEffort.request.duration = duration;
        client.call(applyJointEffort);


/*
         ros::ServiceClient client1 = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        getLinkState.request.link_name = steering1;
        getLinkState.request.reference_frame = steering1.c_str();
        client1.call(getLinkState);*/

        localLinkState.link_name = steering1;
        localLinkState.pose = start_pose1;
        localLinkState.twist = twist;
        localLinkState.reference_frame = "Unicycle::carbody";

   ros::ServiceClient sls_client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

      setLinkState.request.link_state = localLinkState;
      sls_client.call(setLinkState);

        /* ros::ServiceClient client2 = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        getLinkState1.request.link_name = steering2;
        getLinkState1.request.reference_frame = steering2.c_str();
        client2.call(getLinkState1);

*/
        localLinkState1.link_name = steering2;
        localLinkState1.pose = start_pose;
        localLinkState1.twist = twist;
        localLinkState1.reference_frame = "Unicycle::carbody";

  ros::ServiceClient sls_client2 = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

      setLinkState1.request.link_state = localLinkState1;
      sls_client2.call(setLinkState1);
  
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
