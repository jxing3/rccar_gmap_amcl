#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo/msgs/MessageTypes.hh"
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "movewheel");

 std::string leftWheelName   = (std::string)"Unicycle::backwheels";

 std::string steering1   = (std::string)"Unicycle::steeringblock1";

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  geometry_msgs::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.linear.z  = 0.0;
    twist.angular.x = 0;
    twist.angular.y = 6.28; //6.28/10;
    twist.angular.z = 0;

  geometry_msgs::Twist twist1;

    twist1.linear.x  = 0.0;
    twist1.linear.y  = 0.0;
    twist1.linear.z  = 0.0;
    twist1.angular.x = 0;
    twist1.angular.y = 0; //6.28/10;
    twist1.angular.z = 0;

  gazebo_msgs::GetLinkState getLinkState;
  gazebo_msgs::LinkState localLinkState;
   gazebo_msgs::SetLinkState setLinkState;

  gazebo_msgs::GetLinkState getLinkState1;
   gazebo_msgs::LinkState localLinkState1;
   gazebo_msgs::SetLinkState setLinkState1;

   while (ros::ok())
   {

        ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        getLinkState.request.link_name = leftWheelName;
        getLinkState.request.reference_frame = leftWheelName.c_str();
        client.call(getLinkState);


      localLinkState = getLinkState.response.link_state;
        localLinkState.twist = twist;
        localLinkState.reference_frame = leftWheelName.c_str();

   ros::ServiceClient sls_client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

      setLinkState.request.link_state = localLinkState;
      sls_client.call(setLinkState);

        getLinkState1.request.link_name = steering1;
        getLinkState1.request.reference_frame = steering1.c_str();
        client.call(getLinkState1);


      localLinkState1 = getLinkState1.response.link_state;
        localLinkState1.twist = twist1;
        localLinkState1.reference_frame = steering1.c_str();

      setLinkState1.request.link_state = localLinkState1;
      sls_client.call(setLinkState1);
  
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
