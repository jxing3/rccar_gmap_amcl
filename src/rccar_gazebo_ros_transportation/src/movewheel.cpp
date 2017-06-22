#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo_core.hh>
#include <gazebo/physics/LinkState.hh>
#include <gazebo/physics/State.hh>
#include "gazebo/physics/CollisionState.hh"
#include "gazebo/math/Pose.hh"



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char** argv){
    ros::init(argc, argv, "my_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::ServiceClient client;

    std::string leftWheelName   = (std::string)"Unicycle::base_to_backwheels";

    geometry_msgs::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 6.28/10;
    twist.angular.z = 0.0;

    gazebo::physics::LinkState localLinkState;
    gazebo::GetLinkState getLinkState;
    gazebo::SetLinkState setLinkState;

    while (ros::ok()) {

        client = n.ServiceClient<gazebo::physics::GetLinkState>("/gazebo/get_link_state");
        getLinkState.request.link_name = leftWheelName;
        getLinkState.request.reference_frame = leftWheelName.c_str();
        client.call(getLinkState);

        localLinkState = getLinkState.response.link_state;
        localLinkState.twist = twist;
        localLinkState.reference_frame = leftWheelName.c_str();

        client = n.ServiceClient<gazebo::physics::SetLinkState>("/gazebo/set_link_state");
        setLinkState.request.link_state = localLinkState;
        client.call(setLinkState);

        ros::spinOnce();
        loop_rate.sleep();
   }
    return 0;
}
