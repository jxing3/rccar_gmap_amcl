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

double pos_y;
void callback(gazebo_msgs::LinkStates msgs_ls)
{
   double pos_y = msgs_ls.pose[1].position.y
   pos_y = pos_y + 1;
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "my_node");
   ros::NodeHandle n;
   ros::Subscriber ls_sub = n.subscribe("/gazebo/link_states", 10, callback);
   ros::ServiceClient sls_client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
   gazebo_msgs::SetLinkState setLinkState;
   while (ros::ok())
   {
      setLinkState.request.link_state.position.y = pos_y;
      sls_client.call(setLinkState);
  }

}
