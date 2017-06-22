#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"
#include "ros/ros.h"
#include "gcop_comm/CtrlTraj.h"
//#include "gcop_comm/CurrPose.h"
#include "nav_msgs/Path.h"
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "rc_car/nav_msg.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sstream>

/**
 * GCOP integration with Gazebo
 */


geometry_msgs::Pose curr_pose;
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseStamped sendpose;
int closestpoint = 0;

rc_car::nav_msg getnav;

float lin_vel;

void retclosestpoint(const nav_msgs::Path msg) {

    double closestdistance = sqrt(pow((msg.poses[closestpoint].pose.position.x-curr_pose.position.x),2) + pow((msg.poses[closestpoint].pose.position.y-curr_pose.position.y),2));

    int n = closestpoint;

    for(int i = n; i< n+10; i++) {
        double newclosest = sqrt(pow((msg.poses[i].pose.position.x-curr_pose.position.x),2) + pow((msg.poses[i].pose.position.y-curr_pose.position.y),2));
        if(newclosest<closestdistance) {
             closestpoint = i;
        }
    }
    std::cout<<"This is the closest distance "<<closestdistance<<std::endl;
    std::cout<<"This is the closest point "<<closestpoint<<std::endl;

    return;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "destsend");

  gazebo_msgs::GetModelState model;

  ros::NodeHandle n;
  ros::ServiceClient client;

  ros::Rate loop_rate(100);


  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/target", 10);

  tf::TransformListener listener;

   while (ros::ok())
   {

       tf::StampedTransform transform;

    try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time::now(), transform);
    }
    catch (tf::TransformException &ex) {
      //ROS_WARN("%s",ex.what());
      continue;
    }



     curr_pose.position.x = transform.getOrigin().x();
     curr_pose.position.y = transform.getOrigin().y();
     curr_pose.position.z = 0;
     curr_pose.orientation.w = transform.getRotation().w();
     curr_pose.orientation.x = transform.getRotation().x();
     curr_pose.orientation.y = transform.getRotation().y();
     curr_pose.orientation.z = transform.getRotation().z();



      client = n.serviceClient<rc_car::nav_msg>("path_srv");


     client.call(getnav);

     retclosestpoint(getnav.response.path);

 	int nextpos = closestpoint+5;
 	/*for(int i = closestpoint; i< closestpoint+10; i++) {
             double nextpt = sqrt(pow((getnav.response.path.poses[i].pose.position.x-curr_pose.position.x),2) + pow((getnav.response.path.poses[i].pose.position.y-curr_pose.position.y),2));
        if(nextpt>5) {
		nextpos = i;
	     break;
        }
    }*/
    

     geometry_msgs::Point e;

     e.x = getnav.response.path.poses[nextpos].pose.position.x;
     e.y = getnav.response.path.poses[nextpos].pose.position.y;
     e.z = 0;

    double deltx = getnav.response.path.poses[nextpos+3].pose.position.x -getnav.response.path.poses[nextpos-3].pose.position.x;
    double delty = getnav.response.path.poses[nextpos+3].pose.position.y -getnav.response.path.poses[nextpos-3].pose.position.y;

    double yaw = atan2 (delty, deltx);

    std::cout<<"The following is change in x: "<<deltx<<std::endl;

    std::cout<<"The following is the change in y: "<<delty<<std::endl;

    std::cout<<yaw<<std::endl;

    sendpose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);

    sendpose.pose.position = e;

    pose_pub.publish(sendpose);



      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
