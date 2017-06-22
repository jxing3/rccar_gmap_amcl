#include "std_msgs/String.h"
//#include "gazebo/msgs/MessageTypes.hh"

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


float lin_vel;

void retclosestpoint(const nav_msgs::Path::ConstPtr& msg) {

    double closestdistance = sqrt(pow((msg->poses[closestpoint].pose.position.x-curr_pose.position.x),2) + pow((msg->poses[closestpoint].pose.position.y-curr_pose.position.y),2));

    int n = closestpoint;

    for(int i = n; i< n+10; i++) {
        double newclosest = sqrt(pow((msg->poses[i].pose.position.x-curr_pose.position.x),2) + pow((msg->poses[i].pose.position.y-curr_pose.position.y),2));
        if(newclosest<closestdistance) {
             closestpoint = i;
        }
    }

    return;

}

void chatterCallback(const nav_msgs::Path::ConstPtr& msg) {

 /* if(curr_pose NULL) {
      return;
  }*/

/*
  double prevdistance = 0;
  for(int i =0; i<msg->poses.size(); i++) {
     double currdistance = sqrt(pow((msg->poses[i].pose.position.x-curr_pose.position.x),2) + pow((msg->poses[i].pose.position.y-curr_pose.position.y),2));
     
     if(currdistance > 2 && currdistance>prevdistance) {
  */
         retclosestpoint(msg);
         int nextpos = closestpoint+5;

         geometry_msgs::Point e;

         e.x = msg->poses[nextpos].pose.position.x;
         e.y = msg->poses[nextpos].pose.position.y;
         e.z = 0;

         double deltx = msg->poses[nextpos+1].pose.position.x -msg->poses[nextpos-1].pose.position.x;
         double delty = msg->poses[nextpos+1].pose.position.y -msg->poses[nextpos-1].pose.position.y;

         double yaw = atan (delty/deltx);

         sendpose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);

         sendpose.pose.position = e;

         return;


     //}

    // prevdistance = currdistance;


  //}

   
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "destsend");

  gazebo_msgs::GetModelState model;

  ros::NodeHandle n;

  ros::Rate loop_rate(100);

 

  ros::Subscriber vel_sub = n.subscribe<nav_msgs::Path>("dsl/path", 1, chatterCallback);

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/target", 10);

  tf::TransformListener listener;

   while (ros::ok())
   {

       tf::StampedTransform transform;
       tf::StampedTransform transform1;
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

     pose_pub.publish(sendpose);



      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
