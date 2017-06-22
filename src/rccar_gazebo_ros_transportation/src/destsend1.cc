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
geometry_msgs::Pose closest_point;
const double des_dist = 2;

rc_car::nav_msg getnav;

float lin_vel;

geometry_msgs::Pose GetClosestPoint(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose P)
{
    geometry_msgs::Pose AP;
    AP.position.x =  P.position.x-A.position.x;
    AP.position.y = P.position.y-A.position.y;

    geometry_msgs::Pose AB;
    AB.position.x = B.position.x-A.position.x;
    AB.position.y = B.position.y-A.position.y;
    float ab2 = AB.position.x*AB.position.x + AB.position.y*AB.position.y;
    float ap_ab = AP.position.x*AB.position.x + AP.position.y*AB.position.y;
    float t = ap_ab / ab2;

         if (t < 0.0) {
           t = 0.0;
         }
         else if (t > 1.0) {
           t = 1.0;
          }
    geometry_msgs::Pose final;
    final.position.x = A.position.x + AB.position.x * t;
    final.position.y = A.position.y+AB.position.y*t;
    return final;
}


geometry_msgs::Pose retclosestpoint(const nav_msgs::Path msg) {

    /*double closestdistance = sqrt(pow((msg.poses[closestpoint].pose.position.x-curr_pose.position.x),2) + pow((msg.poses[closestpoint].pose.position.y-curr_pose.position.y),2));

    int n = closestpoint;

    for(int i = n; i< n+10; i++) {
        double newclosest = sqrt(pow((msg.poses[i].pose.position.x-curr_pose.position.x),2) + pow((msg.poses[i].pose.position.y-curr_pose.position.y),2));
        if(newclosest<closestdistance) {
             closestpoint = i;
        }
    }*/

   geometry_msgs::Pose finalpose = GetClosestPoint(msg.poses[0].pose,msg.poses[1].pose, curr_pose);
   closestpoint = 1;

   for (int i=1; i<msg.poses.size()-1; i++) {

       geometry_msgs::Pose pose1 = GetClosestPoint(msg.poses[i].pose,msg.poses[i+1].pose, curr_pose);
       float distance1 = sqrt(pow(curr_pose.position.x-pose1.position.x,2) + pow(curr_pose.position.y-pose1.position.y,2));
       float distancefin = sqrt(pow(curr_pose.position.x-finalpose.position.x,2) + pow(curr_pose.position.y-finalpose.position.y,2));
       if(distance1<distancefin) {
           finalpose = pose1;
           closestpoint = i+1;
       }
   }

   /* geometry_msgs::Pose pose1 = GetClosestPoint(msg.poses[closestpoint].pose,msg.poses[closestpoint+1].pose, curr_pose);

    geometry_msgs::Pose pose2 = GetClosestPoint(msg.poses[closestpoint-1].pose,msg.poses[closestpoint].pose, curr_pose);

    float distance1 = sqrt(pow(curr_pose.position.x-pose1.position.x,2) + pow(curr_pose.position.y-pose1.position.y,2));
    float distance2 = sqrt(pow(curr_pose.position.x-pose2.position.x,2) + pow(curr_pose.position.y-pose2.position.y,2));
   // std::cout<<"This is the closest distance "<<closestdistance<<std::endl;
    //std::cout<<"This is the closest point "<<closestpoint<<std::endl;

    if(distance1<distance2) {
        return pose1;
    }*/
    return finalpose;

}

void destPoint(const nav_msgs::Path p, const geometry_msgs::Pose closepose) {


	double dist = sqrt(pow(closepose.position.x-p.poses[closestpoint].pose.position.x,2) + pow(closepose.position.y-p.poses[closestpoint].pose.position.y,2));


	if(dist>des_dist) {
		double mx =  p.poses[closestpoint].pose.position.x - closepose.position.x;
		double my = p.poses[closestpoint].pose.position.y - closepose.position.y;
		double totdist = sqrt(pow(mx,2)+pow(my,2));
		double t = des_dist/totdist;
		double y = closepose.position.y +my*t;
		double x = closepose.position.x +mx*t;
		sendpose.pose.position.x = x;
		sendpose.pose.position.y = y;
                double yaw = atan2 (my, mx);

                sendpose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);


		return;
        }

     double prevdist = 0;
     for(int i = closestpoint; i< p.poses.size(); i++) {
        if(dist>des_dist) {
		int prev = (i-1> 0) ? i-1 : 0;

		double mx =  p.poses[i].pose.position.x - p.poses[prev].pose.position.x;
		double my = p.poses[i].pose.position.y - p.poses[prev].pose.position.y;
		double totdist = sqrt(pow(mx,2)+pow(my,2));
		double extradist = des_dist-prevdist;
		double t = extradist/totdist;
		double y = p.poses[prev].pose.position.y +my*t;
		double x = p.poses[prev].pose.position.x +mx*t;
		sendpose.pose.position.x = x;
		sendpose.pose.position.y = y;
                double yaw = atan2 (my, mx);

                sendpose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);


		return;
        }
	prevdist = dist;

        dist += sqrt(pow(p.poses[i+1].pose.position.x-p.poses[i].pose.position.x,2) + pow(p.poses[i+1].pose.position.y-p.poses[i].pose.position.y,2));
    }

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

     geometry_msgs::Pose closepose = retclosestpoint(getnav.response.path);

     std::cout<<"x: "<<closepose.position.x<<std::endl;

     std::cout<<"y: "<<closepose.position.y<<std::endl;

     nav_msgs::Path p = getnav.response.path;

 	//int nextpos = closestpoint+5;

     destPoint(p,closepose);
    

   /*  geometry_msgs::Point e;

     e.x = getnav.response.path.poses[nextpos].pose.position.x;
     e.y = getnav.response.path.poses[nextpos].pose.position.y;
     e.z = 0;

    
    std::cout<<"The following is change in x: "<<deltx<<std::endl;

    std::cout<<"The following is the change in y: "<<delty<<std::endl;

    std::cout<<yaw<<std::endl;

    sendpose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);

    sendpose.pose.position = e;*/

    pose_pub.publish(sendpose);



      ros::spinOnce();
      loop_rate.sleep();


  }

  return 0;
}
