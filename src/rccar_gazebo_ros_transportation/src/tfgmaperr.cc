/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/GetJointProperties.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include <math.h>

#include <iostream>

gazebo::transport::PublisherPtr gz_vel_cmd_pub;
ros::Publisher ros_odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
int ros_odom_pub_seq;
std::string ros_odom_frame;
std::string ros_child_frame;
std::string gz_model_name;
double ros_odom_tf_future_date;
ros::Time prevtime(0);
gazebo_msgs::GetJointProperties basewheels;

ros::ServiceClient JointClient;


float x = 0;
float y = 0;

float prevwheelpos = 0;
float prevx1 = 0;
float prevy1 = 0;


tf::Quaternion prev_right_wheel_car = tf::createQuaternionFromYaw(0);;
tf::Quaternion prev_left_wheel_car = tf::createQuaternionFromYaw(0);;


void gz_odom_Callback(ConstPosesStampedPtr &msg_in)
{ 
  //std::cout << "gz_odom" << msg_in->DebugString() << std::endl;

  float x1  = 0;
  float y1  = 0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

  
  tf::Quaternion car_body;

  tf::Quaternion right_wheel_car;
  tf::Quaternion left_wheel_car;

  ros::Time currtime(0);

 /*for (int i = 0; i < msg_in->pose_size(); i++) {
     std::cout<<msg_in->pose(i).name()<<std::endl;
   
  }*/
  
  for (int i = 0; i < msg_in->pose_size(); i++)  {
    
    if(msg_in->pose(i).name() == gz_model_name)
    {
      //std::cout << msg_in->pose(i).DebugString() << std::endl;
      x1 = msg_in->pose(i).position().x();
      y1 = msg_in->pose(i).position().y();
      car_body = tf::Quaternion (msg_in->pose(i).orientation().x(), msg_in->pose(i).orientation().y(),msg_in->pose(i).orientation().z(), msg_in->pose(i).orientation().w());
      tf::quaternionTFToMsg(car_body,odom_quat);
      currtime.sec = msg_in->time().sec();
      currtime.nsec = msg_in->time().nsec();
    }    

    if(msg_in->pose(i).name() == "Unicycle::rear_right_wheel")
    {
      //std::cout << msg_in->pose(i).DebugString() << std::endl;

      right_wheel_car = tf::Quaternion (msg_in->pose(i).orientation().x(), msg_in->pose(i).orientation().y(),msg_in->pose(i).orientation().z(), msg_in->pose(i).orientation().w());


      currtime.sec = msg_in->time().sec();
      currtime.nsec = msg_in->time().nsec();
    } 

  if(msg_in->pose(i).name() == "Unicycle::rear_left_wheel")
    {
      //std::cout << msg_in->pose(i).DebugString() << std::endl;

      left_wheel_car = tf::Quaternion (msg_in->pose(i).orientation().x(), msg_in->pose(i).orientation().y(),msg_in->pose(i).orientation().z(), msg_in->pose(i).orientation().w());


      currtime.sec = msg_in->time().sec();
      currtime.nsec = msg_in->time().nsec();
    } 

   }




    tf::Quaternion delta_right = right_wheel_car.inverse()* prev_right_wheel_car;
    tf::Quaternion delta_left = left_wheel_car.inverse()* prev_left_wheel_car;


    double right_roll, right_pitch, right_yaw;

    tf::Matrix3x3(delta_right).getRPY(right_roll, right_pitch, right_yaw);

    double left_roll, left_pitch, left_yaw;

    tf::Matrix3x3(delta_left).getRPY(left_roll, left_pitch, left_yaw);

     std::cout<< "right rpy: "<<right_roll <<" "<< right_pitch<<" "<< right_yaw<< std::endl;

     std::cout<< "left rpy: "<<left_roll <<" "<< left_pitch<<" "<< left_yaw<< std::endl;

    double car_roll, car_pitch, car_yaw;
    tf::Matrix3x3(car_body).getRPY(car_roll, car_pitch, car_yaw);

    double deltwheelpos = (right_yaw + left_yaw)/2.0;



     
    std::cout<< "x, y "<<x <<" "<< y <<" yaw: "<< car_yaw<< std::endl;
    
    if(prevtime.toSec() != 0) {

      x+= deltwheelpos*.05*cos(car_yaw);

      y+= deltwheelpos*.05*sin(car_yaw);


  } else { 

     x = x1;
     y = y1;
  }

    ROS_INFO("act x: [%f]  act y: [%f]  odom x: [%f] odom y: [%f] ", x1, y1, x, y);


    prev_right_wheel_car = right_wheel_car;

    prev_left_wheel_car = left_wheel_car;

  prevtime = currtime;


/*
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(odom_quat, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);



      basewheels.request.joint_name = "rear_left_wheel_joint";

      JointClient.call(basewheels);

      float curr_vel_wheel1 = basewheels.response.rate[0]; //speed of backwheels

      float wheel1_pos = basewheels.response.position[0];



      basewheels.request.joint_name = "rear_right_wheel_joint";

      JointClient.call(basewheels);

      float curr_vel_wheel2 = basewheels.response.rate[0]; //speed of backwheels

      float wheel2_pos = basewheels.response.position[0];

      float lin_vel = (curr_vel_wheel1 + curr_vel_wheel2)/2 *.05;

      float currwheelpos = (wheel2_pos + wheel1_pos)/2;
 
      float deltwheelpos = currwheelpos-prevwheelpos;


  ros::Duration d = currtime - prevtime;

  if(prevtime.toSec() != 0) {
      //x+= lin_vel*cos(yaw)*d.toSec();

      //y+= lin_vel*sin(yaw)*d.toSec();

      x+= deltwheelpos*.05*cos(yaw);

      y+= deltwheelpos*.05*sin(yaw);


  } else { 

     x = x1;
     y = y1;
  }

  prevtime = currtime;

  prevwheelpos = currwheelpos;


 // ROS_INFO("time: [%f]  errorx: [%f]  errory: [%f]", d.toSec(), x1-x, y1-y);

  ROS_INFO("deltx1: [%f]  delty1: [%f]  odomx1: [%f] odomy1: [%f] yaw: [%f]", x1-prevx1, y1-prevy1, deltwheelpos*.05*cos(yaw), deltwheelpos*.05*sin(yaw), yaw);

  prevx1 = x1;

  prevy1 = y1;*/




  geometry_msgs::TransformStamped odom_trans;
  ros::Duration future_date(ros_odom_tf_future_date);
  odom_trans.header.stamp             = ros::Time::now() + future_date;
  odom_trans.header.frame_id          = ros_odom_frame;
  odom_trans.child_frame_id           = ros_child_frame;
  odom_trans.transform.translation.x  = x;
  odom_trans.transform.translation.y  = y;
  odom_trans.transform.translation.z  = 0.0;
  odom_trans.transform.rotation       = odom_quat;
  odom_broadcaster->sendTransform(odom_trans);

}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS
  ros::init(argc, argv, "Gmapping_tf");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  n_.param<std::string>("ros_odom_frame", ros_odom_frame,   "/odom");
  n_.param<std::string>("ros_child_frame", ros_child_frame, "/base_link");
  n_.param<double>("ros_odom_tf_future_date", ros_odom_tf_future_date,  0);
  
  n_.param<std::string>("gz_model_name", gz_model_name,   "Unicycle");
  std::string gz_pose_topic;
  n_.param<std::string>("gz_pose_topic", gz_pose_topic, "~/pose/info");
  
  odom_broadcaster = new tf::TransformBroadcaster;
  ros_odom_pub_seq = 0;
    
  // When launched from a launch file we need to give Gazebo time to load
  ros::Duration(5.0).sleep();
  
  // Initialize Gazebo
  gazebo::transport::init();
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::run();

  //JointClient = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    
  // Subscribers
  gazebo::transport::SubscriberPtr gz_odom_sub = node->Subscribe(gz_pose_topic, gz_odom_Callback);
  
  // Spin
  ros::spin();

  // Shutdown
  gazebo::transport::fini();
}

