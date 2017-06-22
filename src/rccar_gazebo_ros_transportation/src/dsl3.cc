#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <time.h>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include "rc_car/nav_msg.h"

#include "dsl/gridsearch3d.h"

#include "dsl/gridsearch.h"

#include "trimesh2/TriMesh.h"
#include "trimesh2/TriMesh_algo.h"
#include "trimesh2/Vec.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace dsl;
using namespace trimesh;

int height, width, xmin, ymin;

float resolution;

double *chmap;
double *inflmap;

bool subonce = true;
bool newpath = false;

int destX;
int destY;

int startX;
int startY;

nav_msgs::Path path;


void inflatecosts() {

   inflmap = new double[width*height];

   for(int i=0; i<width*height; i++) {

      inflmap[i] = chmap[i];

   }
   
   for (int i=0; i<width; i++) {
       for(int j=0; j<height; j++) {
          if(chmap[j*width + i] != 0) {
             // std::cout << "hi"<< std::endl;
            
            int inxmin = (i-15< 0 ) ? 0: i-15;
            int inymin = (j-15<0)? 0: j-15;
            int inxmax = (i+15> width)? width: i+15;
            int inymax = (j+15>height)? height: j+15;

            for(int k=inxmin; k<inxmax; k++) {
              inflmap[k+j*width] = 1000;

            }

            for(int l=inymin; l< inymax; l++) {
              inflmap[i+l*width] = 1000;
            }
          }

       }
   }

}

void rvizPosReg(const geometry_msgs::PoseStamped::ConstPtr &posnmsg) {


// Convert quaternion to RPY.

    destX = (posnmsg->pose.position.x - xmin)/resolution;
    destY = (posnmsg->pose.position.y - ymin)/resolution;
       
    tf::StampedTransform transform;

    tf::TransformListener listener;
    while(true) {

        try{
            listener.lookupTransform("/map", "/base_link",
                               ros::Time::now(), transform);
        }
        catch (tf::TransformException &ex) {
         //ROS_WARN("%s",ex.what());
          continue;
        }
        break;

    }

    startX = (transform.getOrigin().x()- xmin)/resolution;
    startY = (transform.getOrigin().y() - ymin)/resolution;


    newpath = true;


}

void chatterCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

  if(subonce) {

    width = msg->info.width;

    height = msg->info.height;

    xmin = msg->info.origin.position.x;

    ymin = msg->info.origin.position.y;

    resolution = msg ->info.resolution;

    chmap = new double[width*height];

    for(int i=0; i<width*height; i++) {

      if(msg->data[i] == 0) {
        chmap[i] = 0;
      } else {
         chmap[i] = 1000;
      }
    }

  inflatecosts();
  std::cout << msg->info.resolution << std::endl;
   
  }

  subonce = false;

}


bool pathsrv(rc_car::nav_msg::Request  &req,
         rc_car::nav_msg::Response &res)
{
  res.path = path;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

nav_msgs::Path dsl_path_to_ros_msg(GridPath dsl_path, float meterPerCell, float xmin, float ymin, float zmin)
{
  nav_msgs::Path msg;  
  
  msg.header.frame_id = "/map";
  msg.poses.resize(dsl_path.count);
  float offset = 1.0/(2/meterPerCell);
  for(int i = 0; i < dsl_path.count; i++)
  {
    msg.poses[i].pose.position.x = dsl_path.pos[2*i]*meterPerCell + offset + xmin;//0.5;
    msg.poses[i].pose.position.y = dsl_path.pos[2*i+1]*meterPerCell + offset + ymin;//0.5;
    msg.poses[i].pose.position.z = 0 + offset;//dsl_path.pos[3*i+2]/cellsPerMeter + offset + zmin;//0.5;
  }
  return msg; 
}

void save_map(const double* map, const char* filename)
{
  int i;
  char data[width*height*3];
  FILE* file = fopen(filename, "w");
  assert(file);
  fprintf(file, "P6\n%d %d 255\n", width, height);
  for (i = 0; i < width*height; i++) {
    if(map[i] <= 10) {
    data[i*3] = data[i*3+1] = data[i*3+2] = (char)(0);

    } else if(map[i] ==50) {
        data[i*3] = data[i*3+1] = data[i*3+2] = (char) map[i];
     }
     else {
       data[i*3] = data[i*3+1] = data[i*3+2] = (char)(255);
    }
  }
  assert(i == width*height);
  assert((int)fwrite(data, sizeof(char), i*3, file) == i*3);
  fclose(file);
}


int main(int argc, char **argv)
{

  bool runonce = true;

  // Publish to ROS
  ros::init(argc, argv, "dslviz");

  ros::NodeHandle n;

  ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 2, chatterCallback);

  ros::Subscriber rvizposn_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,rvizPosReg);

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/dsl/path", 1000);
/*
  ros::Publisher optpath_pub = n.advertise<nav_msgs::Path>("/dsl/optpath", 1000);
  nav_msgs::Path optpath_path = dsl_path_to_ros_msg(optpath,cellsPerMeter, xmin, ymin, zmin);
  */
  ros::Rate loop_rate(100);
  ros::ServiceServer service = n.advertiseService("path_srv", pathsrv);
  bool init = false;

  while(subonce) {    
     ros::spinOnce();
   }
 
  GridSearch gdsl(width, height, inflmap);
  std::cout << "Map initialized" << std::endl;

 while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    GridPath dslpath, optpath;


    if(newpath) {


  //Perform dsl gridsearch3D
       gdsl.SetStart(startX,startY);
       gdsl.SetGoal(destX,destY);
 
       gdsl.Plan(dslpath);
       gdsl.OptPath(dslpath, optpath);

        path = dsl_path_to_ros_msg(optpath, resolution, xmin, ymin, 0);
       newpath = false;

       // double *mapPath = new double[width*height];

 // char mapPath2[width*height];

  //memcpy(mapPath, chmap, width*height);

       /* for (int i =0;i< width*height; i++) {
         mapPath[i] = chmap[i];

        }*/

  //memcpy(mapPath2, chmap, width*height);*/

       /* for (int i = 0; i < dslpath.count; ++i) {
            printf("(%d,%d) ", dslpath.pos[2*i], dslpath.pos[2*i+1]);
            mapPath[dslpath.pos[2*i+1]*width + dslpath.pos[2*i]] = 50;
         }*/

       //save_map(chmap, "path1.ppm");
      //  save_map(mapPath, "path2.ppm");

        // save_map2(chmap, width, height, "map1.ppm");
        std::cout << "Done planning" << std::endl;
  }

  if(!newpath) {
     path_pub.publish(path);

  }
}
 

 // path_pub.publish(path);
/*  optpath_pub.publish(optpath_path);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "dsl"));
  vis_pub.publish( marker );
  //plane_vis_pub.publish( plane );
  occ_map_vis_pub.publish( occmap_viz );
    */
  //delete[] occupancy_map;
  return 0;
}
