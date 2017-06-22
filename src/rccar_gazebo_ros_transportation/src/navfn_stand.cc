#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

std::vector<geometry_msgs::PoseStamped> path;

int main (int argc, char** argv)
{

  ros::init(argc, argv, "global_planner");

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);

  navfn::NavfnROS navfn;
  navfn.initialize("my_navfn_planner", &costmap);

  geometry_msgs::PoseStamped start;

  ros::Time currtime = ros::Time::now();
  start.header.stamp = currtime;
  start.pose.position.x = 0;
  start.pose.position.y = 0;
  start.pose.position.z = 0;

  geometry_msgs::PoseStamped end;

  end.header.stamp = currtime + ros::Duration(20);
  end.pose.position.x = 10;
  end.pose.position.y = 20;
  end.pose.position.z = 0;

  navfn.makePlan(start, end, 2, path);

  navfn.publishPlan(path, 255, 255, 255, 255);
  ros::spin();

  return 0;
}
