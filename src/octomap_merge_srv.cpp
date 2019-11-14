#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapMerge.h"

bool merge(octomap_merge::OctomapMerge::Request & req,
						 octomap_merge::OctomapMerge::Response &res)
{
	ROS_INFO("Combining octomaps");
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv ,"octomap_merge_srv");
  ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("octomap_merge", merge);
  ROS_INFO("Ready to merge maps");
  ros::spin();

  return 0;
}
