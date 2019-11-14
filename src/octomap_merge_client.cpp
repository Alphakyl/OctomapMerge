#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapMerge.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "octomap_merge_client");
  ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<octomap_merge::OctomapMerge>("octomap_merge");
  
  octomap_merge::OctomapMerge srv;
	srv.request.robot_map = argv[1];
  srv.request.other_map = argv[2];
  srv.request.pose_diff = argv[3];

  if(client.call(srv)
  {
  }
  else
  {
		ROS_ERROR("Failed to call service octomap_merge")
		return -1;
  }
  return 0;
}
