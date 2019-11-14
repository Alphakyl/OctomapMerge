#ifndef OCTOMAP_MERGE_H_
#define OCTOMAP_MERGE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapMerge.h"

class OctomapMerge
{
/********************************************************************/
/* Public Variables and Methods */
/********************************************************************/
public:

  // Constructor
  OctomapMerge();
  // Destructor
  ~OctomapMerge();
  // Callbacks
	void callback_myMap(const octomap_msgs::Octomap::ConstPtr& msg);
  void callback_otherMapAndTF(const octomap_merge::OctomapMerge::ConstPtr& msg);
  void callback_merge(const std_msgs::Bool::ConstPtr& msg);
  // Publishers
  void publish_merged(ros::Publisher *merged_map);
  // Public Methods
  void merge();
  // Variables
  octomap_msgs::Octomap myMap;
  octomap_msgs::Octomap otherMap;
  nav_msgs::Odometry transform;

  //Refresh Vars
	bool myMapNew;
  bool otherMapNew;  
/*******************************************************************/
/* Private Variables and Methods */
/*******************************************************************/
private:

}; //end class OctomapMerge

#endif
