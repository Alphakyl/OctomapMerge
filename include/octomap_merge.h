#ifndef OCTOMAP_MERGE_H_
#define OCTOMAP_MERGE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapArray.h"
#include "octomap_merge/OctomapMerge.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
  void callback_neighborMaps(const octomap_merge::OctomapMerge::ConstPtr& msg);
  // Publishers
  void publish_merged(ros::Publisher *merged_map);
  // Public Methods
  int merge();
  // Variables
  octomap_msgs::Octomap myMap;
  octomap_merge::OctomapArray neighbors;

  //Refresh Vars
	bool myMapNew;
  bool otherMapsNew;  
/*******************************************************************/
/* Private Variables and Methods */
/*******************************************************************/
private:

	void octomap_to_pcl(const octomap_msgs::Octomap::ConstPtr& map);
  void pcl_merge(); //(pcl_pointer? map1, pcl_pointer? map2)

}; //end class OctomapMerge

#endif
