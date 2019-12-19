#ifndef OCTOMAP_MERGE_H_
#define OCTOMAP_MERGE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapArray.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud_xyz;
typedef pcl::PCLPointCloud2 PointCloud;

class OctomapMerge
{
/* Public Variables and Methods */
public:
  // Constructor
  OctomapMerge(ros::NodeHandle* nodehandle);
  // Destructor
  ~OctomapMerge();
  // Callbacks
  void callback_myMap(const octomap_msgs::Octomap::ConstPtr& msg);
  void callback_neighborMaps(const octomap_merge::OctomapArrayConstPtr &msg);
  // Public Methods
  void merge();
  // Variables
  bool myMapNew;
  bool otherMapsNew;  

/* Private Variables and Methods */
private:
  ros::NodeHandle nh_;

  octomap_msgs::Octomap myMap;
  octomap_merge::OctomapArray neighbors;
  octomap_msgs::Octomap * maptoconvertptr;
  octomap_msgs::Octomap maptoconvert;

  ros::Subscriber sub_mymap;
  ros::Subscriber sub_neighbors;

  ros::Publisher pub_merged;

  void octomap_to_pcl(octomap_msgs::Octomap * map, sensor_msgs::PointCloud2Ptr occupiedCellsMsg);
  void initializeSubscribers();
  void initializePublishers();
  int pub_times;

}; //end class OctomapMerge

#endif
