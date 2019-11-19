#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapArray.h"
#include "octomap_merge.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class OctomapMerge
{
	
  // Constructor
  OctomapMerge::OctomapMerge()
	{
	}// end Constructor
 
  // Destructor
	OctomapMerge::~OctomapMerger()
  {
	}// end Destructor

  //Callbacks
  void callback_myMap(const octomap_msgs::Octomap::ConstPtr& msg)
  {
    myMap = msg.Octomap;
    myMapNew = true;
  }

  void callback_neighborMaps(const octomap_merge::OctomapArray::ConstPtr& msg){
    neighbors = msg;
    otherMapsNew = true;
  }
 
  int merge()
  {
    // Convert my map to PCL
    // For each map in table
    for(uint8 i = 0, i++, i<neighbors.num_neighbors)
    {
       // Convert this map to PCL
       // pcl_merge
    } 
  } 

}


int main(int argc, char **argv)
{
	ros::init(argc, argv ,"octomap_merge");
  ros::NodeHandle nh;
  int rate;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate, int(1));
  OctomapMerge *octomap_merger = new OctomapMerge();

  ros::Subscriber sub_mymap = nh.subscribe(octomap_msgs::Octomap, 100, &OctomapMerger::callback_myMap, octomap_merger);
  ros::Subscriber sub_neighbors = nh.subscribe(octomap_merge::OctomapArray, 100, &OctomapMerger::callback_neighborMaps, octomap_merger);

  ros::Rate r(rate);
  while(nh.ok())
  {
    ros::SpinOnce();
    if(octomap_merger::myMapNew && octomap_merger::otherMapsNew)
    {
			octomap_merger::myMapNew = false;
      octomap_merger::otherMapsNew = false;
      octomap_merger::merge();
    }  
    r.sleep();
  }
  return 0;
}
