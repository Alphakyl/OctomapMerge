#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapAndPoseDiffOdom.h"
#include "octomap_merge.h"

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

  void callback_otherMapAndTF(const octomap_merge::OctomapAndPoseDiffOdom::ConstPtr& msg){
    otherMap = msg.map;
    transform = msg.pose_diff;
    otherMapNew = true;
  }
  
	void OctomapMerge::publishMerged(ros::Publisher *merged_map)
  {
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

	//ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("merged_map", 5);

  ros::Subscriber sub_mymap = nh.subscribe(octomap_msgs::Octomap, 100, &OctomapMerger::callback_myMap, octomap_merger);
  ros::Subscriber sub_othermap = nh.subscribe();

  ros::Rate r(rate);
  while(nh.ok())
  {
    ros::SpinOnce();
    if(octomap_merger::myMapNew && octomap_merger::otherMapNew)
    {
			octomap_merger::myMapNew = false;
      octomap_merger::otherMapNew = false;
      octomap_merger::merge();
    }  
    r.sleep();
  }
  return 0;
}
