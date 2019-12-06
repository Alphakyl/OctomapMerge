#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapArray.h"
#include "octomap_merge.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud_xyz;
typedef pcl::PCLPointCloud2 PointCloud;

	
// Constructor
OctomapMerge::OctomapMerge()
{
}// end Constructor
 
// Destructor
OctomapMerge::~OctomapMerge()
{
}// end Destructor

//Callbacks
void OctomapMerge::callback_myMap(const octomap_msgs::OctomapConstPtr& msg)
{
    myMap = *msg;
	myMapNew = true;
} // end callback_myMap();

void OctomapMerge::callback_neighborMaps(const octomap_merge::OctomapArrayConstPtr& msg){
    neighbors = *msg;
	otherMapsNew = true;
} // end callback_neighborMaps();
 
int OctomapMerge::merge()
{
	// Convert my map to PCL
	// For each map in table
    for(int i = 0; i++; i<neighbors.num_octomaps)
	{
		// Convert this map to PCL
		// pcl_merge
	} // endfor
} // end merge();
	
void OctomapMerge::octomap_to_pcl(octomap_msgs::Octomap& map)
{
    octomap::OcTree* myTree = new octomap::OcTree(map.resolution);
    myTree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(map);
	double size, value;
	float lower_corner[3];
	int idk, depth, width;
    int lowest_depth = (int)myTree->getTreeDepth();
	int count = 0;
	int freeCount = 0;
	int occCount = 0;
    float voxel_size = (float)myTree->getResolution();

	PointT point;
	PointCloud_xyz::Ptr freeCells(new PointCloud_xyz);

    for(octomap::OcTree::leaf_iterator it = myTree->begin_leafs(), end=myTree->end_leafs(); it!=end; ++it)
	{
		depth = (int)it.getDepth();
		point.x = (float)it.getX();
		point.y = (float)it.getY();
		point.z = (float)it.getZ();
		size = it.getSize();

		if (depth == lowest_depth)
		{
			if (value)
 			{
				freeCells->points.push_back(point);
			} else {
				width = (int)std::pow(2.0, (double)(lowest_depth-depth));
				lower_corner[0] = point.x - size/2.0 + voxel_size/2.0;
				lower_corner[1] = point.y - size/2.0 + voxel_size/2.0;
				lower_corner[2] = point.z - size/2.0 + voxel_size/2.0;
				for (int i = 0; i < width; i++)
				{
					point.x = lower_corner[0] + i*voxel_size;
					for (int j = 0; j < width; j++)
					{
						point.y = lower_corner[1] + j*voxel_size;
						for (int k = 0; k < width; k++)
						{
							point.z = lower_corner[2] + k*voxel_size;
							if (value)
							{
								freeCells->points.push_back(point);
							} // endif
						} // endfor
					} // endfor
				} // endfor
			}  // endelseif
		} // endif
	} // endfor
	pcl::toROSMsg(*freeCells, freeCellsMsg);
    delete myTree;
} // endoctomap_to_pcl();


int main(int argc, char **argv)
{
	ros::init(argc, argv ,"octomap_merge");
  ros::NodeHandle nh;
  int rate;
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate, int(1));
  OctomapMerge *octomap_merger = new OctomapMerge();

  ros::Subscriber sub_mymap = nh.subscribe("octomap_binary", 100, &OctomapMerge::callback_myMap, octomap_merger);
  ros::Subscriber sub_neighbors = nh.subscribe("neighbors", 100, &OctomapMerge::callback_neighborMaps, octomap_merger);

  ros::Rate r(rate);
  while(nh.ok())
  {
    ros::spinOnce();
    if(octomap_merger->myMapNew && octomap_merger->otherMapsNew)
    {
			octomap_merger->myMapNew = false;
      octomap_merger->otherMapsNew = false;
      octomap_merger->merge();
    }  
    r.sleep();
  }
  return 0;
}
