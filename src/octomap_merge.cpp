#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
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
  } // end callback_myMap();

  void callback_neighborMaps(const octomap_merge::OctomapArray::ConstPtr& msg){
    neighbors = msg;
    otherMapsNew = true;
  } // end callback_neighborMaps();
 
  int merge()
  {
    // Convert my map to PCL
    // For each map in table
    for(uint8 i = 0, i++, i<neighbors.num_neighbors)
    {
       // Convert this map to PCL
       // pcl_merge
    } // endfor
  } // end merge();
	
	void octomap_to_pcl(const octomap_msgs::Octomap::ConstPtr& map)
  {
		octomap::Octree* myTree = new octomap::Octree(map->resolution);
    myTree = (octomap::Octree*)octomap_msgs::binaryMsgToMap(*map);
    double size, value;
    float lower_corner[3];
    int idk, depth, width;
    int lowest_depth = (int)mytree->getTreeDepth();
    int count = 0;
    int freeCount = 0;
    int occCount = 0;
    float voxel_size = (float)mytree->getResolution();
    
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr freeCells(new pcl::PointCloud<pcl::pointXYZ>);

    for(octomap::Octree::leaf_iterator it = mytree->begin_leafs(), end=mytree->end_leafs(); it!=end, ++i)
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
    delete mytree;
  } // endoctomap_to_pcl();
} // endclass


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
