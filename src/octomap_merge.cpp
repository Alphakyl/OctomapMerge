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

	
// Constructor
OctomapMerge::OctomapMerge(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("Constructing OctomapMerge Class");
    initializeSubscribers();
    initializePublishers();
    maptoconvertptr = &maptoconvert;
    pub_times = 5;
    myMapNew = false;
    otherMapsNew = false;
}// end Constructor
 
// Destructor
OctomapMerge::~OctomapMerge()
{
}// end Destructor

void OctomapMerge::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    sub_mymap = nh_.subscribe("octomap_binary", 100, &OctomapMerge::callback_myMap, this);
    sub_neighbors = nh_.subscribe("neighbors", 100, &OctomapMerge::callback_neighborMaps, this);
}

void OctomapMerge::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    pub_merged = nh_.advertise<sensor_msgs::PointCloud2>("merged_pc", 10, true);  
}


//Callbacks
void OctomapMerge::callback_myMap(const octomap_msgs::OctomapConstPtr& msg)
{
  ROS_INFO("my_map callback");
  myMap = *msg;
	myMapNew = true;
} // end callback_myMap();

void OctomapMerge::callback_neighborMaps(const octomap_merge::OctomapArrayConstPtr& msg){
  ROS_INFO("neighbors callbck");
  neighbors = *msg;
	otherMapsNew = true;
} // end callback_neighborMaps();
 
void OctomapMerge::octomap_to_pcl(octomap_msgs::Octomap * map, sensor_msgs::PointCloud2Ptr occupiedCellsMsg)
{
    ROS_INFO("Octomap_to_pcl");
	  ROS_INFO("Creating tree with same resolution");
    octomap::OcTree* myTree = new octomap::OcTree(map->resolution);
    myTree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*map);
    double size, value;
    float lower_corner[3];
    int idk, depth, width;
    int lowest_depth = (int)myTree->getTreeDepth();
    int count = 0;
    int freeCount = 0;
    int occCount = 0;
    float voxel_size = (float)myTree->getResolution();

    PointT point;
    PointCloud_xyz::Ptr occupiedCells(new PointCloud_xyz);
    ROS_INFO("Iterating through tree");
    for(octomap::OcTree::leaf_iterator it = myTree->begin_leafs(), end=myTree->end_leafs(); it!=end; ++it)
    {
        depth = (int)it.getDepth();
        point.x = (float)it.getX();
        point.y = (float)it.getY();
        point.z = (float)it.getZ();
        size = it.getSize();

        if (myTree->isNodeOccupied(*it))
        {
            if(depth==lowest_depth)
            {
                occupiedCells->points.push_back(point);
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
                                occupiedCells->points.push_back(point);
                            } // endif
                        } // endfor
                    } // endfor
                } // endfor
            }  // endelseif
        } // endif
    } // endfor
	  ROS_INFO("deleting tree structure");
    delete myTree;
    pcl::toROSMsg(*occupiedCells,*occupiedCellsMsg);
} // endoctomap_to_pcl();

void OctomapMerge::merge()
{  
    ROS_INFO("Entered Merge Function");
		ROS_INFO("Creating pointers");
    sensor_msgs::PointCloud2Ptr myMapMsg(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr mergedMapMsg(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2Ptr neighborMapMsg(new sensor_msgs::PointCloud2);


		// Convert my map to PCL
    ROS_INFO("Converting my map");
    *maptoconvertptr = myMap;
    octomap_to_pcl(maptoconvertptr, myMapMsg);
    // Add map to merge
    ROS_INFO("Adding Map to Merge");
    pcl::concatenatePointCloud(*mergedMapMsg, *myMapMsg, *mergedMapMsg);
    // For each map in the neighbor set
    for(int i = 0; i<neighbors.num_octomaps; i++)
		{
			// Convert this map to PCL
      ROS_INFO("Converting Neighbor Map");
      *maptoconvertptr = neighbors.octomaps[i];
      octomap_to_pcl(maptoconvertptr,neighborMapMsg);
      // Add map to merge
      ROS_INFO("Adding NeighborMap to Merge");
      pcl::concatenatePointCloud(*mergedMapMsg, *neighborMapMsg, *mergedMapMsg);
		} // endfor
    /* Filter merge cloud to avoid over publishing points */
    // pcl_conversions::toPCL(merged_out, *merged_cloud);
    // sor.setInputCloud(merged_cloud);
    // sor.setLeafSize(0.1f, 0.1f, 0.1f);
    // sor.filter(*filtered_merged_cloud);
    // pcl_conversions::fromPCL(*filtered_merged_cloud, merged_pub);
    // merged_pub = merged_out;
  for(int i = 0; i<pub_times; i++){
    mergedMapMsg->header.stamp = ros::Time::now();
    mergedMapMsg->header.frame_id = "world";
		pub_merged.publish(*mergedMapMsg);
  }
} // end merge();

int main(int argc, char **argv)
{
    ros::init(argc, argv ,"octomap_merge");
    ros::NodeHandle nh;

    int rate;
    ros::NodeHandle private_node_handle("~");
    private_node_handle.param("rate", rate, int(1));

    OctomapMerge *octomap_merger = new OctomapMerge(&nh);

    ros::Rate r(rate);
    while(nh.ok())
    {
        ros::spinOnce();
        if(octomap_merger->otherMapsNew)
        {
						octomap_merger->myMapNew = false;
            octomap_merger->otherMapsNew = false;
            octomap_merger->merge();
        }
        r.sleep();
    }
    return 0;
}
