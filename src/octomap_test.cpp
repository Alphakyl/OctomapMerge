#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <cstdlib>
#include "octomap_merge/OctomapArray.h"
#include "octomap_merge.h"

ros::Publisher pub;
bool has_been_called;


void callback_map(const octomap_msgs::OctomapConstPtr& msg)
{
    octomap_merge::OctomapArray myNeighbors;
    myNeighbors.num_octomaps = 1;
    myNeighbors.octomaps.push_back(*msg);
    pub.publish(myNeighbors);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_test");
    ros::NodeHandle nh;
    has_been_called = 0;
    ros::Subscriber sub = nh.subscribe("octomap_binary", 100, callback_map);
    pub = nh.advertise<octomap_merge::OctomapArray>("neighbors",100);

    ros::spin();
    return 0;
}
