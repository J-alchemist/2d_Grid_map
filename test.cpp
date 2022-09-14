#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GridMap_test");
    ros::NodeHandle n;
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = ros::Time::now(); 
    map.info.resolution = 0.05;          // m/ceil
    map.info.width      = 100;           // ceil
    map.info.height     = 100;          
    map.info.origin.position.x = 2.0;
    map.info.origin.position.y = 2.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 0.0;
    map.data.resize(map.info.width * map.info.height);


    for (int i=0; i<map.info.width*map.info.height; ++i) {
        if (i<70)
            map.data[i] = 0;
        else {

            map.data[i] = -1; 
        }

    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        map_pub.publish(map);
        loop_rate.sleep();
    }

    return 0;
}
