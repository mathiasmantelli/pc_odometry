#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudData{

public:
    PointCloudData();
    void receive_point_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    int x;

};