#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pc;
    void print4x4Matrix (const Eigen::Matrix4d & matrix);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pc;
private:
    int total_icp_iterations_;
    bool is_first_pc_reading_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    Eigen::Matrix4d odometry_estimation_;

};