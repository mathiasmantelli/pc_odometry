
#include "../include/point_cloud_data.h"



int main(int argc, char **argv){
    PointCloudData pcd(argc, argv);
    pcd.run();
    // ros::NodeHandle my_node; 
    // pcd.publish_odom = my_node.advertise<nav_msgs::Odometry>("odom_mathias", 100);
    // ros::Subscriber pose_subscriber = my_node.subscribe("/velodyne_points",10, &PointCloudData::receivePointCloud, &pcd);
    // ros::spin();

    return 0;
}