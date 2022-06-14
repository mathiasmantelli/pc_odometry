
#include "../include/point_cloud_data.h"



int main(int argc, char **argv){
    PointCloudData pcd;
    ros::init(argc, argv, "point_cloud_odometry");
    ros::NodeHandle my_node; 
    ros::Subscriber pose_subscriber = my_node.subscribe("/velodyne_points",10, &PointCloudData::receive_point_cloud, &pcd);
    ros::spin();

    return 0;
}