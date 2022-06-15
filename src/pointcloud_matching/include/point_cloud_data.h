#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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
    PointCloudData(int argc, char **argv);
    void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void print4x4Matrix (const Eigen::Matrix4d & matrix);
    void publishOdom();
    void computeOdometry();
    
    void run();

private:
    int total_icp_iterations_;
    bool is_first_pc_reading_, new_pc_reading_;
    float delta_distance_, delta_angle_;
    Eigen::Vector3f euler_angles_;
    nav_msgs::Odometry new_odom_, to_be_published_odom_;
    ros::NodeHandle* my_node_;
    ros::Publisher publish_odom_;
    ros::Subscriber pose_subscriber_;
    ros::Rate* loop_rate_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pc_, current_pc_;
    Eigen::Matrix4d odometry_estimation_;

};