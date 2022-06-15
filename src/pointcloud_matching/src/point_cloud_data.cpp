#include "../include/point_cloud_data.h"

PointCloudData::PointCloudData(int argc, char **argv){

    ros::init(argc, argv, "point_cloud_odometry");    
    my_node_ = new ros::NodeHandle("~");
    loop_rate_ = new ros::Rate(5);

    publish_odom_ = my_node_->advertise<nav_msgs::Odometry>("odom_mathias", 100);
    pose_subscriber_ = my_node_->subscribe("/velodyne_points",10, &PointCloudData::receivePointCloud, this);

    total_icp_iterations_ = 15;
    previous_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    current_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    is_first_pc_reading_ = true;
    new_pc_reading_ = false;

    icp_.setMaximumIterations (total_icp_iterations_);

    odometry_estimation_ = Eigen::Matrix4d::Identity();

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    to_be_published_odom_.header.stamp = ros::Time::now();
    to_be_published_odom_.header.frame_id = "odom";
    to_be_published_odom_.child_frame_id = "base_link";
    to_be_published_odom_.pose.pose.position.x = 0;
    to_be_published_odom_.pose.pose.position.y = 0;
    to_be_published_odom_.pose.pose.position.z = 0;
    to_be_published_odom_.pose.pose.orientation.x = q.x();
    to_be_published_odom_.pose.pose.orientation.y = q.y();
    to_be_published_odom_.pose.pose.orientation.z = q.z();
    to_be_published_odom_.pose.pose.orientation.w = q.w();
    to_be_published_odom_.twist.twist.linear.x = 0;
    to_be_published_odom_.twist.twist.linear.y = 0;
    to_be_published_odom_.twist.twist.linear.z = 0;
    to_be_published_odom_.twist.twist.angular.x = 0;
    to_be_published_odom_.twist.twist.angular.y = 0;
    to_be_published_odom_.twist.twist.angular.z = 0;

    delta_distance_ = 0; 
    delta_angle_ = 0;
}

void PointCloudData::run(){
    while(ros::ok()){
        // print4x4Matrix(odometry_estimation_);
        computeOdometry();
        publishOdom();
        ros::spinOnce();
        loop_rate_->sleep();
    }
}

void PointCloudData::receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    //Converting Pointcloud2 to PointCloud from PCL
    pcl::PCLPointCloud2 pcl_version_pc; 
    pcl_conversions::toPCL(*cloud_msg, pcl_version_pc);
    pcl::fromPCLPointCloud2(pcl_version_pc, *current_pc_);
    new_pc_reading_ = true;
}

void PointCloudData::computeOdometry(){
    if(new_pc_reading_){
        // Defining a rotation matrix and translation vector    
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
        
        //Checking whether there is a previous point cloud to compute the odometry
        if(!is_first_pc_reading_){

            //Defining the point clouds to be matched
            icp_.setInputSource(current_pc_);
            icp_.setInputTarget(previous_pc_);
            icp_.align(*current_pc_);
            if(icp_.hasConverged()){
                //Getting the transformation between previous point cloud and the current one
                transformation_matrix = icp_.getFinalTransformation().cast<double>();
                
                //Computing the distance between the previous pose to the current one
                delta_distance_ = sqrt(pow(transformation_matrix(0,3) - odometry_estimation_(0,3), 2) + 
                                  pow(transformation_matrix(1,3) - odometry_estimation_(1,3), 2) + 
                                  pow(transformation_matrix(2,3) - odometry_estimation_(2,3), 2));
                
                
                Eigen::Matrix3f rotation; 
                rotation(0,0) = odometry_estimation_(0,0); rotation(1,0) = odometry_estimation_(1,0); rotation(2,0) = odometry_estimation_(2,0);
                rotation(0,1) = odometry_estimation_(0,1); rotation(1,1) = odometry_estimation_(1,1); rotation(2,1) = odometry_estimation_(2,1);
                rotation(0,2) = odometry_estimation_(0,2); rotation(1,2) = odometry_estimation_(1,2); rotation(2,2) = odometry_estimation_(2,2);
                euler_angles_ = rotation.eulerAngles(0, 1, 2);
                float current_yaw_angle = euler_angles_[2];


                rotation(0,0) = transformation_matrix(0,0); rotation(1,0) = transformation_matrix(1,0); rotation(2,0) = transformation_matrix(2,0);
                rotation(0,1) = transformation_matrix(0,1); rotation(1,1) = transformation_matrix(1,1); rotation(2,1) = transformation_matrix(2,1);
                rotation(0,2) = transformation_matrix(0,2); rotation(1,2) = transformation_matrix(1,2); rotation(2,2) = transformation_matrix(2,2);
                euler_angles_ = rotation.eulerAngles(0, 1, 2);

                //Computing the angle between the previous pose to the current one
                delta_angle_ = current_yaw_angle - euler_angles_[2];
                delta_angle_ += (delta_angle_ > 180) ? -360 : (delta_angle_ < -180) ? 360 : 0;

                //Estimating the odom given based on the current transformation between the point clouds
                odometry_estimation_ = odometry_estimation_ * transformation_matrix;

                rotation(0,0) = odometry_estimation_(0,0); rotation(1,0) = odometry_estimation_(1,0); rotation(2,0) = odometry_estimation_(2,0);
                rotation(0,1) = odometry_estimation_(0,1); rotation(1,1) = odometry_estimation_(1,1); rotation(2,1) = odometry_estimation_(2,1);
                rotation(0,2) = odometry_estimation_(0,2); rotation(1,2) = odometry_estimation_(1,2); rotation(2,2) = odometry_estimation_(2,2);
                euler_angles_ = rotation.eulerAngles(0, 1, 2);
                current_yaw_angle = euler_angles_[2] + 3.14;
                current_yaw_angle += (current_yaw_angle > 180) ? -360 : (current_yaw_angle < -180) ? 360 : 0;

                //Creating the odom message based on the computations
                tf2::Quaternion q;
                q.setRPY(0, 0, current_yaw_angle);
                new_odom_.header.stamp = ros::Time::now();
                new_odom_.header.frame_id = "odom";
                new_odom_.child_frame_id = "base_link";
                new_odom_.pose.pose.position.x = transformation_matrix(0,3);
                new_odom_.pose.pose.position.y = transformation_matrix(1,3);
                new_odom_.pose.pose.position.z = transformation_matrix(2,3);
                new_odom_.pose.pose.orientation.x = q.x();
                new_odom_.pose.pose.orientation.y = q.y();
                new_odom_.pose.pose.orientation.z = q.z();
                new_odom_.pose.pose.orientation.w = q.w();
                new_odom_.twist.twist.linear.x = 0;
                new_odom_.twist.twist.linear.y = 0;
                new_odom_.twist.twist.linear.z = 0;
                new_odom_.twist.twist.angular.x = 0;
                new_odom_.twist.twist.angular.y = 0;
                new_odom_.twist.twist.angular.z = 0;
                
            }else{
                ROS_INFO("ICP has not converged\n");
            }
        }else{
            is_first_pc_reading_ = false;    
        }
        *previous_pc_ = *current_pc_;
        new_pc_reading_ = false;
    }
}

void PointCloudData::publishOdom(){
 
    //Filling up the odom message to be published
    to_be_published_odom_.header.stamp = ros::Time::now();
    to_be_published_odom_.header.frame_id = "odom";
    to_be_published_odom_.child_frame_id = "base_link";
    to_be_published_odom_.pose.pose.position.x = new_odom_.pose.pose.position.x;
    to_be_published_odom_.pose.pose.position.y = new_odom_.pose.pose.position.y;
    to_be_published_odom_.pose.pose.position.z = new_odom_.pose.pose.position.z;
    to_be_published_odom_.pose.pose.orientation.x = new_odom_.pose.pose.orientation.x;
    to_be_published_odom_.pose.pose.orientation.y = new_odom_.pose.pose.orientation.y;
    to_be_published_odom_.pose.pose.orientation.z = new_odom_.pose.pose.orientation.z;
    to_be_published_odom_.pose.pose.orientation.w = new_odom_.pose.pose.orientation.w;
    to_be_published_odom_.twist.twist.linear.x = delta_distance_ /(to_be_published_odom_.header.stamp.toSec() - new_odom_.header.stamp.toSec());
    to_be_published_odom_.twist.twist.linear.y = 0;
    to_be_published_odom_.twist.twist.linear.z = 0;
    to_be_published_odom_.twist.twist.angular.x = 0;
    to_be_published_odom_.twist.twist.angular.y = 0;
    to_be_published_odom_.twist.twist.angular.z = delta_angle_ /(to_be_published_odom_.header.stamp.toSec() - new_odom_.header.stamp.toSec());

    new_odom_.header.stamp = to_be_published_odom_.header.stamp;

    publish_odom_.publish(new_odom_);
}

void PointCloudData::print4x4Matrix (const Eigen::Matrix4d & matrix){
    ROS_INFO("Rotation matrix :");
    ROS_INFO("    | %f %f %f | ", matrix(0, 0),matrix (0, 1), matrix (0, 2));
    ROS_INFO("R = | %f %f %f | ", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    ROS_INFO("    | %f %f %f | ", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    ROS_INFO("Translation vector :");
    ROS_INFO("t = < %f, %f, %f >", matrix (0, 3), matrix (1, 3), matrix (2, 3));

    Eigen::Matrix3f rotation; 
    rotation(0,0) = matrix(0,0); rotation(1,0) = matrix(1,0); rotation(2,0) = matrix(2,0);
    rotation(0,1) = matrix(0,1); rotation(1,1) = matrix(1,1); rotation(2,1) = matrix(2,1);
    rotation(0,2) = matrix(0,2); rotation(1,2) = matrix(1,2); rotation(2,2) = matrix(2,2);
    euler_angles_ = rotation.eulerAngles(0, 1, 2);
    ROS_INFO("R: %f, P: %f, Y: %f", euler_angles_[0]*(180/M_PI), euler_angles_[1]*(180/M_PI), euler_angles_[2]*(180/M_PI));
}

