#include "../include/point_cloud_data.h"

PointCloudData::PointCloudData(int argc, char **argv){

    ros::init(argc, argv, "point_cloud_odometry");    
    my_node_ = new ros::NodeHandle("~");
    loop_rate_ = new ros::Rate(10);

    publish_odom_ = my_node_->advertise<nav_msgs::Odometry>("odom_mathias", 100);
    pose_subscriber_ = my_node_->subscribe("/velodyne_points",10, &PointCloudData::receivePointCloud, this);

    total_icp_iterations_ = 15;
    previous_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    current_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointXYZ my_point; 
    // my_point.x = 10;
    // my_point.z = 10;
    // my_point.y = 10;
    // test->push_back(my_point);

    is_first_pc_reading_ = true;
    new_pc_reading_ = false;

    icp_.setMaximumIterations (total_icp_iterations_);

    odometry_estimation_ = Eigen::Matrix4d::Identity();
}

void PointCloudData::run(){
    

    while(ros::ok()){
        print4x4Matrix(odometry_estimation_);
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
    
    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    // double theta = M_PI / 8;  // The angle of rotation in radians
    // transformation_matrix (0, 0) = std::cos (theta);
    // transformation_matrix (0, 1) = -sin (theta);
    // transformation_matrix (1, 0) = sin (theta);
    // transformation_matrix (1, 1) = std::cos (theta);

    // A translation on Z axis (0.4 meters)
    // transformation_matrix (2, 3) = 0.4;

    
    // The point clouds we will be using
    // PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ>);  // Original point cloud
    // PointCloud<PointXYZ>::Ptr cloud_tr (new PointCloud<PointXYZ>T);  // Transformed point cloud
    // PointCloud<PointXYZ>::Ptr cloud_icp (new PointCloud<PointXYZ>);  // ICP output point cloud 
    

    // Executing the transformation
    // pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
    // *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setMaximumIterations (iterations);
    // icp.setInputSource (cloud_icp);
    // icp.setInputTarget (cloud_in);
    // icp.align (*cloud_icp);
    // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function

    // if (icp.hasConverged ())
    // {
    //     std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    //     std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    //     transformation_matrix = icp.getFinalTransformation ().cast<double>();
    //     print4x4Matrix (transformation_matrix);
    // }
    // else
    // {
    //     PCL_ERROR ("\nICP has not converged.\n");
    //     return (-1);
    // }
}

void PointCloudData::computeOdometry(){
    if(new_pc_reading_){
        // Defining a rotation matrix and translation vector    
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

        if(!is_first_pc_reading_){
            icp_.setInputSource(current_pc_);
            icp_.setInputTarget(previous_pc_);
            icp_.align(*current_pc_);
            if(icp_.hasConverged()){
                transformation_matrix = icp_.getFinalTransformation().cast<double>();
                // print4x4Matrix(transformation_matrix);
                odometry_estimation_ = odometry_estimation_ * transformation_matrix;
                
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
  tf2::Quaternion q;
         
  q.setRPY(0, 0, 0);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = ros::Time::now();
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = 0;
  quatOdom.pose.pose.position.y = 0;
  quatOdom.pose.pose.position.z = 0;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = 0;
  quatOdom.twist.twist.linear.y = 0;
  quatOdom.twist.twist.linear.z = 0;
  quatOdom.twist.twist.angular.x = 0;
  quatOdom.twist.twist.angular.y = 0;
  quatOdom.twist.twist.angular.z = 0;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  publish_odom_.publish(quatOdom);
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
    Eigen::Vector3f euler_angles = rotation.eulerAngles(0, 1, 2);
    ROS_INFO("R: %f, P: %f, Y: %f", euler_angles[0]*(180/M_PI), euler_angles[1]*(180/M_PI), euler_angles[2]*(180/M_PI));
}

