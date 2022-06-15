#include "../include/point_cloud_data.h"

PointCloudData::PointCloudData(){
    total_icp_iterations_ = 5;
    previous_pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointXYZ my_point; 
    // my_point.x = 10;
    // my_point.z = 10;
    // my_point.y = 10;
    // test->push_back(my_point);

    is_first_pc_reading_ = true;

    icp_.setMaximumIterations (total_icp_iterations_);

    odometry_estimation_ = Eigen::Matrix4d::Identity();
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
    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0);
    ROS_INFO("R: %f, P: %f, Y: %f", euler_angles[0], euler_angles[1], euler_angles[2]);
    // float roll, pitch, yaw; 
    // roll = M_PI / atan2(matrix(2,1), matrix(2,2));
    // pitch = M_PI / atan2( -matrix(2,0), std::pow(matrix(2,1) * matrix(2,1) + matrix(2,2) * matrix(2,2),0.5));
    // yaw = M_PI / atan2( matrix(1,0),matrix(0,0));
    // ROS_INFO("R: %f, P: %f, Y: %f", roll, pitch, yaw);
    // std::cout<<"roll is Pi/" <<M_PI/atan2( matrix(2,1),matrix(2,2) ) <<std::endl;
    // std::cout<<"pitch: Pi/" <<M_PI/atan2( -matrix(2,0), std::pow( matrix(2,1)*matrix(2,1) +matrix(2,2)*matrix(2,2) ,0.5  )  ) <<std::endl;
    // std::cout<<"yaw is Pi/" <<M_PI/atan2( matrix(1,0),matrix(0,0) ) <<std::endl;
}

void PointCloudData::receive_point_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    pcl::PCLPointCloud2 pcl_version_pc; 
    pcl_conversions::toPCL(*cloud_msg, pcl_version_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_version_pc, *current_pc);

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    if(!is_first_pc_reading_){
        icp_.setInputSource(current_pc);
        icp_.setInputTarget(previous_pc);
        icp_.align(*current_pc);
        if(icp_.hasConverged()){
            transformation_matrix = icp_.getFinalTransformation().cast<double>();
            print4x4Matrix(transformation_matrix);
            odometry_estimation_ = odometry_estimation_ * transformation_matrix;
            print4x4Matrix(odometry_estimation_);
        }else{
            ROS_INFO("ICP has not converged\n");
        }
    }else{
        is_first_pc_reading_ = false;    
    }
    *previous_pc = *current_pc;

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