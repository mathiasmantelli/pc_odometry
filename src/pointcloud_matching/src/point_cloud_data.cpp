#include "../include/point_cloud_data.h"

PointCloudData::PointCloudData(){
    x = 2;
    test = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ my_point; 
    my_point.x = 10;
    my_point.z = 10;
    my_point.y = 10;
    test->push_back(my_point);
}

void PointCloudData::receive_point_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    //ROS_INFO("Sequence: [%d]", cloud_msg->header.seq);
    // ROS_INFO("Size: [%d]", cloud_msg->data.size());
    
    pcl::PCLPointCloud2 pcl_version_pc; 
    pcl_conversions::toPCL(*cloud_msg, pcl_version_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_version_pc, *current_pc);
    // test = current_pc;

    ROS_INFO("Size: [%d] - [%d]", cloud_msg->data.size(), test->size());
    int cont = 0;
    for(sensor_msgs::PointCloud2ConstIterator<float> it(*cloud_msg, "x"); it != it.end(); ++it){
        if(cont == 50){
            ROS_INFO("Point2 0: [%f, %f, %f]", it[0], it[1], it[2]);    
        }
        if(cont <= 50)cont++;
    }
    int index = 50;
    ROS_INFO("PointP 0: [%f, %f, %f]", current_pc->points[index].x, current_pc->points[index].y, current_pc->points[index].z);

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

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