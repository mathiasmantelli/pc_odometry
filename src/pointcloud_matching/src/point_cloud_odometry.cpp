
#include "../include/point_cloud_data.h"

int main(int argc, char **argv){
    PointCloudData pcd(argc, argv);
    pcd.run();

    return 0;
}