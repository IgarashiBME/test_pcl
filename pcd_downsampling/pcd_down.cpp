#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    // 作成したPointCloudを読み込む
    pcl::io::loadPCDFile("../takaido_75per_ori.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZI>("test.pcd", *cloud_filtered, false);

    return (0);
}
