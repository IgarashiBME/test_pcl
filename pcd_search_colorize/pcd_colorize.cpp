#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
using namespace std;

const double MACHINE_HEIGHT = 1.5;
const double OBSTACLE_HEIGHT = 0.10;

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    cout << "viewerOneOff" << std::endl;
}

// ビューワー起動中の毎フレーム実行される
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    //cout << "viewer" << std::endl;
}


int main()
{
    pcl::PointXYZ searchPoint;
    searchPoint.x = 11.005;
    searchPoint.y = -5.252;
    searchPoint.z = 0.023;

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 作成したPointCloudを読み込む
    // pcl::io::loadPCDFile("p_cloud_ascii.pcd", *p_cloud);
    pcl::io::loadPCDFile("takaido_s71-39per_0p08_color.pcd", *p_cloud);
    pcl::copyPointCloud(*p_cloud, *color_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (p_cloud);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 1.0;

    // Neighbors within radius search
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            //std::cout << "    "  <<   (*p_cloud)[ pointIdxRadiusSearch[i] ].x 
            //          << " " << (*p_cloud)[ pointIdxRadiusSearch[i] ].y 
            //          << " " << (*p_cloud)[ pointIdxRadiusSearch[i] ].z 
            //          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

            if ( MACHINE_HEIGHT + searchPoint.z > (*p_cloud)[ pointIdxRadiusSearch[i] ].z && 
                 (*p_cloud)[ pointIdxRadiusSearch[i] ].z > OBSTACLE_HEIGHT + searchPoint.z )
            {
                std::cout << " " << i
                          << " " << (*p_cloud)[ pointIdxRadiusSearch[i] ].x 
                          << " " << (*p_cloud)[ pointIdxRadiusSearch[i] ].y 
                          << " " << (*p_cloud)[ pointIdxRadiusSearch[i] ].z
                          //<< " " << (*color_cloud)[ pointIdxRadiusSearch[i] ].x
                          //<< " " << (*color_cloud)[ pointIdxRadiusSearch[i] ].y
                          //<< " " << (*color_cloud)[ pointIdxRadiusSearch[i] ].z 
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

                // colorized
                (*color_cloud)[ pointIdxRadiusSearch[i] ].r = 255;
            }
    }

    // viewer
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(color_cloud);

    // ビューワー起動時の一回だけ呼ばれる関数をセット
    viewer.runOnVisualizationThreadOnce(viewerOneOff);

    // ビューワー起動中の毎フレーム実行される関数をセット
    viewer.runOnVisualizationThread(viewerPsycho);

    // ビューワー視聴用ループ
    while (!viewer.wasStopped())
    {

    }
    return 0;
}
