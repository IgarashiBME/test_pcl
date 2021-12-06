#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
using namespace std;

const double MACHINE_HEIGHT = 1.00;
const double OBSTACLE_HEIGHT = 0.30;
const int NEIGHBOR_THRE = 10;

// Called once when the viewer is started
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
	cout << "viewerOneOff" << std::endl;
}

// executed every frame
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	//cout << "viewer" << std::endl;
}

vector<string> split(string& input, char delimiter)
{
    istringstream stream(input);
    string field;
    vector<string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

int main()
{
    ifstream ifs("../ground.csv");

    // csv to vector
    string line;
    int i = 0;
    vector<vector <string>> data;
    while (getline(ifs, line)) {
        vector<string> strvec = split(line, ',');

        if (strvec.size() == 3){ 
            data.emplace_back();
            data.at(i).emplace_back(strvec.at(0));
            data.at(i).emplace_back(strvec.at(1));
            data.at(i).emplace_back(strvec.at(2));
        }
        i++;
    }
    cout << data.size() << "," << data.at(0).size() << std::endl;

    //for (int i = 0; i < data.size(); i++){
    //    cout << data.at(i).at(0) << ", "
    //         << data.at(i).at(1) << ", "
    //         << data.at(i).at(2) << std::endl;
    //}

    // prepare ground pointcloud
    pcl::PointCloud<pcl::PointXYZRGB> ground_cloud;
    ground_cloud.width = data.size();
    ground_cloud.height = 1;
    ground_cloud.is_dense = false;
    ground_cloud.points.resize(ground_cloud.width * ground_cloud.height);

    for (int i = 0; i < data.size(); i++){
        ground_cloud.points[i].x = stof(data.at(i).at(0));
        ground_cloud.points[i].y = stof(data.at(i).at(1));
        ground_cloud.points[i].z = stof(data.at(i).at(2));
        ground_cloud.points[i].r = 255;
    }

    // ground pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(ground_cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ground_cloud_ptr, *search_cloud);

    for (int i = 0; i < ground_cloud.points.size(); i++){
        cout << ground_cloud.points[i] << "," << search_cloud->points[i] << std::endl;
    }

    // map pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("../takaido_75per_0p1.pcd", *map_cloud);
    pcl::copyPointCloud(*map_cloud, *map_rgb_cloud);

    // set kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (map_cloud);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 2.0;

    // Neighbors within radius search
    for (int i = 0; i < ground_cloud.points.size(); i++)
    {
        if ( kdtree.radiusSearch (search_cloud->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for (std::size_t j = 0; j < pointIdxRadiusSearch.size() ; ++j)
            {
                //std::cout << "    "  <<   (*map_cloud)[ pointIdxRadiusSearch[j] ].x 
                //          << " " << (*map_cloud)[ pointIdxRadiusSearch[j] ].y 
                //         << " " << (*map_cloud)[ pointIdxRadiusSearch[j] ].z 
                //          << " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;

                // find a pointcloud higher than the ground 
                // MACHINE_HEIGHT +search_cloud->points[i].z > (*map_cloud)[ pointIdxRadiusSearch[j-1] ].z &&
                // +search_cloud->points[i].z
                if ( MACHINE_HEIGHT +search_cloud->points[i].z > (*map_cloud)[ pointIdxRadiusSearch[j] ].z &&
                     (*map_cloud)[ pointIdxRadiusSearch[j] ].z > OBSTACLE_HEIGHT +search_cloud->points[i].z &&
                     abs( (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].x - search_cloud->points[i].x ) < 0.2 &&
                     abs( (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].y - search_cloud->points[i].y ) < 0.2 )
                {
                    //std::cout << " " << i << " " << j
                    //          << " " << (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].x 
                    //          << " " << (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].y 
                    //          << " " << (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].z
                    //          << " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;

                    //std::cout << " " << i << " " << j
                    //          << " " << abs( (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].x - search_cloud->points[i].x )
                    //          << " " << abs( (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].y - search_cloud->points[i].y )
                    //          << " " << (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].z
                    //          << std::endl;

                    // colorized
                    (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].r = 255;
                }
            }
        }
        cout << i << std::endl;
    }

    // denoise
    radius = 0.3;
    int neighbor_obs;
    for (int i = 0; i < map_rgb_cloud->points.size(); i++)
    {
        if ( (*map_rgb_cloud)[i].r > 0 )
        {
            if ( kdtree.radiusSearch ((*map_cloud)[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                neighbor_obs = 0;
                for (std::size_t j = 0; j < pointIdxRadiusSearch.size() ; ++j)
                {
                    if ( (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ].r > 0 ) 
                    {
                        neighbor_obs ++;
                        //cout << (*map_rgb_cloud)[ pointIdxRadiusSearch[j] ]
                        //     << " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;
                    }
                }

                if (neighbor_obs < NEIGHBOR_THRE)
                {
                    cout << neighbor_obs << std::endl;
                    (*map_rgb_cloud)[i].r = 0;
                }
            }
        }        
    }

    // ビューワーの作成
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(ground_cloud));
    viewer.showCloud(map_rgb_cloud);

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
