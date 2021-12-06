#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
using namespace std;

// Called once when the viewer is started
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
	cout << "viewerOneOff" << std::endl;
}

// executed every frame
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	cout << "viewer" << std::endl;
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

            //cout << i << " ";
            //cout << data.at(i).at(0) << ", "
            //     << data.at(i).at(1) << ", "
            //     << data.at(i).at(2) << std::endl;
        }
        i++;
    }
    cout << data.size() << "," << data.at(0).size() << std::endl;

    //for (int i = 0; i < data.size(); i++){
    //    cout << data.at(i).at(0) << ", "
    //         << data.at(i).at(1) << ", "
    //         << data.at(i).at(2) << std::endl;
    //}

    //pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI> ground_cloud;
    ground_cloud.width = data.size() * data.at(0).size();
    ground_cloud.height = 1;
    ground_cloud.is_dense = false;
    ground_cloud.points.resize(ground_cloud.width * ground_cloud.height);

    for (int i = 0; i < data.size(); i++){
        ground_cloud.points[i].x = stof(data.at(i).at(0));
        ground_cloud.points[i].y = stof(data.at(i).at(1));
        ground_cloud.points[i].z = stof(data.at(i).at(2));
    }
 
    // ビューワーの作成
    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(ground_cloud));
    viewer.showCloud(cloud_ptr);

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
