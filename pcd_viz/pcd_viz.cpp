#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
using namespace std;

// Called only once when the viewer is started
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2);
	cout << "viewerOneOff" << std::endl;
}

// Runs every frame while the viewer is running
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	cout << "viewer" << std::endl;
}


int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Loads the PointCloud
	// pcl::io::loadPCDFile("p_cloud_ascii.pcd", *p_cloud);
	pcl::io::loadPCDFile("../test.pcd", *p_cloud);

	// Preparing a viewer
	pcl::visualization::CloudViewer viewer("PointCloudViewer");
	viewer.showCloud(p_cloud);

	// Set a function that will be called only once when the viewer starts
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// Set a function that will be executed every frame when the viewer is started
	viewer.runOnVisualizationThread(viewerPsycho);

	// Loop for viewing
	while (!viewer.wasStopped())
	{

	}
	return 0;
}
