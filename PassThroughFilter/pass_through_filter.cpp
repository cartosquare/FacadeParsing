#include "cmd_utility.h"
#include "pcl\io\pcd_io.h"
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl\visualization\pcl_visualizer.h>

#include "pcl\filters\\voxel_grid.h"
#include "pcl\filters\statistical_outlier_removal.h"
#include "pcl\filters\extract_indices.h"
#include "pcl\filters\radius_outlier_removal.h"
#include "pcl\filters\conditional_removal.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <fstream>
#include <iostream>

CommandLineArgument cmd_arguments;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ> search;

bool filter = true;
int pick_time = 0;
std::vector<pcl::PointXYZ> picked_points;

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie);
void FindPickedPoint(const pcl::visualization::PointPickingEvent& event);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookie);
void ShowCloud();

pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough_filter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,		// input cloud
	const std::string& axe,			//  filter field name is which coordinate? can be: "x", "y" or "z"
	double minLimits,				// min range
	double maxLimits,				// max range
	bool inside						// inside or outside a given user range.
	);


int main(int argc, char** argv) {
	/* Step 0 */
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	/* Step 1 */
	cout << "\n=========== Load raw building point cloud ... ==================\n";
	pcl::io::loadPCDFile(cmd_arguments.pt_pcd_, *cloud);

	/* Step 2 */
	cout << "\n=========== Showing the building plane ... ==================\n";
	ShowCloud();


	return 0;
}

void FindPickedPoint(const pcl::visualization::PointPickingEvent& event) {
	int idx = event.getPointIndex ();
	if (idx == -1)
	{
		std::cout << "Invalid pick!\n;";
		return;
	}
	search.setInputCloud(cloud);

	// Return the correct index in the cloud instead of the index on the screen
	std::vector<int> indices (1);
	std::vector<float> distances (1);

	// Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
	pcl::PointXYZ picked_pt;
	event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
	search.nearestKSearch (picked_pt, 1, indices, distances);
	picked_points.push_back(picked_pt);
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* cookie)
{
	if(pick_time == 0)
	{
		FindPickedPoint(event);
		pick_time = 1;

		cout << "Lower bottom point Chosen! [" << picked_points[0].x << ", " 
			<< picked_points[0].y << ", " << picked_points[0].z << "]\n";
	}
	else
	{
		FindPickedPoint(event);

		pick_time = 0;
		cout << "Upper right point chosen! And it's ready to filter.[" << picked_points[1].x << ", " 
			<< picked_points[1].y << ", " << picked_points[1].z << "]\n";
	}
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookie) {
	if(event.getKeySym() == "f" && filter)
	{
		std::cout << "original #points = " << cloud->points.size() <<  std::endl;
		cout << "Filter x axe ... \n";

		pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered_cloud = 
		passThrough_filter(cloud, "x", picked_points[0].x, picked_points[1].x, true);
		std::cout << "now #points = " << x_filtered_cloud->points.size() <<  std::endl;

		cout << "Filter y axe ...\n";
		pcl::PointCloud<pcl::PointXYZ>::Ptr xy_filtered_cloud = 
			passThrough_filter(x_filtered_cloud, "y", picked_points[0].y, picked_points[1].y, true);
		std::cout << "now #points = " << xy_filtered_cloud->points.size() <<  std::endl;
		cout << "Filter Done.\n";

		cout << "Save filtered clouds ...\n";
		pcl::io::savePCDFile(cmd_arguments.pt_pcd_, *xy_filtered_cloud);
		cout << "Save Done.\n";
		filter = false;
	}
}

void ShowCloud() {
	viewer.reset (new pcl::visualization::PCLVisualizer());
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "build plane cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerPointPickingCallback(&pp_callback);
	viewer->registerKeyboardCallback(&keyboardEventOccurred);
	viewer->spin ();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough_filter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,		// input cloud
	const std::string& axe,			//  filter field name is which coordinate? can be: "x", "y" or "z"
	double minLimits,				// min range
	double maxLimits,				// max range
	bool inside						// inside or outside a given user range.
	)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(inputCloud);
	pass.setFilterFieldName(axe);
	pass.setFilterLimits(minLimits, maxLimits);
	if(!inside)
		pass.setFilterLimitsNegative(true);
	pass.filter(*filtered);

	return (filtered);
}