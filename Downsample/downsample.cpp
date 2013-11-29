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
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//double size_x = 0.1;
//double size_y = 0.1;
//double size_z = 0.1;

int main(int argc, char** argv) {
	/* Step 0 */
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	/* Step 1 */
	std::cout << "Load pcd file ...\n";
	pcl::io::loadPCDFile(cmd_arguments.raw_pcd_, *cloud);

	std::cout << "Before resample, #points = " << cloud->points.size() << std::endl;
	std::cout << "Resample ...\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample(new pcl::PointCloud<pcl::PointXYZ>);

	// create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(cloud);
	//grid.setLeafSize(size_x, size_y, size_z);
	grid.setLeafSize(cmd_arguments.resample_resolution_[0],cmd_arguments.resample_resolution_[1],
		cmd_arguments.resample_resolution_[2]);

	grid.filter(*sample);

	/* Step 2 */
	std::cout << "After resample, #points = " << sample->points.size() << std::endl;
	std::cout << "Saving ...\n";
	pcl::io::savePCDFile(cmd_arguments.raw_pcd_, *sample);

	return 0;
}