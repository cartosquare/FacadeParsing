
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

double radius = 0.5;
int neighbers = 5;

CommandLineArgument cmd_arguments;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_outlier;

bool load_pcd_from_txt(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZ>::Ptr radius_outliers_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
															double radius, int min_neighbors);

pcl::PointCloud<pcl::PointXYZ>::Ptr random_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
															double distanceThreshold, int maxIters, double probility);

pcl::PointCloud<pcl::PointXYZ>::Ptr statiscal_outlier_removal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	int num_neighbors,	//  The number of neighbors to analyze for each point
	double stddevmul, // all points who have a distance larger than @stddevmul@ standard deviation of the mean distance 
	// to the query point will be marked as outliers and removed.
	bool inliers
	);

pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough_filter(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,		// input cloud
	const std::string& axe,			//  filter field name is which coordinate? can be: "x", "y" or "z"
	double minLimits,				// min range
	double maxLimits,				// max range
	bool inside						// inside or outside a given user range.
	);


int main(int argc, char** argv) 
{
	// Step 0
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	// Step 1
	std::cout << "Convert .txt format to .pcd format ...\n";
	if (!load_pcd_from_txt(cmd_arguments.raw_txt_,  cloud))
	{
		std::cout  << ".txt to .pcd fail!\n";
	}

	// Step 2
	std::cout << "Remove outliers ... \n";
	cloud_remove_outlier = radius_outliers_removal(cloud, radius, neighbers);
	
	// Step 3
	std::cout << "Save ... \n";
	pcl::io::savePCDFile(cmd_arguments.raw_pcd_, *cloud_remove_outlier);

#if 0
	// Step 3
	std::cout << "Add to viewer ... \n";
	viewer.reset (new pcl::visualization::PCLVisualizer());
	viewer->addPointCloud<pcl::PointXYZ> (cloud_remove_outlier, "build plane cloud");

	// get the bounder box of the transformed cloud
	pcl::PointXYZ min_extent, max_extent;
	pcl::getMinMax3D(*cloud_remove_outlier, min_extent, max_extent);

	pcl::PointXYZ p1(min_extent.x, min_extent.y, max_extent.z), p2(min_extent.x, max_extent.y, max_extent.z), 
		p3(max_extent.x, max_extent.y, max_extent.z), p4(max_extent.x, min_extent.y, max_extent.z);

	viewer->addLine(p1, p2, " line1");
	viewer->addLine(p2, p3, "line2");
	viewer->addLine(p3, p4, "line3");
	viewer->addLine(p4, p1, "line4");
	
	viewer->setBackgroundColor (0, 0, 0);
	viewer->spin ();
#endif
	

	return 0;
}

bool load_pcd_from_txt(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// fill in the cloud data
	std::fstream ifs(filePath);

	if(ifs.is_open())
	{
		while(!ifs.eof())
		{
			pcl::PointXYZ point;
			ifs >> point.x >> point.y >> point.z;

			cloud->push_back(point);
		}

		ifs.close();

		return true;
	}
	return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr radius_outliers_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, double radius, int min_neighbors)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(inputCloud);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.filter(*filter);

	return filter;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr my_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
												  double size_x, double size_y, double size_z)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample(new pcl::PointCloud<pcl::PointXYZ>);

	// create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(inputCloud);
	grid.setLeafSize(size_x, size_y, size_z);
	grid.filter(*sample);

	return (sample);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr statiscal_outlier_removal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	int num_neighbors,	//  The number of neighbors to analyze for each point
	double stddevmul, // all points who have a distance larger than @stddevmul@ standard deviation of the mean distance 
	// to the query point will be marked as outliers and removed.
	bool inliers
	)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterd(new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(inputCloud);
	sor.setMeanK(num_neighbors);
	sor.setStddevMulThresh(stddevmul);
	if(!inliers)
		sor.setNegative(true);
	sor.filter(*filterd);

	return (filterd);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr random_sample_consensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
															double distanceThreshold, int maxIters, double probility)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);	// output cloud
	std::vector<int> inliers;	// model indices

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr	// plane Model
		PlaneModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_in));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(PlaneModel);	// ransac

	// set attributes...
	ransac.setDistanceThreshold(distanceThreshold);
	ransac.setMaxIterations(maxIters);
	ransac.setProbability(probility);
	ransac.computeModel();


	// get result
	ransac.getInliers(inliers);
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *cloud_out);

	return cloud_out;
}
