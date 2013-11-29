#include "cmd_utility.h"
#include "pcl\io\pcd_io.h"
#include "pcl\filters\statistical_outlier_removal.h"
#include "pcl\filters\radius_outlier_removal.h"
#include <fstream>
#include <iostream>
using namespace std;

CommandLineArgument cmd_arguments;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove_outlier;

pcl::PointCloud<pcl::PointXYZ>::Ptr radius_outliers_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
															double radius, int min_neighbors);

pcl::PointCloud<pcl::PointXYZ>::Ptr statiscal_outlier_removal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
	int num_neighbors,	//  The number of neighbors to analyze for each point
	double stddevmul, // all points who have a distance larger than @stddevmul@ standard deviation of the mean distance 
	// to the query point will be marked as outliers and removed.
	bool inliers
	);


int main(int argc, char** argv) 
{
	/* Step 0 */
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	/* Step 1 */
	cout << "\n=========== Load raw building point cloud ... ==================\n";
	pcl::io::loadPCDFile(cmd_arguments.raw_pcd_ , *cloud);

	/* Step 2 */
	std::cout << "\n=============== Remove outliers ... =======================\n";
	if (cmd_arguments.ready_radius_outlier_remover_)
		cloud_remove_outlier = radius_outliers_removal(cloud, cmd_arguments.radius_outlier_remover_[0],
		cmd_arguments.radius_outlier_remover_[1]);
	else if(cmd_arguments.ready_statiscal_outlier_remover_)
		cloud_remove_outlier = statiscal_outlier_removal(cloud, cmd_arguments.statiscal_outlier_remover_[0],
		cmd_arguments.statiscal_outlier_remover_[1], true);

	// Step 3
	std::cout << "Save ... \n";
	pcl::io::savePCDFile(cmd_arguments.raw_pcd_, *cloud);

	return 0;
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
