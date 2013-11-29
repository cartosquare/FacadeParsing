#include "pclib.h"
#include <fstream>


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

pclib::pclib(void)
{
}


pclib::~pclib(void)
{
}

 bool pclib::load_cloudType1_from_pcd(const std::string& filePath, CloudType1::Ptr cloud)
{
//	CloudType1::Ptr cloud(new CloudType1);
	if (pcl::io::loadPCDFile<PointType1>(filePath, *cloud) == -1) //* load the file
		return false;
	else
		return true;
}

bool pclib::load_cloudType2_from_pcd(const std::string& filePath, CloudType2::Ptr cloud)
{
	if (pcl::io::loadPCDFile<PointType2>(filePath, *cloud) == -1) //* load the file
		return false;
	else
		return true;
}
bool pclib::load_cloudType1_from_txt(const std::string& filePath, CloudType1::Ptr cloud)
{
	// fill in the cloud data
	std::fstream ifs(filePath);

	if(ifs.is_open())
	{
		while(!ifs.eof())
		{
			PointType1 point;
			ifs >> point.x >> point.y >> point.z;

			cloud->push_back(point);
		}

		ifs.close();

		return true;
	}
	return false;
}
bool pclib::load_cloudType2_from_txt(const std::string& filePath, CloudType2::Ptr cloud)
{
	// fill in the cloud data
	std::fstream ifs(filePath);

	if(ifs.is_open())
	{
		while(!ifs.eof())
		{
			PointType2 point;
			ifs >> point.x >> point.y >> point.z
				>> point.r >> point.g >> point.b;
			uint32_t rgb = ((uint32_t)point.r << 16 | (uint32_t)point.g << 8 | (uint32_t)point.b);
			point.rgb = *reinterpret_cast<float*>(&rgb);

			cloud->push_back(point);
		}

		ifs.close();

		return true;
	}
	else
		return false;
}

bool pclib::transfer_txt_to_pcd_type(const std::string& txtPath, const std::string& pcdPath, bool rgb)
{
	if (!rgb)
	{
		CloudType1::Ptr cloud(new CloudType1);
		if(!load_cloudType1_from_txt(txtPath, cloud))
			return false;
		else
		{
			pcl::io::savePCDFileBinary(pcdPath, *cloud);
			return true;
		}
	} 
	else
	{
		CloudType2::Ptr cloud(new CloudType2);
		if(!load_cloudType2_from_txt(txtPath, cloud))
			return false;
		else
		{
			pcl::io::savePCDFileBinary(pcdPath, *cloud);
			return true;
		}
	}
	return false;
}

bool pclib::transfer_pcd_to_txt_type(const std::string& pcdPath, const std::string& txtPath, bool rgb)
{
	if (!rgb)
	{
		CloudType1::Ptr cloud(new CloudType1);
		if(!load_cloudType1_from_pcd(pcdPath, cloud))
			return false;
		else
		{
			std::ofstream ofs(txtPath);
			for ( int i = 0; i < cloud->points.size(); ++i)
			{
				ofs << cloud->points[i].x << " " << cloud->points[i].y << " " 
					<< cloud->points[i].z << std::endl;
			}
			ofs.close();
			return true;
		}
	} 
	else
	{
		CloudType2::Ptr cloud(new CloudType2);
		if(!load_cloudType2_from_pcd(pcdPath, cloud))
			return false;
		else
		{
			std::ofstream ofs(txtPath);
			for ( int i = 0; i < cloud->points.size(); ++i)
			{
				ofs << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z 
					<< " " << cloud->points[i].r << " " << cloud->points[i].g << " " << cloud->points[i].b << std::endl;
			}
			ofs.close();
			
			return true;
		}
	}
	return false;
}
void pclib::render_cloudType1(CloudType1::Ptr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<PointType1> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void pclib::render_cloudType2(CloudType2::Ptr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointType2> rgb(cloud);
	viewer->addPointCloud<PointType2> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}
CloudType2::Ptr pclib::region_growing_segemtation_by_normal(
	CloudType1::Ptr cloud,	// 点云 
	int nKSearch,			// 使用多少个点计算法向量和曲率
	double minClusterSize,		// 分割完成后，小于多少的聚类要舍弃？（按占cloud的百分比计算）
	double maxClusterSize,		// 分割完成后，大于多少的聚类要舍弃？（按占cloud的百分比计算）
	int nNeighbors,					// 增长聚类时，考虑多少个邻居？

	// 两个向量间的角度容差（单位为弧度）
	double smoothnessThreshold, 
	// 如果两点之间的向量的角度差小于容差，就要比较曲率的差异
	double curvatureThreshold
	)
{
	pcl::search::Search<PointType1>::Ptr tree = 
		boost::shared_ptr<pcl::search::Search<PointType1> >(
		new pcl::search::KdTree<PointType1>);

	pcl::PointCloud<pcl::Normal>::Ptr normals(
		new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointType1, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(nKSearch);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<PointType1, pcl::Normal> reg;
	reg.setMinClusterSize(cloud->points.size() * minClusterSize);
	reg.setMaxClusterSize(cloud->points.size() * maxClusterSize);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(nNeighbors);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(smoothnessThreshold);
	reg.setCurvatureThreshold(curvatureThreshold);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);
	

	CloudType2::Ptr colored_cloud = reg.getColoredCloud();

	return colored_cloud;
}

CloudType1::Ptr pclib::radius_outliers_removal(CloudType1::Ptr inputCloud, double radius, int min_neighbors)
{
	CloudType1::Ptr filter(new CloudType1);

	pcl::RadiusOutlierRemoval<PointType1> outrem;
	outrem.setInputCloud(inputCloud);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(min_neighbors);
	outrem.filter(*filter);

	return filter;
}

CloudType1::Ptr pclib::passThrough_filter(
	CloudType1::Ptr inputCloud,		// input cloud
	const std::string& axe,			//  filter field name is which coordinate? can be: "x", "y" or "z"
	double minLimits,				// min range
	double maxLimits,				// max range
	bool inside						// inside or outside a given user range.
	)
{
	CloudType1::Ptr filtered (new CloudType1);

	// Create the filtering object
	pcl::PassThrough<PointType1> pass;
	pass.setInputCloud(inputCloud);
	pass.setFilterFieldName(axe);
	pass.setFilterLimits(minLimits, maxLimits);
	if(!inside)
		pass.setFilterLimitsNegative(true);
	pass.filter(*filtered);

	return (filtered);
}

CloudType1::Ptr pclib::my_downsample(CloudType1::Ptr inputCloud,
	double size_x, double size_y, double size_z)
{
	CloudType1::Ptr sample(new CloudType1);

	// create the filtering object
	pcl::VoxelGrid<PointType1> grid;
	grid.setInputCloud(inputCloud);
	grid.setLeafSize(size_x, size_y, size_z);
	grid.filter(*sample);

	return (sample);
}

CloudType1::Ptr pclib::statiscal_outlier_removal(
	CloudType1::Ptr inputCloud,
	int num_neighbors,	//  The number of neighbors to analyze for each point
	double stddevmul, // all points who have a distance larger than @stddevmul@ standard deviation of the mean distance 
	// to teh query point will be marked as outliers and removed.
	bool inliers
	)
{
	CloudType1::Ptr filterd(new CloudType1);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointType1> sor;
	sor.setInputCloud(inputCloud);
	sor.setMeanK(num_neighbors);
	sor.setStddevMulThresh(stddevmul);
	if(!inliers)
		sor.setNegative(true);
	sor.filter(*filterd);

	return (filterd);
}

CloudType1::Ptr pclib::random_sample_consensus(CloudType1::Ptr cloud_in, 
	double distanceThreshold, int maxIters, double probility)
{
	CloudType1::Ptr cloud_out(new CloudType1);	// output cloud
	std::vector<int> inliers;	// model indices

	pcl::SampleConsensusModelPlane<PointType1>::Ptr	// plane Model
		PlaneModel(new pcl::SampleConsensusModelPlane<PointType1>(cloud_in));

	pcl::RandomSampleConsensus<PointType1> ransac(PlaneModel);	// ransac

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