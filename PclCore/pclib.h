#ifndef PCLIB_H_
#define PCLIB_H

#include "pcl\point_types.h"
#include "pcl\io\pcd_io.h"

#include <iostream>
#include <vector>

typedef pcl::PointXYZ				PointType1;
typedef pcl::PointXYZRGB			PointType2;
typedef pcl::PointCloud<PointType1> CloudType1;
typedef pcl::PointCloud<PointType2> CloudType2;

class pclib
{
public:
	pclib(void);
	~pclib(void);

	// I/O operations
	static bool load_cloudType1_from_pcd(const std::string& filePath, CloudType1::Ptr);
	static bool load_cloudType2_from_pcd(const std::string& filePath, CloudType2::Ptr);
	static bool load_cloudType1_from_txt(const std::string& filePath, CloudType1::Ptr);
	static bool load_cloudType2_from_txt(const std::string& filePath, CloudType2::Ptr);

	//static bool save_pcd(CloudType1::Ptr cloud, const std::string& filePath);
	static bool transfer_txt_to_pcd_type(const std::string& txtPath, const std::string& pcdPath, bool rgb = false); 
	static bool transfer_pcd_to_txt_type(const std::string& pcdPath, const std::string& txtPath, bool rgb = false); 

	// visualization
	static void render_cloudType1(CloudType1::Ptr cloud);
	static void render_cloudType2(CloudType2::Ptr cloud);

	/// filter
	static CloudType1::Ptr my_downsample(CloudType1::Ptr inputCloud,
		double size_x, double size_y, double size_z);

	static CloudType1::Ptr radius_outliers_removal(CloudType1::Ptr inputCloud, 
		double radius, int min_neighbors);

	static CloudType1::Ptr passThrough_filter(
		CloudType1::Ptr inputCloud,		// input cloud
		const std::string& axe,			//  filter field name is which coordinate? can be: "x", "y" or "z"
		double minLimits,				// min range
		double maxLimits,				// max range
		bool inside = true);			// inside or outside a given user range.

	static CloudType1::Ptr statiscal_outlier_removal(
		CloudType1::Ptr inputCloud,
		int num_neighbors,	//  The number of neighbors to analyze for each point
		double stddevmul = 1.0, // all points who have a distance larger than @stddevmul@ standard deviation of the mean distance 
		// to teh query point will be marked as outliers and removed.
		bool inliers = true
		);

	// segmentation
	static CloudType2::Ptr region_growing_segemtation_by_normal(
		CloudType1::Ptr cloud,	// 点云 
		int nKSearch,			// 使用多少个点计算法向量和曲率
		double minClusterSize = 0.001,		// 分割完成后，小于多少的聚类要舍弃？（按占cloud的百分比计算）
		double maxClusterSize = 0.2,		// 分割完成后，大于多少的聚类要舍弃？（按占cloud的百分比计算）
		int nNeighbors = 30,					// 增长聚类时，考虑多少个邻居？

		// 两个向量间的角度容差（单位为弧度）
		double smoothnessThreshold = (7.0 / 180.0 * M_PI), 
		// 如果两点之间的向量的角度差小于容差，就要比较曲率的差异
		double curvatureThreshold = 1.0
		);


	static CloudType1::Ptr random_sample_consensus(CloudType1::Ptr cloud_in, 
		double distanceThreshold, int maxIters, double probility);
};

#endif // PCLIB_H