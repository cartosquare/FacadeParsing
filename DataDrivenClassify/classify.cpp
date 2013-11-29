#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

void RemovePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<int>& indices, 
				  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
	cloud_out->width = cloud_in->width - indices.size();
	cloud_out->height = cloud_in->height;
	bool* remove = new bool[cloud_in->points.size()];
	for (int i = 0; i < cloud_in->points.size(); ++i)
	{
		remove[i] = false;
	}
	for (int i = 0; i < indices.size(); ++i)
	{
		remove[indices[i]] = true;
	}

	for (int i = 0; i < cloud_in->points.size(); ++i)
	{
		if (!remove[i])
		{
			pcl::PointXYZ point;
			point.x = cloud_in->points[i].x;
			point.y = cloud_in->points[i].y;
			point.z = cloud_in->points[i].z;
			cloud_out->points.push_back(point);
		}
	}
	delete[] remove;
}

void ExtractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::vector<int> inliers;	// build plane indices in the raw building point cloud
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		PlaneModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // plane Model
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(PlaneModel);	// ransac
	std::ostringstream oss;
	int count = 0;
	ransac.setDistanceThreshold(0.01);
	std::cout << "compute model.\n";
	while (ransac.computeModel())
	{
		std::cout << "compute done.\n";
		++count;
		oss << count << ".pcd";

		ransac.getInliers(inliers);

		std::cout << "plane " << oss.str() << " has " << inliers.size() << " inliers.\n";
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_plane);
		pcl::io::savePCDFile(oss.str(), *cloud_plane);
		std::cout << oss.str() << " saved\n";

		std::cout << "remove the plane from cloud.\n";
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce(new pcl::PointCloud<pcl::PointXYZ>);
		RemovePoints(cloud, inliers, cloud_reduce);
		std::cout << "remove done.\n";

		inliers.clear();
		PlaneModel = pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr(new 
			pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_reduce));
		ransac.setSampleConsensusModel(PlaneModel);
		ransac.setDistanceThreshold(0.01);
		std::cout << "compute model.\n";
	}
}

int main (int argc, char** argv)
{
	pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
#if 1
	std::cout << "Reading pcd ... \n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce (new pcl::PointCloud<pcl::PointXYZ>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("rt.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	
	std::cout << "Extract plane ...\n";
	std::vector<int> inliers;	// build plane indices in the raw building point cloud
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		PlaneModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // plane Model
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(PlaneModel);	// ransac
	// set attributes...
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();

	Eigen::VectorXf build_plane_coeff(4);
	ransac.getModelCoefficients(build_plane_coeff);

	// get result
	ransac.getInliers(inliers);

	std::cout << "save building plane...\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *build_cloud_plane);
	pcl::io::savePCDFile("build_plane.pcd", *build_cloud_plane);

	std::cout << "Remove plane cloud ... \n";
	std::cout << "width = " << cloud->width << " height = " << cloud->height << std::endl;
	std::cout << "original #points = " << cloud->points.size() << std::endl;
	// remove the inliers from the cloud.
	RemovePoints(cloud, inliers, cloud_reduce);
	std::cout << "after remove, #points = " << cloud_reduce->points.size() << std::endl;
	pcl::io::savePCDFile("rt_reduce.pcd", *cloud_reduce);
#endif

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduce (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile <pcl::PointXYZ> ("rt.pcd", *cloud_reduce);*/

	std::cout << "Compute normal ... \n";
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud_reduce);
	normal_estimator.setKSearch (30);
	normal_estimator.compute (*normals);

	//pcl::IndicesPtr indices (new std::vector <int>);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (0.0, 1.0);
	//pass.filter (*indices);

	std::cout << "Region growing ... \n";
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (10);
	reg.setMaxClusterSize (10000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud_reduce);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
	int count = 0;
	for (size_t i = 0; i < clusters.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud_reduce, clusters[i], *cluster_cloud);

		std::vector<int> cluster_inliers;	
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
			cluster_PlaneModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cluster_cloud)); 
		pcl::RandomSampleConsensus<pcl::PointXYZ> cluster_ransac(cluster_PlaneModel);
		// set attributes...
		cluster_ransac.setDistanceThreshold(0.01);
		cluster_ransac.computeModel();

		Eigen::VectorXf cluster_plane_coeff(4);
		cluster_ransac.getModelCoefficients(cluster_plane_coeff);

		double coeff_a = cluster_plane_coeff(0) / build_plane_coeff(0);
		double coeff_b = cluster_plane_coeff(1) / build_plane_coeff(1);
		double coeff_c = cluster_plane_coeff(2) / build_plane_coeff(2);
		if (std::fabs(coeff_a - coeff_b) < 0.1 && std::fabs(coeff_a - coeff_c) < 0.1)
		{
			count++;
			std::ostringstream oss;
			oss << count << ".pcd";
			// get result
			cluster_ransac.getInliers(cluster_inliers);
			pcl::PointCloud<pcl::PointXYZ>::Ptr per_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud<pcl::PointXYZ>(*cluster_cloud, clusters[i], *per_cloud);
			pcl::io::savePCDFile(oss.str(), *per_cloud);

			viewer.addPointCloud(per_cloud, oss.str());
		}

	}

	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	//viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}

	std::cout << "Done.\n";
	return (0);
}