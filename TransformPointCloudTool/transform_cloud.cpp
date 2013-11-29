#include "cmd_utility.h"

// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

// Global visualizer object
#if VTK_MAJOR_VERSION==6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
pcl::visualization::PCLPlotter ph_global;
#endif
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
CommandLineArgument cmd_arguments;
pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_transformed_plane(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_accurate_plane(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_transformed_raw(new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ> search;

bool transform = true;
int pick_time = 0;
std::vector<pcl::PointXYZ> picked_points;
Eigen::VectorXf build_plane_coeff(4);
pcl::PointXYZ min_extent, max_extent;

// Functions
void pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie);
void FindPickedPoint(const pcl::visualization::PointPickingEvent& event);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookie);
void TransformCloud();
void ExtractBuildPlane();
void Create_building_plane_bounder_box_and_add_to_viewer();
void ShowCloud();

int main (int argc, char** argv)
{
	/* Step 0 */
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	/* Step 1 */
	cout << "\n=========== Load raw building point cloud ... ==================\n";
	pcl::io::loadPCDFile(cmd_arguments.raw_pcd_, *build_cloud_raw);

	/* Step 2 */
	cout << "\n=========== Extract the building plane ... ==================\n";
	ExtractBuildPlane();

	/* Step 3 */
	cout << "\n=========== Showing the building plane ... ==================\n";
	ShowCloud();
}

void FindPickedPoint(const pcl::visualization::PointPickingEvent& event) {
	int idx = event.getPointIndex ();
	if (idx == -1)
	{
		std::cout << "Invalid pick!\n;";
		return;
	}
	search.setInputCloud(build_cloud_accurate_plane);

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
		viewer->removeAllShapes();
		picked_points.clear();

		FindPickedPoint(event);

		pick_time = 1;

		cout << "Original point Chosen! [" << picked_points[0].x << ", " 
			<< picked_points[0].y << ", " << picked_points[0].z << "]\n";
	}
	else if(pick_time == 1)
	{
		FindPickedPoint(event);

		pick_time = 2;

		cout << "First point chosen![" << picked_points[1].x << ", " 
			<< picked_points[1].y << ", " << picked_points[1].z << "]\n";
	}
	else
	{
		FindPickedPoint(event);
		viewer->addArrow(picked_points[2], picked_points[1], 1.0, 0.0, 0.0, "arrow");

		pick_time = 0;

		cout << "Second point chosen! And it's ready to transform.[" << picked_points[2].x << ", " 
			<< picked_points[2].y << ", " << picked_points[2].z << "]\n";
	}
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* cookie) {
	if(event.getKeySym() == "c" && transform)
	{
		cout << "Transform ... \n";
		TransformCloud();
		cout << "Transform Done.\n";

		transform = false;
	}
}

void ExtractBuildPlane() {
	std::vector<int> inliers;	// build plane indices in the raw building point cloud
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		PlaneModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(build_cloud_raw)); // plane Model
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(PlaneModel);	// ransac
	// set attributes...
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();

	// get result
	ransac.getInliers(inliers);
	// copies all inliers of the model computed to another PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*build_cloud_raw, inliers, *build_cloud_plane);

	// get plane model
	ransac.getModelCoefficients(build_plane_coeff);

	// project the points to the plane to reduce error
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = build_plane_coeff(0);
	coefficients->values[1] = build_plane_coeff(1);
	coefficients->values[2] = build_plane_coeff(2);
	coefficients->values[3] = build_plane_coeff(3);

	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(build_cloud_plane);
	proj.setModelCoefficients(coefficients);
	proj.filter(*build_cloud_accurate_plane);
}

void TransformCloud() {
	/// get the original point, X and Y direction normal of the new coordinate
	Eigen::Affine3f transform;
	Eigen::Vector3f y_direction, z_axis, origin;
	origin(0) = picked_points[0].x;
	origin(1) = picked_points[0].y;
	origin(2) = picked_points[0].z;
	y_direction(0) = picked_points[2].x - picked_points[1].x;
	y_direction(1) = picked_points[2].y - picked_points[1].y;
	y_direction(2) = picked_points[2].z - picked_points[1].z;
	z_axis(0) = build_plane_coeff(0);
	z_axis(1) = build_plane_coeff(1);
	z_axis(2) = build_plane_coeff(2);
	y_direction.normalize();
	z_axis.normalize();

	double result2 = y_direction(0) * z_axis(0) + y_direction(1) * z_axis(1)
		+ y_direction(2) * z_axis(2);
	cout << "Test orthogonal: " << result2 << endl;

	// Get transformation matrix
	pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_direction,
		z_axis, origin, transform);

	// transform the cloud
	pcl::transformPointCloud(*build_cloud_accurate_plane,
		*build_cloud_transformed_plane, transform);

	pcl::transformPointCloud(*build_cloud_raw,
		*build_cloud_transformed_raw, transform);

	// save...

		pcl::io::savePCDFileASCII(cmd_arguments.pt_pcd_, *build_cloud_transformed_plane);
		cout << cmd_arguments.pt_pcd_ << " is saved.\n";

	
		pcl::io::savePCDFileASCII(cmd_arguments.rt_pcd_, *build_cloud_transformed_raw);
		cout << cmd_arguments.rt_pcd_ << " is saved.\n";

	viewer->updatePointCloud(build_cloud_transformed_plane, "build plane cloud");
	viewer->initCameraParameters();
	viewer->removeAllShapes();

	// get the bounder box of the transformed cloud
	Create_building_plane_bounder_box_and_add_to_viewer();
}

void Create_building_plane_bounder_box_and_add_to_viewer()
{
	// get the bounder box of the transformed cloud
	pcl::getMinMax3D(*build_cloud_transformed_plane, min_extent, max_extent);

	pcl::PointCloud<pcl::PointXYZ>::Ptr build_polygon_points(new pcl::PointCloud<pcl::PointXYZ>);
	build_polygon_points->width = 4;
	build_polygon_points->height = 1;
	pcl::PointXYZ p1(min_extent.x, min_extent.y, 0.0), p2(min_extent.x, max_extent.y, 0.0), 
		p3(max_extent.x, max_extent.y, 0.0), p4(max_extent.x, min_extent.y, 0.0);

	build_polygon_points->points.push_back(p1);
	build_polygon_points->points.push_back(p2);
	build_polygon_points->points.push_back(p3);
	build_polygon_points->points.push_back(p4);

	Eigen::Vector4f planarCoeff;
	planarCoeff(0) = 0.0;
	planarCoeff(1) = 0.0;
	planarCoeff(2) = 1.0;
	planarCoeff(3) = 0.0;
	pcl::PlanarPolygon<pcl::PointXYZ> planarPolygon(build_polygon_points->points, planarCoeff);

	// And add the bounder box to the viewer
	const std::string facadeId = "facade";
	viewer->addPolygon(planarPolygon, 0.0, 0.0, 1.0, facadeId);

	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facadeId);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, facadeId);

	cout << "\nSummary: \n";
	cout << "Facade Extent: [ minx, miny, minz, maxx, maxy, maxz] == [ " <<
		min_extent.x << ", " << min_extent.y << ", " <<  min_extent.z << ", "
		<< max_extent.x << ", " << max_extent.y << ", " << max_extent.z << " ]" << endl;
}

void ShowCloud() {
	viewer.reset (new pcl::visualization::PCLVisualizer());
	viewer->addPointCloud<pcl::PointXYZ> (build_cloud_accurate_plane, "build plane cloud");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerPointPickingCallback(&pp_callback);
	viewer->registerKeyboardCallback(&keyboardEventOccurred);
	viewer->spin ();
}