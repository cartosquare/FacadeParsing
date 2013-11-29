
#include "cmd_utility.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl\io\pcd_io.h>
#include "boost/program_options.hpp"
#include "facade_grammar.h"
#include "facade_model.h"
#include "opencv2/opencv.hpp"

#include <fstream>
using namespace std;

CommandLineArgument cmd_arguments;

std::shared_ptr<FacadeGrammar> facade_grammar;
std::shared_ptr<FacadeModel> facade_model;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZ>::Ptr build_cloud(new pcl::PointCloud<pcl::PointXYZ>);
double** grid;

struct ResultSymbol
{
	std::string name_;
	int pos_x_, pos_y_, width_, height_;
};

std::vector<ResultSymbol> result_symbols;
int symbol_number;

double original_x, original_y;
double resolution;
int width, height;
double zmin = -3.0;
double zmax = 3.0;

void AddSymbols();
void MyAddCube(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, 
			   double r, double g, double b, double transparent, std::string id) ;
void ConstructBuildBox();
void DebugLearningResult(std::string filename);

int main(int argc, char** argv) {
	// Step 0
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	// Step 1
	cout << "\nLoad the shape grammar.\n";
	facade_grammar = std::shared_ptr<FacadeGrammar>(new FacadeGrammar);
	facade_grammar->ReadGrammar(cmd_arguments.grammar_);

	// Step 2
	cout << "\nInitialize the facade model, ";
	facade_model = std::shared_ptr<FacadeModel>(new FacadeModel(facade_grammar));

	// Step 3
	cout << "\nLoad grids.\n";
	facade_model->LoadGrids(cmd_arguments.revise_grid_);

	// Load the symbols
	ifstream ifs(cmd_arguments.result_txt_.c_str());	
	if (!ifs.is_open())
	{
		return -1;
	}
	ifs >> symbol_number;
	ifs >> original_x >> original_y >> resolution >> width >> height;
	result_symbols.reserve(symbol_number);

	for (int i = 0; i < symbol_number; ++i)
	{
		ResultSymbol symbol;
		ifs >> symbol.name_ >> symbol.pos_x_ >> symbol.pos_y_ >> symbol.width_ >> symbol.height_;
		result_symbols.push_back(symbol);
	}
	ifs.close();

	// 
	grid = facade_model->get_grd_ptr(1);
	DebugLearningResult(cmd_arguments.original_result_overlap_png_);

	// load the building cloud.
	pcl::io::loadPCDFile(cmd_arguments.rt_pcd_, *build_cloud);

	// Open the viewer and add the symbols to the viewer
	viewer.reset (new pcl::visualization::PCLVisualizer());
	viewer->addPointCloud<pcl::PointXYZ> (build_cloud, "build plane cloud");
	viewer->setBackgroundColor (0, 0, 0);
	AddSymbols();

	viewer->spin ();
}

void DebugLearningResult(std::string filename) {
	cv::Mat mat(height, width, CV_8UC3);
	cv::Mat mat_learning(height, width, CV_8UC3);
	cv::Mat mat_compare(height, width, CV_8UC3);

	cv::Mat_<cv::Vec3b> _Mat = mat;

	for ( int i = 0; i < height; ++i)
	{
		for ( int j = 0; j < width; ++j)
		{
			_Mat(i, j)[0] = (uchar)(grid[i][j] * 255);
			_Mat(i, j)[1] = (uchar)(grid[i][j] * 255);
			_Mat(i, j)[2] = (uchar)(grid[i][j] * 255);
		}
	}

	for (int i = 0; i < symbol_number; ++i)
	{
		ResultSymbol symbol = result_symbols[i];

		float x_min = symbol.pos_x_;
		float x_max = x_min + symbol.width_;
		float y_min = symbol.pos_y_;
		float y_max = y_min + symbol.height_;

		if (symbol.name_ == "Wall")
		{
			cv::rectangle(mat_learning, cv::Point(x_min, y_min), cv::Point(x_max, y_max), 
				cv::Scalar(0, 255, 255), -1, 8);
		}
		else
		{
			cv::rectangle(mat_learning, cv::Point(x_min, y_min), cv::Point(x_max, y_max), 
				cv::Scalar(0, 0, 255), -1, 8);
		}
	}

	cv::addWeighted(_Mat, 0.5, mat_learning, 0.5, 0.0, mat_compare);

	imwrite(filename, mat_compare);
}

void AddSymbols()
{
	int count = 0;
	double transparent = 0.5;

	for (int i = 0; i < symbol_number; ++i)
	{
		ResultSymbol symbol = result_symbols[i];

		float x_min = original_x + symbol.pos_x_ * resolution;
		float x_max = x_min + symbol.width_ * resolution;
		float y_min = original_y + symbol.pos_y_ * resolution;
		float y_max = y_min + symbol.height_ * resolution;

		count++;
		ostringstream id;

		if ( symbol.name_ == "Window" )
		{
			id << "window " << count;
			MyAddCube(x_min, x_max, y_min, y_max, zmin, zmax, 0.0, 0.0, 1.0, transparent, id.str());
		}
		else // Wall
		{
			if(symbol.width_ == width)
			{
				// flat wall
				id << "flat wall " << count;
				MyAddCube(x_min, x_max, y_min, y_max, zmin, zmax, 0.0, 1.0, 0.0, transparent, id.str());
			}
			else
			{
				// floor wall
				id << "wall " << count;
				MyAddCube(x_min, x_max, y_min, y_max, zmin, zmax, 0.0, 1.0, 1.0, transparent, id.str());
			}
		}
	}
	ConstructBuildBox();
}

void MyAddCube(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax, 
			   double r, double g, double b, double transparent, std::string id) 
{
	pcl::PointXYZ p1(xmin, ymin, zmin), p2(xmin, ymax, zmin), p3(xmax, ymax, zmin), 
		p4(xmax, ymin, zmin), p5(xmin, ymin, zmax), p6(xmin, ymax, zmax), p7(xmax, ymax, zmax), p8(xmax, ymin, zmax);

	pcl::PointCloud<pcl::PointXYZ>::Ptr facade1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr facade2(new pcl::PointCloud<pcl::PointXYZ>);
	
	facade1->width = 4;
	facade1->height = 1;
	facade2->width = 4;
	facade2->height = 1;

	// front
	facade1->push_back(p1);
	facade1->push_back(p2);
	facade1->push_back(p3);
	facade1->push_back(p4);

	// back
	facade2->push_back(p5);
	facade2->push_back(p6);
	facade2->push_back(p7);
	facade2->push_back(p8);

	Eigen::Vector4f planarCoeff;
	planarCoeff(0) = 0.0;
	planarCoeff(1) = 0.0;
	planarCoeff(2) = 1.0;
	planarCoeff(3) = 0.0;
	pcl::PlanarPolygon<pcl::PointXYZ> facade1_planar(facade1->points, planarCoeff);
	pcl::PlanarPolygon<pcl::PointXYZ> facade2_planar(facade2->points, planarCoeff);

	std::string facade_name = id + "_front";
	viewer->addPolygon(facade1_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);

	facade_name = id + "_front_outline";
	viewer->addPolygon(facade1_planar, 1.0, 0.0, 0.0, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, facade_name);

	facade_name = id + "_back";
	viewer->addPolygon(facade2_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);

	facade_name = id + "_back_outline";
	viewer->addPolygon(facade2_planar, 1.0, 0.0, 0.0, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, facade_name);
}

void ConstructBuildBox() {
	float x_min = original_x;
	float x_max = x_min + width* resolution;
	float y_min = original_y;
	float y_max = y_min + height * resolution;
	pcl::PointXYZ p1(x_min, y_min, zmin), p2(x_min, y_max, zmin), p3(x_max, y_max, zmin), 
		p4(x_max, y_min, zmin), p5(x_min, y_min, zmax), p6(x_min, y_max, zmax),
		p7(x_max, y_max, zmax), p8(x_max, y_min, zmax);

	pcl::PointCloud<pcl::PointXYZ>::Ptr facade3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr facade4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr facade5(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr facade6(new pcl::PointCloud<pcl::PointXYZ>);

	facade3->width = 4;
	facade3->height = 1;
	facade4->width = 4;
	facade4->height = 1;
	facade5->width = 4;
	facade5->height = 1;
	facade6->width = 4;
	facade6->height = 1;

	// left
	facade3->push_back(p1);
	facade3->push_back(p2);
	facade3->push_back(p6);
	facade3->push_back(p5);

	// right
	facade4->push_back(p3);
	facade4->push_back(p4);
	facade4->push_back(p8);
	facade4->push_back(p7);

	// top
	facade5->push_back(p2);
	facade5->push_back(p6);
	facade5->push_back(p7);
	facade5->push_back(p3);

	// bottom
	facade6->push_back(p1);
	facade6->push_back(p5);
	facade6->push_back(p8);
	facade6->push_back(p4);

	Eigen::Vector4f planarCoeff;
	planarCoeff(0) = 0.0;
	planarCoeff(1) = 0.0;
	planarCoeff(2) = 1.0;
	planarCoeff(3) = 0.0;
	pcl::PlanarPolygon<pcl::PointXYZ> facade3_planar(facade3->points, planarCoeff);
	pcl::PlanarPolygon<pcl::PointXYZ> facade4_planar(facade4->points, planarCoeff);
	pcl::PlanarPolygon<pcl::PointXYZ> facade5_planar(facade5->points, planarCoeff);
	pcl::PlanarPolygon<pcl::PointXYZ> facade6_planar(facade6->points, planarCoeff);
	
	double r = 0.5;
	double g = 0.5;
	double b = 0.5;
	double transparent = 1.0;

	std::string facade_name = "left facade";
	viewer->addPolygon(facade3_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);

	facade_name = "right facade";
	viewer->addPolygon(facade4_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);

	facade_name = "top facade";
	viewer->addPolygon(facade5_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);

	facade_name = "bottom facade";
	viewer->addPolygon(facade6_planar, r, g, b, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, facade_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, transparent, facade_name);
}