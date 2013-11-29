
#include "facade_grammar.h"
#include "facade_model.h"
#include "cmd_utility.h"
#include "pcl\io\pcd_io.h"
#include "opencv2/opencv.hpp"
#include "boost/program_options.hpp"

#include <iostream>
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <assert.h>

using namespace std;

CommandLineArgument cmd_arguments;

pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::shared_ptr<FacadeGrammar> facade_grammar;
std::shared_ptr<FacadeModel> facade_model;

// Main function
int main(int argc, char** argv)
{
	// Step 0
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	// Step 1
	cout << "\nLoad the shape grammar.\n";
	facade_grammar = std::shared_ptr<FacadeGrammar>(new FacadeGrammar);
	facade_grammar->ReadGrammar(cmd_arguments.grammar_);

	// Step 2
	cout << "Initialize the facade model, ";
	facade_model = std::shared_ptr<FacadeModel>(new FacadeModel(facade_grammar));

	// Step 3
	cout << "\nLoad the facade point cloud.\n";
	pcl::io::loadPCDFile(cmd_arguments.rt_pcd_, *facade_cloud);

	// Step 4
	cout << "\nConstruct the facade grid ...\n";
	facade_model->CreateGridFromPointCloud(facade_cloud, cmd_arguments.resolution_);

	facade_model->RenderGrd(cmd_arguments.grid_);

	facade_model->SaveGrids(cmd_arguments.grid_);

	return 0;
}