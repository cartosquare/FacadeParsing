#include "cmd_utility.h"
#include "facade_model.h"
#include "boost/program_options.hpp"
#include <pcl\point_types.h>
#include <pcl\io\pcd_io.h>

using namespace std;

CommandLineArgument cmd_arguments;

pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::shared_ptr<FacadeGrammar> facade_grammar;
std::shared_ptr<FacadeModel> facade_model;

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

	// Step 4 
	cout << "\nLoad point cloud ...\n";
	//pcl::io::loadPCDFile(cloud_file_path, *facade_cloud);

	cout << "\nCompute feature ...\n";
	facade_model->ComputeFeature2();

	// Step 5
	cout << "\nShow feature.\n";
	facade_model->ShowFeature();

	// Step 6
	double vertical_thread, horizontal_thread;
	std::cout << "Please input the vertical thread: ";
	std::cin >> vertical_thread;
	std::cout << "Please input the horizontal thread: ";
	std::cin >> horizontal_thread;

	// Step 7
	std::cout << "Calculate the action parameters ...\n";
	facade_model->CalculateActionParameters2(vertical_thread, horizontal_thread);

	// Step 8
	std::cout << "Save the action parameters.\n";
	facade_model->SaveActionParammeters2(cmd_arguments.action_parameters_debug_);
	 
	// Step 9 
	std::cout << "Generate validate picture.\n";
	facade_model->DebugCalculateFeature(cmd_arguments.action_parameters_png_);
	facade_model->ParseActionParameters();

	facade_model->SaveActionParammeters(cmd_arguments.action_parameters_);

	cout << "\nDone.\n";
	return 0;
}