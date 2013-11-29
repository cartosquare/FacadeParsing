#include "cmd_utility.h"
#include "facade_grammar.h"
#include "facade_model.h"

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
	cout << "\nInitialize the facade model, ";
	facade_model = std::shared_ptr<FacadeModel>(new FacadeModel(facade_grammar));

	// Step 3
	cout << "\nLoad the facade grid ...\n";
	facade_model->LoadGrids(cmd_arguments.revise_grid_);
	
	// Step 4
	std::cout << "\nRunning algorithm ... \n";
	facade_model->set_action_parameter_file(cmd_arguments.action_parameters_);
	facade_model->InitModel();
	facade_model->set_qtable_path(cmd_arguments.qtable_.c_str());

	if (cmd_arguments.load_qtable_)
	{
		facade_model->LoadQTable();
	}
	facade_model->RunParsingAlgorithm(cmd_arguments.episodes_);

	// Step 5
	std::cout << "\nRendering result ...\n";
	facade_model->RenderResult(cmd_arguments.result_png_);
	facade_model->SaveLearningResult(cmd_arguments.result_txt_);

	return 0;
}