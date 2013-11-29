#include "cmd_utility.h"
#include "facade_model.h"
#include "boost/program_options.hpp"
using namespace std;

CommandLineArgument cmd_arguments;

std::shared_ptr<FacadeGrammar> facade_grammar;
std::shared_ptr<FacadeModel> facade_model;

bool ParseCommandLine(int argc, char** argv);

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

	cout << "\nLoad grids.\n";
	facade_model->LoadGrids(cmd_arguments.grid_);

	cout << "\nRevise grids.\n";
	for (int i = 0; i < facade_model->get_width(); ++i)
	{
		// up
		for (int j = 0; j < cmd_arguments.margin_[0]; ++j)
		{
			facade_model->SetProperty(i, j, 0, 1.0);
			facade_model->SetProperty(i, j, 1, 0.0);
		}

		// down
		int start = facade_model->get_height() - cmd_arguments.margin_[1];
		for (int j = start; j < facade_model->get_height(); ++j)
		{
			facade_model->SetProperty(i, j, 0, 1.0);
			facade_model->SetProperty(i, j, 1, 0.0);
		}
	}

	for (int i = 0; i < facade_model->get_height(); ++i)
	{
		// left
		for (int j = 0; j < cmd_arguments.margin_[2]; ++j)
		{
			facade_model->SetProperty(j, i, 0, 1.0);
			facade_model->SetProperty(j, i, 1, 0.0);
		}

		// right
		int start = facade_model->get_width() - cmd_arguments.margin_[3];
		for (int j = start; j < facade_model->get_width(); ++j)
		{
			facade_model->SetProperty(j, i, 0, 1.0);
			facade_model->SetProperty(j, i, 1, 0.0);
		}
	}

	cout << "\nRender grids.\n";
	facade_model->RenderGrd(cmd_arguments.revise_grid_);
	cout << "\nSave grids.\n";
	facade_model->SaveGrids(cmd_arguments.revise_grid_);

	cout << "\nDone.\n";
	return 0;
}