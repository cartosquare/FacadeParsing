#include <iostream>
#include "facade_grammar.h"
#include "facade_model.h"
#include "opencv2/opencv.hpp"
#include <memory>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <assert.h>
using namespace std;

#define  SHOW_RESULT 0

// 0 - simple one dimension; 1 - complex one dimension; 2 - simple two dimension; 3 - complex two dimension
const int TEST_CASE = 2; 
int episodes = 50000;

int main() {
	srand(static_cast<unsigned int>(time(0)));

	std::string grammar_file;
	shared_ptr<FacadeGrammar> facade_grammar(new FacadeGrammar);

	switch (TEST_CASE)
	{
	case 0:
		{
			grammar_file = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple one dimension.xml";
			facade_grammar->ReadGrammar(grammar_file);

			shared_ptr<FacadeModel> facade_model(new FacadeModel(10, 5, facade_grammar));

			std::cout << "Configure facade geography ...\n";
			std::string path = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple_one_dimension.png";
			facade_model->ConfigureOneDimensionFacade(path);

			std::cout << "Running algorithm ... \n";
			facade_model->RunParsingAlgorithm(episodes);

			std::cout << "Rendering result ...\n";
			std::string result = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple_one_dimension_result.png";
			facade_model->RenderResult(result);

			break;
		}
	case 1:
		{
			grammar_file = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\complex one dimension.xml";
			facade_grammar->ReadGrammar(grammar_file);

			shared_ptr<FacadeModel> facade_model(new FacadeModel(210, 100, facade_grammar));

			std::cout << "Configure facade geography ...\n";
			std::string path = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\complex_one_dimension.png";
			facade_model->ConfigureComplexOneDimensioFacade(path);

			std::cout << "Running algorithm ... \n";
			facade_model->RunParsingAlgorithm(episodes);

			std::cout << "Rendering result ...\n";
			std::string result = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\complex_one_dimension_result.png";
			facade_model->RenderResult(result);

			break;
		}
	case 2:
		{
			grammar_file = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple two dimension.xml";
			facade_grammar->ReadGrammar(grammar_file);

			shared_ptr<FacadeModel> facade_model(new FacadeModel(25, 25, facade_grammar));

			std::cout << "Configure facade geography ...\n";
			std::string path = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple_two_dimension.png";
			facade_model->ConfigureTwoDimensionFacade(path);
			//std::string grid_path = "grid";
			//facade_model->SaveGrids(grid_path);

			std::cout << "Running algorithm ... \n";
			facade_model->RunParsingAlgorithm(episodes);

			std::cout << "Rendering result ...\n";
			std::string result = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\simple_two_dimension_result.png";
			facade_model->RenderResult(result);

			break;
		}
	case 3:
		{


			grammar_file = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\complex two dimension.xml";
			facade_grammar->ReadGrammar(grammar_file);

			// ..........................
			// ............................
			break;
		}
	default:
		break;
	}

#if 0 // validate the CalculateScore function

	std::string grid_path = "grid";
	facade_model->SaveGrids(grid_path);

	std::shared_ptr<ShapeSymbol> wall1(new ShapeSymbol(0, 0, 4, 5, "Wall", facade_grammar));
	assert(facade_model->CalculateScore(wall1) == 20);

	std::shared_ptr<ShapeSymbol> wall2(new ShapeSymbol(7, 0, 3, 5, "Wall", facade_grammar));
	assert(facade_model->CalculateScore(wall2) == 15);

	std::shared_ptr<ShapeSymbol> window(new ShapeSymbol(4, 0, 3, 5, "Window", facade_grammar));
	assert(facade_model->CalculateScore(window) == 15);

	std::shared_ptr<ShapeSymbol> wall_window(new ShapeSymbol(4, 0, 6, 5, "Window", facade_grammar));
	assert(facade_model->CalculateScore(wall_window) == 15);
#endif



	std::cout << "Done.\n";

#if SHOW_RESULT
	std::cout << "Showing result ...\n";
	cv::namedWindow("Learning result");
	cv::namedWindow("Original facade");
	cv::Mat image = cv::imread(result);//, CV_LOAD_IMAGE_COLOR);
	cv::Mat original_image = cv::imread(path);//, CV_LOAD_IMAGE_GRAYSCALE);
	cv::imshow("Original facade", original_image);
	cv::imshow("Learning result", image);

	cv::waitKey();
#endif

	return 0;
}