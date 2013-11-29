#ifndef RL_UTILITY_TEST_H
#define RL_UTILITY_TEST_H

#include "gtest/gtest.h"
#include "rl_utility.h"
#include "facade_grammar.h"

using namespace RL;

TEST(RL_Utility_Test, Test_Max) {
	double array[9] = {1, 2, 3, 4, 5, 4, 3, 2, 1};

	EXPECT_TRUE(RLUtility::Max(array, 9) == 5);
}

TEST(RL_Utility_Test, Test_Max_With_Index) {
	double array[9] = {1, 2, 3, 4, 5, 4, 3, 2, 1};
	std::vector<int> index;
	for (size_t i = 0; i < 4; ++i)
	{
		index.push_back(i);
	}

	EXPECT_TRUE(RLUtility::Max(array, index) == 4);
}
TEST(RL_Utility_Test, Test_ArgMax) {
	double array[9] = {1, 2, 3, 4, 5, 4, 3, 2, 1};

	EXPECT_TRUE(RLUtility::ArgMax(array, 9) == 4);
}

TEST(RL_Utility_Test, Test_ArgMax_With_Index) {
	double array[9] = {1, 2, 3, 4, 5, 4, 3, 2, 1};
	std::vector<int> index;
	for (size_t i = 0; i < 4; ++i)
	{
		index.push_back(i);
	}

	EXPECT_TRUE(RLUtility::ArgMax(array, index) == 3);
}

TEST(RL_Utility_Test, Test_MaxALL) {
	double array[10] = {1, 2, 3, 4, 5, 4, 3, 2, 1, 5};
	std::vector<double> max_all = RLUtility::MaxAll(array, 10);
	EXPECT_TRUE(max_all.size() == 2);
	EXPECT_TRUE(max_all[0] == 5);
	EXPECT_TRUE(max_all[1] == 5);
}

TEST(RL_Utility_Test, Test_MaxALL_With_Index) {
	double array[10] = {1, 2, 3, 4, 5, 4, 3, 2, 1, 5};
	std::vector<int> index;
	index.push_back(0);
	index.push_back(1);
	index.push_back(2);
	index.push_back(3);
	index.push_back(5);
	index.push_back(6);
	index.push_back(7);
	index.push_back(8);
	std::vector<double> max_all = RLUtility::MaxAll(array, index);
	EXPECT_TRUE(max_all.size() == 2);
	EXPECT_TRUE(max_all[0] == 4);
	EXPECT_TRUE(max_all[1] == 4);
}

TEST(RL_Utility_Test, Test_ArgMaxALL) {
	double array[10] = {1, 2, 3, 4, 5, 4, 3, 2, 1, 5};
	std::vector<int> max_all = RLUtility::ArgMaxAll(array, 10);
	EXPECT_TRUE(max_all.size() == 2);
	EXPECT_TRUE(max_all[0] == 4);
	EXPECT_TRUE(max_all[1] == 9);
}

TEST(RL_Utility_Test, Test_ArgMaxALL_With_Index) {
	double array[10] = {1, 2, 3, 4, 5, 4, 3, 2, 1, 5};
	std::vector<int> index;
	index.push_back(0);
	index.push_back(1);
	index.push_back(2);
	index.push_back(3);
	index.push_back(5);
	index.push_back(6);
	index.push_back(7);
	index.push_back(8);
	std::vector<int> max_all = RLUtility::ArgMaxAll(array, index);
	EXPECT_TRUE(max_all.size() == 2);
	EXPECT_TRUE(max_all[0] == 3);
	EXPECT_TRUE(max_all[1] == 5);
}

TEST(RL_Utility_Test, Test_Rand_0A) {
	std::srand(static_cast<unsigned int>(std::time(0)));
	double results[10];
	int experiment_times = 10000;
	for (int i = 0; i < 10; ++i)
	{
		results[i] = 0;
	}

	for (int i = 0; i < experiment_times; ++i)
	{
		int random = RLUtility::Rand0A(10);
		EXPECT_TRUE(random >= 0);
		EXPECT_TRUE(random < 10);
		results[random]++;
	}

	for (int i = 0; i < 10; ++i)
	{
		 results[i] = (results[i] / experiment_times);
		 EXPECT_TRUE(std::fabs(results[i] - 0.1) < 0.1);
	}
}

TEST(RL_Utility_Test, Test_Rand_AB) {
	std::srand(static_cast<unsigned int>(std::time(0)));
	double results[20];
	int experiment_times = 10000;
	int A(10), B(19);
	for (int i = A - 1; i < B - 1; ++i)
	{
		results[i] = 0;
	}

	for (int i = 0; i < experiment_times; ++i)
	{
		int random = RLUtility::RandAB(A, B);
		EXPECT_TRUE(random >= A);
		EXPECT_TRUE(random < B);
		results[random - 1]++;
	}

	for (int i = A - 1; i < B - 1; ++i)
	{
		results[i] = results[i] / experiment_times;
		EXPECT_TRUE(std::fabs(results[i] - 0.1) < 0.1);
	}
}

TEST(RL_Utility_Test, Test_RandUnit) {
	unsigned int seed = static_cast<unsigned int>(std::time(0));
	std::cout << "Seed: " << seed << std::endl;
	std::srand(seed);
	double results[10];
	int experiment_times = 10000;
	for (int i = 0; i < 10; ++i)
	{
		results[i] = 0;
	}

	for (int i = 0; i < experiment_times; ++i)
	{
		double random = RLUtility::RandUnit();
		EXPECT_TRUE(random >= 0.0);
		EXPECT_TRUE(random <= 1.0);
		results[int(random * 10)]++;
	}


	for (int i = 0; i < 10; ++i)
	{
		double random = RLUtility::RandUnit();
		std::cout << random << " ";
	}
	std::cout << std::endl;
}


TEST(Test_SG_Test, Test_Read_Grammar) {
	std::string grammar_path = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\shape grammar.xml";

	FacadeGrammar facade_grammar;
	facade_grammar.ReadGrammar(grammar_path);

	EXPECT_TRUE(facade_grammar.get_atom_symbol() == "Facade");
	EXPECT_TRUE(facade_grammar.get_terminal_symbol_size() == 2);
	EXPECT_TRUE(facade_grammar.get_terminal_symbol(0) == "Wall");
	EXPECT_TRUE(facade_grammar.get_terminal_symbol(1) == "Window");
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[0] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[1] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[2] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[0] == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[1] == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[2] == 255);


	EXPECT_TRUE(facade_grammar.get_grammar_number() == 6);
	ShapeGrammar sg = facade_grammar.get_grammar(0);
	EXPECT_TRUE(sg.grammar_name_ == "Facade2WallAndFacadeFloor");
	EXPECT_TRUE(sg.apply_direction_ == 0);
	EXPECT_TRUE(sg.left_hand_symbol_ == "Facade");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Wall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FacadeFloor");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(1);
	EXPECT_TRUE(sg.grammar_name_ == "Facade2FloorWallAndFacadeWall");
	EXPECT_TRUE(sg.apply_direction_ == 0);
	EXPECT_TRUE(sg.left_hand_symbol_ == "Facade");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "FloorWall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FacadeWall");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(2);
	EXPECT_TRUE(sg.grammar_name_ == "FacadeFloor2FloorWallAndFacadeWall");
	EXPECT_TRUE(sg.apply_direction_ == 0);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FacadeFloor");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "FloorWall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FacadeWall");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(3);
	EXPECT_TRUE(sg.grammar_name_ == "FacadeWall2WallAndFacadeFloor");
	EXPECT_TRUE(sg.apply_direction_ == 0);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FacadeWall");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Wall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FacadeFloor");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(4);
	EXPECT_TRUE(sg.grammar_name_ == "FloorWall2WallAndFloorWindow");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FloorWall");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Wall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWindow");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(5);
	EXPECT_TRUE(sg.grammar_name_ == "FloorWindow2WindowAndFloorWall");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FloorWindow");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Window");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWall");
	//EXPECT_TRUE(sg.parameter_min == 1);
	//EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	EXPECT_TRUE(facade_grammar.get_symbol_direction_map_size() == 5);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("Facade") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FacadeFloor") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FacadeWall") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWall") == 1);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWindow") == 1);
}
TEST(Test_SG_Test, Test_Read_One_Dim_Grammar) {
	std::string grammar_path = "D:\\vs2012projects\\ParsingEngineCore\\Tests\\data\\one dimension sg.xml";

	FacadeGrammar facade_grammar;
	facade_grammar.ReadGrammar(grammar_path);

	EXPECT_TRUE(facade_grammar.get_atom_symbol() == "Facade");
	EXPECT_TRUE(facade_grammar.get_terminal_symbol_size() == 2);
	EXPECT_TRUE(facade_grammar.get_terminal_symbol(0) == "Wall");
	EXPECT_TRUE(facade_grammar.get_terminal_symbol(1) == "Window");
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[0] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[1] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Wall")[2] == 120);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[0] == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[1] == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_color("Window")[2] == 255);

	EXPECT_TRUE(facade_grammar.get_grammar_number() == 4);
	ShapeGrammar sg = facade_grammar.get_grammar(0);
	EXPECT_TRUE(sg.grammar_name_ == "Facade2WallAndFloorWindow");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "Facade");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Wall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWindow");
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 5);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(1);
	EXPECT_TRUE(sg.grammar_name_ == "Facade2WindowAndFloorWall");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "Facade");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Window");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWall");
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 5);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(2);
	EXPECT_TRUE(sg.grammar_name_ == "FloorWall2WallAndFloorWindow");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FloorWall");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Wall");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWindow");
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 5);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(3);
	EXPECT_TRUE(sg.grammar_name_ == "FloorWindow2WindowAndFloorWall");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FloorWindow");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Window");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWall");
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 5);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	EXPECT_TRUE(facade_grammar.get_symbol_direction_map_size() == 3);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("Facade") == 1);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWall") == 1);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWindow") == 1);
}

#endif