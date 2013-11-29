
#ifndef SHAPE_GRAMMAR_TEST_H
#define SHAPE_GRAMMAR_TEST_H

#include "gtest/gtest.h"
#include "facade_grammar.h"

TEST(Test_SG, Read_Grammar) {
	std::string grammar_path = "data\\shape_grammar.xml";

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
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	sg = facade_grammar.get_grammar(5);
	EXPECT_TRUE(sg.grammar_name_ == "FloorWindow2WindowAndFloorWall");
	EXPECT_TRUE(sg.apply_direction_ == 1);
	EXPECT_TRUE(sg.left_hand_symbol_ == "FloorWindow");
	EXPECT_TRUE(sg.right_hand_symbol1_ == "Window");
	EXPECT_TRUE(sg.right_hand_symbol2_ == "FloorWall");
	EXPECT_TRUE(sg.parameter_min == 1);
	EXPECT_TRUE(sg.parameter_max == 15);
	EXPECT_TRUE(sg.exception_symbol == "Wall");

	EXPECT_TRUE(facade_grammar.get_symbol_direction_map_size() == 5);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("Facade") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FacadeFloor") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FacadeWall") == 0);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWall") == 1);
	EXPECT_TRUE(facade_grammar.get_symbol_apply_direction("FloorWindow") == 1);
}

#endif