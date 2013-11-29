#include "facade_grammar.h"
#include "rapidjson/reader.h"
#include "rapidjson/filestream.h"
#include "rapidjson/document.h"
#include "opencv2/opencv.hpp"

FacadeGrammar::FacadeGrammar() {

}

FacadeGrammar::~FacadeGrammar() {

}

bool FacadeGrammar::ReadGrammar ( std::string& configure_file )
{
	rapidjson::Document document;
	FILE* fp = fopen(configure_file.c_str(), "r");
	if(fp == NULL) {
		return false;
	}

	rapidjson::FileStream is(fp);
	if(document.ParseStream<0>(is).HasParseError()) {
		return false;
	}

	if (!document.IsObject()) {
		return false;
	}

	// atom symbol
	if (document.HasMember("atom")) {
		atom_symbol_ = document["atom"].GetString();
	} else {
		return false;
	}

	// terminal symbols
	if (document.HasMember("terminal")) {
		const rapidjson::Value& terminals = document["terminal"];
		if (!terminals.IsArray())  
			return false;

		for (int i = 0; i < terminals.Size(); ++i) {
			const rapidjson::Value& terminal_symbol = terminals[i];
			if (!terminal_symbol.IsObject()) // A terminal symbol can have a color
				return false;
			if (terminal_symbol.HasMember("name")) {
				terminal_symbol_.push_back(terminal_symbol["name"].GetString());
			} else {
				return false;
			}
			if (terminal_symbol.HasMember("color")) {
				std::vector<int> color;
				if (terminal_symbol["color"].Size() != 3)
					return false;
				for (int j = 0; j < 3; ++j) {
					color.push_back(terminal_symbol["color"][j].GetInt());
				}
				symbol_colors_.push_back(color);

				std::pair<std::string, int> color_pair;
				color_pair.first = terminal_symbol_[i];
				color_pair.second = i;
				symbol_color_map_.insert(color_pair);
			}
		}
	}

	// unterminal symbols
	if (document.HasMember("unterminal")) {
		const rapidjson::Value& unterminals = document["unterminal"];
		if (!unterminals.IsArray()) 
			return false;
		for (int i = 0; i < unterminals.Size(); ++i) {
			unterminal_symbol_.push_back(unterminals[i].GetString());
		}
	}

	// grammars
	if (document.HasMember("grammar")) {
		const rapidjson::Value& grammar = document["grammar"];
		if (!grammar.IsArray()) 
			return false;
		for (int i = 0; i < grammar.Size(); ++i) {
			grammar_names_.push_back(grammar["name"].GetString());
			ShapeGrammar shape_grammar;
			shape_grammar.grammar_name_ = grammar["name"].GetString();
			shape_grammar.apply_direction_ = grammar["direction"].GetInt();
			shape_grammar.left_hand_symbol_ = grammar["parent"].GetString();
			shape_grammar.right_hand_symbol1_ = grammar["child1"].GetString();
			shape_grammar.right_hand_symbol2_ = grammar["child2"].GetString();
			shape_grammar.parameter_min = grammar["parameter"][rapidjson::SizeType(0)].GetInt();
			shape_grammar.parameter_max = grammar["parameter"][rapidjson::SizeType(1)].GetInt();
			shape_grammar.exception_symbol = grammar["exception"].GetString();

			grammar_set_.push_back(shape_grammar);
		}
	}

	// symbol_apply_direction_
	for (size_t i = 0; i < grammar_set_.size(); ++i) {
		if(atom_symbol_ == grammar_set_[i].left_hand_symbol_) {
			std::pair<std::string, int> str_dir;
			str_dir.first = atom_symbol_;
			str_dir.second = grammar_set_[i].apply_direction_;
			symbol_apply_direction_.insert(str_dir);
		} else {
			for (size_t j = 0; j < unterminal_symbol_.size(); ++j) {
				if (unterminal_symbol_[j] == grammar_set_[i].left_hand_symbol_) {
					std::pair<std::string, int> str_dir;
					str_dir.first = unterminal_symbol_[j];
					str_dir.second = grammar_set_[i].apply_direction_;
					symbol_apply_direction_.insert(str_dir);
				}			
			}
		}
	}

	return true;
}

void FacadeGrammar::SaveGrammar ( std::string& configure_file )
{
	cv::FileStorage fs;
	fs.open(configure_file, cv::FileStorage::WRITE);

	fs << "Facade_Gramamr";
	fs << "{";

	fs << "Atom";
	fs << "[" << atom_symbol_ << "]";

	fs << "Terminal_Symbol";
	fs << "[";
	for (size_t i = 0; i < terminal_symbol_.size(); ++i)
	{
		fs << terminal_symbol_[i];
	}
	fs << "]";

	fs << "Unterminal_Symbol";
	fs << "[";
	for (size_t i = 0; i < unterminal_symbol_.size(); ++i)
	{
		fs << unterminal_symbol_[i];
	}
	fs << "]";

	fs << "Grammars";
	fs << "[";
	for (size_t i = 0; i < grammar_set_.size(); ++i)
	{
		fs << grammar_set_[i].grammar_name_;
	}
	fs << "]";

	for (size_t i = 0; i < grammar_set_.size(); ++i)
	{
		fs << grammar_set_[i].grammar_name_;
		fs << "{";

		fs << "Apply_Direction";
		fs << grammar_set_[i].apply_direction_;
		fs << "Left_Hand_Symbol";
		fs << grammar_set_[i].left_hand_symbol_;
		fs << "Right_Hand_Symbol1";
		fs << grammar_set_[i].right_hand_symbol1_;
		fs << "Right_Hand_Symbol2";
		fs << grammar_set_[i].right_hand_symbol2_;

		fs << "Parameter_Min";
		fs << grammar_set_[i].parameter_min;
		fs << "Parameter_Max";
		fs << grammar_set_[i].parameter_max;

		fs << "Exception_Symbol";
		fs << grammar_set_[i].exception_symbol;

		fs << "}";
	}
	fs << "}";

	fs.release();
}

void FacadeGrammar::PrintGrammars()
{
	std::cout << "Facade_Gramamr\n";
	std::cout << "{\n";

	std::cout << "Atom\n";
	std::cout << "[" << atom_symbol_ << "]\n\n";

	std::cout << "Terminal_Symbol\n";
	std::cout << "[";
	for (size_t i = 0; i < terminal_symbol_.size(); ++i)
	{
		std::cout << terminal_symbol_[i] << " ";
	}
	std::cout << "]\n\n";

	std::cout << "Unterminal_Symbol\n";
	std::cout << "[";
	for (size_t i = 0; i < unterminal_symbol_.size(); ++i)
	{
		std::cout << unterminal_symbol_[i] << " ";
	}
	std::cout << "]\n\n";

	std::cout << "Grammars\n";
	std::cout << "[";
	for (size_t i = 0; i < grammar_set_.size(); ++i)
	{
		std::cout << grammar_set_[i].grammar_name_ << " ";
	}
	std::cout << "]\n\n";

	for (size_t i = 0; i < grammar_set_.size(); ++i)
	{
		std::cout << grammar_set_[i].grammar_name_;
		std::cout << "{\n";

		std::cout << "Apply_Direction\n[";
		std::cout << grammar_set_[i].apply_direction_;
		std::cout << "]\n\nLeft_Hand_Symbol\n[";
		std::cout << grammar_set_[i].left_hand_symbol_ ;
		std::cout << "]\n\nRight_Hand_Symbol1\n[";
		std::cout << grammar_set_[i].right_hand_symbol1_;
		std::cout << "]\n\nRight_Hand_Symbol2\n[";
		std::cout << grammar_set_[i].right_hand_symbol2_;

		std::cout << "]\n\nParameter_Min\n[";
		std::cout << grammar_set_[i].parameter_min;
		std::cout << "]\n\nParameter_Max\n[";
		std::cout << grammar_set_[i].parameter_max;

		std::cout << "]\n\nException_Symbol\n[";
		std::cout << grammar_set_[i].exception_symbol;

		std::cout << "]\n}\n\n";
	}
	std::cout << "}\n";
}

std::vector<int> FacadeGrammar::get_symbol_color(std::string symbol) {
	int index = symbol_color_map_.at(symbol);
	return symbol_colors_[index];
}