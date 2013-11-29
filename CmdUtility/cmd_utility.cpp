#include "cmd_utility.h"
#include "rapidjson/reader.h"
#include "rapidjson/filestream.h"
#include "rapidjson/document.h"
#include <fstream>

CommandLineArgument::CommandLineArgument() {
	// Initialize some parameters
	episodes_ = 5000;
	margin_[0] = margin_[1] = margin_[2] = margin_[3] = 0;
	resolution_ = 0.2;
	resample_resolution_[0] = resample_resolution_[1] = resample_resolution_[2] = 0.0;
	load_qtable_ = 0;

	ready_radius_outlier_remover_ = ready_refine_grid_ = ready_resample_ = ready_statiscal_outlier_remover_ = false;
}

bool CommandLineArgument::ParseCommandLine(int argc, char** argv) {
	boost::program_options::options_description options_desc("Parsing Building Facade Tools Options");
	options_desc.add_options()
		("help", "print help information.")
		("workplace", boost::program_options::value<std::string>(), "working folder.")
		;

	boost::program_options::variables_map variables_map;
	try
	{
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, options_desc), variables_map);
		boost::program_options::notify(variables_map);

		if(variables_map.count("help") || variables_map.size() == 0) {
			std::cout << options_desc << std::endl;
			return false;
		}
		if(variables_map.count("workplace")) {
			working_folder = variables_map["workplace"].as<std::string>();
		}
		else
		{
			return false;
		}
	}
	catch(boost::program_options::error& e) 
	{
		std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		std::cerr << options_desc << std::endl;
		return false;
	}
	return true;
}

bool CommandLineArgument::ParseCommandLineArgument(int argc, char** argv) {
	if (!ParseCommandLine(argc, argv))
	{
		return false;
	}

	action_parameters_ = working_folder + "\\action.txt";
	action_parameters_debug_ = working_folder + "\\action_debug.txt";
	action_parameters_png_ = working_folder + "\\action.png";
	grammar_ = working_folder + "\\grammar.json";
	/*grid_header_ = working_folder + "\\grid_header.txt";
	grid_wall_png_ = working_folder + "\\grid_wall.png";
	grid_wall_txt_ = working_folder + "\\grid_wall.txt";
	grid_window_png_ = working_folder + "\\grid_window.png";
	grid_window_txt_ = working_folder + "\\grid_window.txt";*/
	grid_ = working_folder + "\\grid_";
	original_result_overlap_png_ = working_folder + "\\compare_result.png";
	parameters_ = working_folder + "\\configure.json";
	pt_pcd_ = working_folder + "\\pt.pcd";
	qtable_ = working_folder + "\\qtable.txt";
	raw_pcd_ = working_folder + "\\raw.pcd";
	raw_txt_ = working_folder + "\\raw.txt";
	result_png_ = working_folder + "\\result.png";
	result_txt_ = working_folder + "\\result.txt";
	/*revise_grid_header_ = working_folder + "\\revise_grid_header.txt";
	revise_grid_wall_png_ = working_folder + "\\revise_grid_wall.png";
	revise_grid_wall_txt_ = working_folder + "\\revise_grid_wall.txt";
	revise_grid_window_png_ = working_folder + "\\revise_grid_window.png";
	revise_grid_window_txt_ = working_folder + "\\revise_grid_window.txt";*/
	revise_grid_ = working_folder + "\\revise_grid_";
	rt_pcd_ = working_folder + "\\rt.pcd";

	if (!ReadParameters())
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool CommandLineArgument::ReadParameters() {
	rapidjson::Document document;
	FILE* fp = fopen(parameters_.c_str(), "r");
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

	if (document.HasMember("episodes")) {
		episodes_ = document["episodes"].GetInt();
	}

	if (document.HasMember("resolution")) {
		resolution_ = document["resolution"].GetDouble();
	}

	if (document.HasMember("load_qtable")) {
		load_qtable_ = document["load_qtable"].GetInt();
	}

	if (document.HasMember("margin")) {
		const rapidjson::Value& margin = document["margin"];
		if(margin.IsArray()) {
			if (margin.Size() == 4) {
				for (rapidjson::SizeType i = 0; i < margin.Size(); ++i) {
					margin_[i] = margin[i].GetInt();
				}
				ready_refine_grid_ = true;
			}
		}	
	}

	if (document.HasMember("radius_outlier_remover")) {
		const rapidjson::Value& radius_outlier_remover = document["radius_outlier_remover"];
		if(radius_outlier_remover.IsArray()) {
			if (radius_outlier_remover.Size() == 2) {
				for (rapidjson::SizeType i = 0; i < radius_outlier_remover.Size(); ++i) {
					radius_outlier_remover_[i] = radius_outlier_remover[i].GetDouble();
				}
				ready_radius_outlier_remover_ = true;
			}
		}	
	}
	if (document.HasMember("static_outlier_remover")) {
		const rapidjson::Value& static_outlier_remover = document["static_outlier_remover"];
		if(static_outlier_remover.IsArray()) {
			if (static_outlier_remover.Size() == 4) {
				for (rapidjson::SizeType i = 0; i < static_outlier_remover.Size(); ++i) {
					statiscal_outlier_remover_[i] = static_outlier_remover[i].GetDouble();
				}
				ready_statiscal_outlier_remover_ = true;
			}
		}	
	}
	if (document.HasMember("resample")) {
		const rapidjson::Value& resample = document["resample"];
		if(resample.IsArray()) {
			if (resample.Size() == 3) {
				for (rapidjson::SizeType i = 0; i < resample.Size(); ++i) {
					resample_resolution_[i] = resample[i].GetDouble();
				}
				ready_resample_ = true;
			}
		}	
	}

	return true;
}