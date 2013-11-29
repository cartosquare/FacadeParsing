#ifndef CMD_UTILITY_H
#define CMD_UTILITY_H

#include "boost/program_options.hpp"
#include <iostream>
#include <string>

class CommandLineArgument {
public:
	CommandLineArgument();
	~CommandLineArgument() {}

	bool ParseCommandLineArgument(int argc, char** argv);
	bool ReadParameters();
	bool ParseCommandLine(int argc, char** argv);

//private:
	std::string working_folder;
	
	// point clouds
	std::string raw_pcd_;		// raw point cloud in pcd format.
	std::string raw_txt_;		// raw point cloud in text format
	std::string rt_pcd_;		// raw transform point cloud.
	std::string pt_pcd_;		// plane transform point cloud.

	// actions
	std::string action_parameters_;
	std::string action_parameters_debug_;
	std::string action_parameters_png_;

	// grammar file
	std::string grammar_;

	// original grid
	//std::string grid_header_;
	//std::string grid_wall_txt_;
	//std::string grid_window_txt_;
	//std::string grid_wall_png_;
	//std::string grid_window_png_;
	std::string grid_;

	// revise grid
	/*std::string revise_grid_header_;
	std::string revise_grid_wall_txt_;
	std::string revise_grid_window_txt_;
	std::string revise_grid_wall_png_;
	std::string revise_grid_window_png_;*/
	std::string revise_grid_;

	// q table
	std::string qtable_;

	// parameter file
	std::string parameters_;
	double resolution_;
	double episodes_;
	int load_qtable_; // 0 -- not load; 1 -- load.
	//int margin_up_, margin_down_, margin_left_, margin_right_;
	int margin_[4]; // up, down, left, right
	double resample_resolution_[3];

	// result
	std::string result_png_;
	std::string result_txt_;
	std::string original_result_overlap_png_;
	
};

#endif // !CMD_UTILITY_H
