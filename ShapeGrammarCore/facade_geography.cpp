#include "facade_geography.h"
#include "facade_grammar.h"

#include "pcl\common\centroid.h"
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl\visualization\pcl_plotter.h>
#include <pcl/filters/voxel_grid.h>

#include "boost/progress.hpp"
#include <fstream>
#include <assert.h>

FacadeGeography::FacadeGeography(int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar)
{
	init(widthX, widthY, facade_grammar);
}

FacadeGeography::~FacadeGeography()
{
	destroy();
}

FacadeGeography::FacadeGeography (std::shared_ptr<FacadeGrammar> facade_grammar)
{
	facade_grammar_ = facade_grammar;
	//	init(res, extMin, extRange);
}

void FacadeGeography::init ( int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar)
{
	facade_grammar_ = facade_grammar;

	width_ = widthX;
	height_ = widthY;

	depth_ = facade_grammar_->get_terminal_symbol_size();
	grd_.resize (depth_);
	for ( int i = 0; i < depth_; ++i)
	{
		grd_[i] = new double*[height_];
		for ( int j = 0; j < height_; ++j)
		{
			grd_[i][j] = new double[width_];
			for (int k = 0; k < width_; ++k)
				grd_[i][j][k] = 0.0;
		}
	}
}

void FacadeGeography::init ( double res, const cv::Point2d& extMin, const cv::Point2d& extRange )
{
	resolution_ = res;
	extents_min_ = extMin;
	extents_range_ = extRange;

	width_ = (int)(extRange.x / res + 0.5);
	height_ = (int)(extRange.y / res + 0.5);
	convert_ratio_ = cv::Point2d((width_-1.0) / extRange.x, (height_-1.0) / extRange.y);

	depth_ = facade_grammar_->get_terminal_symbol_size();
	grd_.resize(depth_);
	for ( int i = 0; i < depth_; ++i)
	{
		//grd_[i] = new Mat(height_, width_, CV_8UC1);
		grd_[i] = new double*[height_];
		for ( int j = 0; j < height_; ++j)
		{
			grd_[i][j] = new double[width_];
			for (int k = 0; k < width_; ++k)
			{
				grd_[i][j][k] = 0.0;
			}	
		}
	}
}

void FacadeGeography::CreateGridFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud, double resolution) {
	pcl::PointXYZ min_extent, max_extent;
	pcl::getMinMax3D(*facade_cloud, min_extent, max_extent);

	resolution_ = resolution;
	extents_min_.x = min_extent.x;
	extents_min_.y = min_extent.y;
	extents_range_.x = max_extent.x - min_extent.x;
	extents_range_.y = max_extent.y - min_extent.y;

	width_ = (int)(extents_range_.x / resolution_ + 0.5);
	height_ = (int)(extents_range_.y / resolution_ + 0.5);
	convert_ratio_ = cv::Point2d((width_-1.0) / extents_range_.x, (height_-1.0) / extents_range_.y);

	depth_ = facade_grammar_->get_terminal_symbol_size();
	grd_.resize(depth_);
	for ( int i = 0; i < depth_; ++i)
	{
		grd_[i] = new double*[height_];
		for ( int j = 0; j < height_; ++j)
		{
			grd_[i][j] = new double[width_];
			for (int k = 0; k < width_; ++k)
			{
				grd_[i][j][k] = 0.0;
			}	
		}
	}

	boost::progress_display display(width_ * height_);

	Eigen::Vector4f max_pt, min_pt;
	for (int i = 0; i < width_; ++i)
	{
		for ( int j = 0; j < height_; ++j)
		{
			min_pt(0) = i * resolution_ + extents_min_.x;
			min_pt(1) = j * resolution_ + extents_min_.y;
			min_pt(2) = min_extent.z;
			min_pt(3) = 1.0;

			max_pt(0) = min_pt(0) + resolution_;
			max_pt(1) = min_pt(1) + resolution_;
			max_pt(2) = max_extent.z;
			max_pt(3) = 1.0;

			std::vector<int> indices;
			pcl::getPointsInBox(*facade_cloud, min_pt, max_pt, indices);

			assert(indices.size() >= 0);
#if 0
			if (indices.size() <= 1) // window
			{
				SetProperty(i, j, 1, 1);
			}
			else // wall
			{
				double height = 0;
				for (int k = 0; k < indices.size(); ++k)
				{
					height += facade_cloud->points[k].z;
				}
				height /= indices.size();

				if (height < -0.5)
				{
					SetProperty(i, j, 1, 1); // window
				}
				else if (height > 0.5)
				{
					SetProperty(i, j, 2, 1); // over wall
				}
				else
				{
					SetProperty(i, j, 0, 1); // wall
				}
			}
#endif
			indices.clear();
			++display;
		}
	}

}

void FacadeGeography::destroy()
{
	for ( int i = 0; i < depth_; ++i )
	{
		if ( grd_[i] != NULL )
		{
			for ( int j = 0; j < height_; ++j )
			{
				delete[] grd_[i][j];
				grd_[i][j] = NULL;

			}
			delete[] grd_[i];
		}
	}
	resolution_ = 0;
	width_ = 0;
	height_ = 0;
	depth_ = 0;
}

double FacadeGeography::GetProperty ( int width, int height, int depth)
{
	return grd_[depth][height][width];
}

double FacadeGeography::GetProperty ( const cv::Point2d& pos, int depth )
{
	cv::Point2d localPos = pos - extents_min_;
	int width = (int)(localPos.x * convert_ratio_.x);
	int height = (int)(localPos.y * convert_ratio_.y);

	return grd_[depth][height][width];
}

void FacadeGeography::SetProperty ( int width, int height, int depth, double value )
{
	// 	uchar* row = grd_[depth]->ptr<uchar>(height);
	// 	row[width] = (uchar)value;
	grd_[depth][height][width] = value;
}

void FacadeGeography::RenderGrd ( std::string path )
{
	for ( size_t depth = 0; depth < grd_.size(); ++depth)
	{
		cv::Mat mat(height_, width_, CV_8UC1);

		for ( int i = 0; i < height_; ++i)
		{
			for ( int j = 0; j < width_; ++j)
			{
				uchar* row = mat.ptr<uchar>(i);
				row[j] = (uchar)(grd_[depth][i][j] * 255);
			}
		}

		std::ostringstream image_path;
		image_path << path << "_" << facade_grammar_->get_terminal_symbol(depth) << ".png";
		imwrite(image_path.str(), mat);
	}
}


void FacadeGeography::DebugCalculateFeature(std::string filename) {

	cv::Mat mat(height_, width_, CV_8UC3);

	cv::Mat_<cv::Vec3b> _Mat = mat;

	for ( int i = 0; i < height_; ++i)
	{
		for ( int j = 0; j < width_; ++j)
		{
			_Mat(i, j)[0] = (uchar)(grd_[1][i][j] * 255);
			_Mat(i, j)[1] = (uchar)(grd_[1][i][j] * 255);
			_Mat(i, j)[2] = (uchar)(grd_[1][i][j] * 255);
		}
	}
	int thickness = 1;
	int line_type = 8;

	int sum = 0;
	for (int i = 0; i < action_parameters_vertical_.size(); ++i)
	{
		int length = action_parameters_vertical_[i].second + sum;
		sum = length;
		cv::Point start(0, length);
		cv::Point end(width_, length);

		if (action_parameters_vertical_[i].first == "wall")
		{
			cv::line(mat, start, end, cv::Scalar(255, 0, 0), thickness, line_type);
		}
		else
		{
			cv::line(mat, start, end, cv::Scalar(0, 0, 255), thickness, line_type);
		}

	}

	sum = 0;
	for (int i = 0; i < action_parameters_horizontal_.size(); ++i)
	{
		int length = action_parameters_horizontal_[i].second + sum;
		sum = length;
		cv::Point start(length, 0);
		cv::Point end(length, height_);

		if (action_parameters_horizontal_[i].first == "wall")
		{
			cv::line(mat, start, end, cv::Scalar(255, 0, 0), thickness, line_type);
		}
		else
		{
			cv::line(mat, start, end, cv::Scalar(0, 0, 255), thickness, line_type);
		}
	}

	imwrite(filename, mat);
}


void FacadeGeography::SaveGrids ( std::string& filename )
{
	// Save grid header.
	std::string grid_header = filename + "header.txt";
	std::ofstream ofs(grid_header.c_str());
	ofs << "Width " << width_ << std::endl;
	ofs << "Height " << height_ << std::endl;
	ofs << "Depth " << depth_ << std::endl;
	ofs << "Resolution " << resolution_ << std::endl;
	ofs << "Extent_Min " << extents_min_.x << " " << extents_min_.y << std::endl;
	ofs << "Extent_Range " << extents_range_.x << " " << extents_range_.y << std::endl;
	ofs << "Convert_Ratio " << convert_ratio_.x << " " << convert_ratio_.y << std::endl;
	ofs.close();

	// Save grids
	for ( size_t depth = 0; depth < grd_.size(); ++depth)
	{
		std::string new_path = filename + facade_grammar_->get_terminal_symbol(depth) + ".txt";
		std::ofstream ofs(new_path.c_str());

		for ( int i = 0; i < height_; ++i)
		{
			for ( int j = 0; j < width_; ++j)
			{
				ofs << grd_[depth][i][j] << " ";
			}
			ofs << std::endl;
		}
		ofs.close();
	}
}

void FacadeGeography::LoadGrids ( std::string& filename )
{
	int grd_depth = facade_grammar_->get_terminal_symbol_size();
	grd_.resize(grd_depth);

	// load header first
	std::string header_path = filename + "header.txt";
	std::ifstream ifs(header_path.c_str());
	std::string dummy;
	ifs >> dummy >> width_;
	ifs >> dummy >> height_;
	ifs >> dummy >> depth_;
	ifs >> dummy >> resolution_;
	ifs >> dummy >> extents_min_.x >> extents_min_.y;
	ifs >> dummy >> extents_range_.x >> extents_range_.y;
	ifs >> dummy >> convert_ratio_.x >> convert_ratio_.y;
	ifs.close();

	for (size_t depth = 0; depth < grd_depth; ++depth)
	{
		std::string new_path = filename + facade_grammar_->get_terminal_symbol(depth) + ".txt";
		std::ifstream ifs(new_path.c_str() );

		grd_[depth] = new double*[height_];
		for ( int j = 0; j < height_; ++j)
		{
			grd_[depth][j] = new double[width_];
			for (int k = 0; k < width_; ++k)
			{
				grd_[depth][j][k] = 0.0;
			}	
		}

		for ( int i = 0; i < height_; ++i )
		{
			for ( int j = 0; j < width_; ++j )
			{
				ifs >> grd_[depth][i][j];
			}
		}

		ifs.close();
	}
}

void FacadeGeography::SetSymbol(double xmin, double xmax, double ymin, double ymax, int depth) {
	for (int x = xmin; x < xmax; ++x)
	{
		for (int y = ymin; y < ymax; ++y)
		{
			SetProperty(x, y, depth, 1.0);
		}	
	}
}

void FacadeGeography::ComputeFeature2() {
	// vertical feature
	for (int i = 0; i < height_; ++i)
	{
		double sum = 0;
		for (int j = 0; j < width_; ++j)
		{
			sum += grd_[1][i][j];
		}
		sum /= width_;
		vertical_feature.push_back(sum);
	}

	// horizontal feature
	for (int i = 0; i < width_; ++i)
	{
		double sum = 0;
		for (int j = 0; j < height_; ++j)
		{
			sum += grd_[1][j][i];
		}
		sum /= height_;
		horizonal_feature.push_back(sum);
	}
}

void FacadeGeography::ComputeFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud) {
	// First down sample the dense cloud
	std::cout << "down sampling ... \n";
	pcl::VoxelGrid<pcl::PointXYZ> filter;
	filter.setInputCloud(facade_cloud);
	filter.setLeafSize(0.1f, 0.1f, 0.1f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	filter.filter(*filtered_cloud);
	std::cout << "down sampling done. After filter, " << filtered_cloud->width * filtered_cloud->height <<
		" points left.\n";
	//
	pcl::search::KdTree<pcl::PointXYZ> search;

	pcl::PointXYZ cloud_min, cloud_max;
	pcl::getMinMax3D(*filtered_cloud, cloud_min, cloud_max);

	boost::progress_display  display(height_ + width_);
	Eigen::Vector4f min_pt, max_pt;
	for (int i = 0; i < width_; ++i) // for each width, compute the average z value
	{
		min_pt(0) = extents_min_.x + i * resolution_;
		min_pt(1) = extents_min_.y;
		min_pt(2) = cloud_min.x;
		min_pt(3) = 1.0;

		max_pt(0) = min_pt(0) + resolution_;
		max_pt(1) = min_pt(1) + resolution_ * height_;
		max_pt(2) = cloud_max.x;
		max_pt(3) = 1.0;

		std::vector<int> indices;
		pcl::getPointsInBox(*filtered_cloud, min_pt, max_pt, indices);

		assert(indices.size() >= 0);
		double sum = 0.0;
		for (int k = 0; k < indices.size(); ++k)
		{
			sum += filtered_cloud->points[indices[k]].z;
		}
		sum /= indices.size();

		horizonal_feature.push_back(sum);
		++display;
	}


	for (int i = 0; i < height_; ++i) // for each height, compute the average z value
	{
		min_pt(0) = extents_min_.x;
		min_pt(1) = extents_min_.y + i * resolution_;
		min_pt(2) = cloud_min.x;
		min_pt(3) = 1.0;

		max_pt(0) = min_pt(0) + resolution_ * width_;
		max_pt(1) = min_pt(1) + resolution_;
		max_pt(2) = cloud_max.x;
		max_pt(3) = 1.0;

		std::vector<int> indices;
		pcl::getPointsInBox(*filtered_cloud, min_pt, max_pt, indices);

		assert(indices.size() >= 0);
		double sum = 0.0;
		for (int k = 0; k < indices.size(); ++k)
		{
			sum += filtered_cloud->points[indices[k]].z;
		}
		sum /= indices.size();

		vertical_feature.push_back(sum);
		++display;
	}
}

void FacadeGeography::ShowFeature() {
	std::vector<double> width, height;
	for (int i = 0; i < width_; ++i)
	{
		width.push_back(i);
	}
	for (int j = 0; j < height_; ++j)
	{
		height.push_back(j);
	}
	assert(horizonal_feature.size() == width_);
	assert(vertical_feature.size() == height_);
	std::shared_ptr<pcl::visualization::PCLPlotter> plotter(new pcl::visualization::PCLPlotter);
	std::shared_ptr<pcl::visualization::PCLPlotter> plotter2(new pcl::visualization::PCLPlotter);
	plotter->addPlotData(width, horizonal_feature, "Horizontal Feature", vtkChart::LINE);
	plotter2->addPlotData(vertical_feature, height, "Vertical Feature", vtkChart::LINE);

	/*
	std::vector<double> XX, YY;
	XX.push_back(horizontal_wall_parameters[0]);
	XX.push_back(horizontal_wall_parameters[0]);
	YY.push_back(0.0);
	YY.push_back(1.0);
	plotter->addPlotData(XX, YY);

	for (int i = 1; i < horizontal_wall_parameters.size(); ++i)
	{
	int lastx = XX[0];
	XX.clear();
	YY.clear();
	XX.push_back(lastx + horizontal_wall_parameters[i]);
	XX.push_back(lastx + horizontal_wall_parameters[i]);
	YY.push_back(0.0);
	YY.push_back(1.0);

	plotter->addPlotData(XX, YY);
	}
	*/
	plotter->plot();
	plotter2->plot();
}

#define UNIQUE_ELEMENT 0

void FacadeGeography::CalculateActionParameters(double vertical_thread, double horizontal_thread) {
	int last_symbol;

	if (vertical_feature[0] > vertical_thread)
	{
		last_symbol = 0; // wall
	}
	else
	{
		last_symbol = 1; // window
	}

	int parameter = 1;
	for (int i = 1; i < vertical_feature.size(); ++i)
	{
		if (vertical_feature[i] < vertical_thread)
		{ // This position is window
			if (last_symbol == 1) // If last symbol is also window
			{
				parameter++; // add the window size
			}
			else // If last symbol is not window
			{
				//  Record last wall
				if (parameter > 2) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < vertical_wall_parameters.size(); ++k)
					{
						if (vertical_wall_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						vertical_wall_parameters.push_back(parameter);
					}
#else
					vertical_wall_parameters.push_back(parameter);
#endif
					// And reset the current symbol to window 
					parameter = 1;
					last_symbol = 1;
				}
				{
					last_symbol = 1;
				}

			}
		}
		else // This position is wall
		{
			if (last_symbol == 0) // If last symbol is also wall
			{
				parameter++; // add the wall size
			}
			else // If last symbol is not wall
			{
				//  Record last window
				if (parameter > 2) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < vertical_window_parameters.size(); ++k)
					{
						if (vertical_window_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						vertical_window_parameters.push_back(parameter);
					}
#else
					vertical_window_parameters.push_back(parameter);
#endif
					// And reset the current symbol to wall 
					parameter = 1;
					last_symbol = 0;
				}
				else
				{
					last_symbol = 0;
				}

			}
		}

	}

	if (horizonal_feature[0] > horizontal_thread)
	{
		last_symbol = 0; // wall
	}
	else
	{
		last_symbol = 1; // window
	}

	parameter = 1;
	for (int i = 1; i < horizonal_feature.size(); ++i)
	{
		if (horizonal_feature[i] < horizontal_thread)
		{ // This position is window
			if (last_symbol == 1) // If last symbol is also window
			{
				parameter++; // add the window size
			}
			else // If last symbol is not window
			{
				//  Record last wall
				if (parameter > 2) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < horizontal_wall_parameters.size(); ++k)
					{
						if (horizontal_wall_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						horizontal_wall_parameters.push_back(parameter);
					}
#else
					horizontal_wall_parameters.push_back(parameter);
#endif
					// And reset the current symbol to window 
					parameter = 1;
					last_symbol = 1;
				}
				else
				{
					last_symbol = 1;
				}

			}
		}
		else // This position is wall
		{
			if (last_symbol == 0) // If last symbol is also wall
			{
				parameter++; // add the wall size
			}
			else // If last symbol is not wall
			{
				//  Record last window
				if (parameter > 2) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < horizontal_window_parameters.size(); ++k)
					{
						if (horizontal_window_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						horizontal_window_parameters.push_back(parameter);
					}
#else
					horizontal_window_parameters.push_back(parameter);
#endif
					// And reset the current symbol to wall 
					parameter = 1;
					last_symbol = 0;
				}
				else
				{
					last_symbol = 0;
				}	
			}
		}
	}
}


void FacadeGeography::CalculateActionParameters2(double vertical_thread, double horizontal_thread) {
	std::string last_symbol;
	int vertical_min_size = 2;
	int horizontal_min_size = 2;

	if (vertical_feature[0] < vertical_thread)
	{
		last_symbol = "wall"; // wall
	}
	else
	{
		last_symbol = "window"; // window
	}

	int parameter = 1;
	for (int i = 1; i < vertical_feature.size(); ++i)
	{
		if (vertical_feature[i] < vertical_thread)
		{ // This position is wall
			if (last_symbol == "wall") // If last symbol is also wall
			{
				parameter++; // add the wall size
				if (i == (vertical_feature.size() - 1))
				{
					action_parameters_vertical_.push_back(std::make_pair(last_symbol, parameter));
				}
			}
			else // If last symbol is not wall
			{
				//  Record last window
				if (parameter > vertical_min_size) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < vertical_wall_parameters.size(); ++k)
					{
						if (vertical_wall_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						vertical_wall_parameters.push_back(parameter);
					}
#else
					//vertical_wall_parameters.push_back(parameter);
					action_parameters_vertical_.push_back(std::make_pair(last_symbol, parameter));
#endif
					// And reset the current symbol to window 
					parameter = 1;
					last_symbol = "wall";
				}
				{
					last_symbol = "wall";
				}
			}
		}
		else // This position is window
		{
			if (last_symbol == "window") // If last symbol is also window
			{
				parameter++; // add the window size
				if (i == (vertical_feature.size() - 1))
				{
					action_parameters_vertical_.push_back(std::make_pair(last_symbol, parameter));
				}
			}
			else // If last symbol is not window
			{
				//  Record last wall
				if (parameter > vertical_min_size) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < vertical_window_parameters.size(); ++k)
					{
						if (vertical_window_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						vertical_window_parameters.push_back(parameter);
					}
#else
					//vertical_window_parameters.push_back(parameter);
					action_parameters_vertical_.push_back(std::make_pair(last_symbol, parameter));
#endif
					// And reset the current symbol to window 
					parameter = 1;
					last_symbol = "window";
				}
				else
				{
					last_symbol = "window";
				}

			}
		}

	}

	if (horizonal_feature[0] < horizontal_thread)
	{
		last_symbol = "wall"; // wall
	}
	else
	{
		last_symbol = "window"; // window
	}

	parameter = 1;
	for (int i = 1; i < horizonal_feature.size(); ++i)
	{
		if (horizonal_feature[i] < horizontal_thread)
		{ // This position is wall
			if (last_symbol == "wall") // If last symbol is also wall
			{
				parameter++; // add the wall size

				if (i == (horizonal_feature.size() - 1))
				{
					action_parameters_horizontal_.push_back(std::make_pair(last_symbol, parameter));
				}
			}
			else // If last symbol is not wall
			{
				//  Record last window
				if (parameter > horizontal_min_size) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < horizontal_wall_parameters.size(); ++k)
					{
						if (horizontal_wall_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						horizontal_wall_parameters.push_back(parameter);
					}
#else
					action_parameters_horizontal_.push_back(std::make_pair(last_symbol, parameter));
					//horizontal_wall_parameters.push_back(parameter);
#endif
					// And reset the current symbol to wall 
					parameter = 1;
					last_symbol = "wall";
				}
				else
				{
					last_symbol = "wall";
				}

			}
		}
		else // This position is window
		{
			if (last_symbol == "window") // If last symbol is also window
			{
				parameter++; // add the window size

				if (i == (horizonal_feature.size() - 1))
				{
					action_parameters_horizontal_.push_back(std::make_pair(last_symbol, parameter));
				}
			}
			else // If last symbol is not window
			{
				//  Record last wall
				if (parameter > horizontal_min_size) // skip the too small parameters, because no so small wall and window.
				{
#if UNIQUE_ELEMENT
					bool already_has = false;
					for (int k = 0; k < horizontal_window_parameters.size(); ++k)
					{
						if (horizontal_window_parameters[k] == parameter)
						{
							already_has = true;
						}
					}
					if (!already_has) // skip the already has parameters.
					{
						horizontal_window_parameters.push_back(parameter);
					}
#else
					action_parameters_horizontal_.push_back(std::make_pair(last_symbol, parameter));
					//horizontal_window_parameters.push_back(parameter);
#endif
					// And reset the current symbol to window 
					parameter = 1;
					last_symbol = "window";
				}
				else
				{
					last_symbol = "window";
				}	
			}
		}
	}
}
void FacadeGeography::SaveActionParammeters(std::string parater_path) {
	std::ofstream fout(parater_path.c_str());
	/*
	fout << "vertical" << std::endl;
	for (int i = 0; i < height_; ++i)
	{
	fout << vertical_feature[i] << " ";
	}
	fout << std::endl << "horizontal" << std::endl;
	for (int i = 0; i < width_; ++i)
	{
	fout << horizonal_feature[i] << " ";
	}
	fout << std::endl;
	*/
	fout << "vertical__wall_parameters " << " " << vertical_wall_parameters.size() << std::endl;
	for (int i = 0; i < vertical_wall_parameters.size(); ++i)
	{
		fout << vertical_wall_parameters[i] << " ";
	}
	fout << std::endl;

	fout << "vertical__window_parameters " << " " << vertical_window_parameters.size() << std::endl;
	for (int i = 0; i < vertical_window_parameters.size(); ++i)
	{
		fout << vertical_window_parameters[i] << " ";
	}
	fout << std::endl;

	fout << "horizontal__wall_parameters " << " " << horizontal_wall_parameters.size() << std::endl;
	for (int i = 0; i < horizontal_wall_parameters.size(); ++i)
	{
		fout << horizontal_wall_parameters[i] << " ";
	}
	fout << std::endl;

	fout << "horizontal__window_parameters " << " " << horizontal_window_parameters.size() << std::endl;
	for (int i = 0; i < horizontal_window_parameters.size(); ++i)
	{
		fout << horizontal_window_parameters[i] << " ";
	}
	fout << std::endl;

	fout.close();
}

void FacadeGeography::SaveActionParammeters2(std::string parater_path) {
	std::ofstream fout(parater_path.c_str());
	/*
	fout << "vertical" << std::endl;
	for (int i = 0; i < height_; ++i)
	{
	fout << vertical_feature[i] << " ";
	}
	fout << std::endl << "horizontal" << std::endl;
	for (int i = 0; i < width_; ++i)
	{
	fout << horizonal_feature[i] << " ";
	}
	fout << std::endl;
	*/


	fout << "vertical_parameters " << " " << action_parameters_vertical_.size() << std::endl;
	for (int i = 0; i < action_parameters_vertical_.size(); ++i)
	{
		fout << action_parameters_vertical_[i].first << " " << action_parameters_vertical_[i].second << std::endl;
	}
	fout << std::endl;

	fout << "horizontal_parameters " << " " << action_parameters_horizontal_.size() << std::endl;
	for (int i = 0; i < action_parameters_horizontal_.size(); ++i)
	{
		fout << action_parameters_horizontal_[i].first << " " << action_parameters_horizontal_[i].second << std::endl;
	}

	fout.close();
}

void FacadeGeography::ParseActionParameters() {
	for (int i = 0; i < action_parameters_vertical_.size(); ++i)
	{
		if (action_parameters_vertical_[i].first == "wall")
		{
			bool already_has = false;
			for (int j = 0; j < vertical_wall_parameters.size(); ++j)
			{
				if (vertical_wall_parameters[j] == action_parameters_vertical_[i].second)
				{
					already_has = true;
				}
			}

			if (!already_has)
			{
				vertical_wall_parameters.push_back(action_parameters_vertical_[i].second);
			}
		}
		else
		{
			bool already_has = false;
			for (int j = 0; j < vertical_window_parameters.size(); ++j)
			{
				if (vertical_window_parameters[j] == action_parameters_vertical_[i].second)
				{
					already_has = true;
				}
			}

			if (!already_has)
			{
				vertical_window_parameters.push_back(action_parameters_vertical_[i].second);
			}
		}
	}

	for (int i = 0; i < action_parameters_horizontal_.size(); ++i)
	{
		if (action_parameters_horizontal_[i].first == "wall")
		{
			bool already_has = false;
			for (int j = 0; j < horizontal_wall_parameters.size(); ++j)
			{
				if (horizontal_wall_parameters[j] == action_parameters_horizontal_[i].second)
				{
					already_has = true;
				}
			}

			if (!already_has)
			{
				horizontal_wall_parameters.push_back(action_parameters_horizontal_[i].second);
			}
		}
		else
		{
			bool already_has = false;
			for (int j = 0; j < horizontal_window_parameters.size(); ++j)
			{
				if (horizontal_window_parameters[j] == action_parameters_horizontal_[i].second)
				{
					already_has = true;
				}
			}

			if (!already_has)
			{
				horizontal_window_parameters.push_back(action_parameters_horizontal_[i].second);
			}
		}

	}
}

