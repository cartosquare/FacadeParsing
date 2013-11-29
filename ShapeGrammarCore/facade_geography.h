/*=========================================================================================
* 				Facade geography define
* 				CopyRight Xiang Xu, 2013
*				xuxiang@mail.bnu.edu.cn
* =========================================================================================*/
#ifndef FACADE_GEOGRAPHY_H
#define FACADE_GEOGRAPHY_H

#include "pcl\point_types.h"
#include "pcl\point_cloud.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <memory>

class FacadeGrammar;
//! The geography position of the facade.
/*!
This class contains the spatial extent of the facade.
The facade is rostered to a rectangle grids.
*/
class FacadeGeography
{
public:
	//! Constructor 
	/*!
	\param widthX Facade width.
	\param widthY Facade Height.
	\param facade_grammar Facade grammar.
	*/
	FacadeGeography(int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar);

	//! Initialize the facade geography
	/*!
	\param widthX Facade width.
	\param widthY Facade Height.
	\param facade_grammar Facade grammar.
	*/
	void init(int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar);

	//! Constructor 
	/*!
	\param facade_grammar Facade grammar.
	*/
	FacadeGeography(std::shared_ptr<FacadeGrammar> facade_grammar);

	//! Initialize the facade geography 
	/*!
	\param res Facade resolution, stands for the meters of one pixel.
	\param extMin The left-bottom corner of the facade.
	\param extRange The extent of the facade.
	*/
	void init(double res, const cv::Point2d &extMin, const cv::Point2d &extRange);

	~FacadeGeography();
	void destroy();

	/** /brief Create the facade grid with pcl point cloud.
	 * \param facade_cloud facade point cloud.
	 */
	void CreateGridFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud, double resolution);

	//! Render the facade to picture.
	void RenderGrd(std::string path);

	//! Serialization
	/*!
	Save facade geography to file.
	*/
	void SaveGrids(std::string& filename);

	//! Serialization
	/*!
	Load facade geography from file.
	*/
	void LoadGrids(std::string& filename);

	//! Set the value at a specified pixel(grid).
	/*!
	\param width the width of the pixel.
	\param height the height of the pixel
	\param depth the depth of the pixel.
	\param value the value that we want to set in this pixel.
	*/
	void SetProperty(int width, int height, int depth, double value);

	//! get the value at a specified pixel(grid).
	/*!
	\param width the width of the pixel.
	\param height the height of the pixel
	\param depth the depth of the pixel.
	\return the value in this pixel.
	*/
	double GetProperty(int width, int height, int depth);

	//! get the value at a specified position.
	/*!
	\param pos the position of the pixel.
	\param depth the depth of the pixel.
	\return the value in this pixel.
	*/
	double GetProperty(const cv::Point2d &pos, int depth);


	void ComputeFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr facade_cloud);
	void ComputeFeature2();
	void SaveActionParammeters(std::string parater_path);
	void SaveActionParammeters2(std::string parater_path);
	void ShowFeature();
	void CalculateActionParameters(double vertical_thread, double horizontal_thread);
	void CalculateActionParameters2(double vertical_thread, double horizontal_thread);
	void ParseActionParameters();
	void DebugCalculateFeature(std::string filename);

	// Inline functions
	inline double get_resolution() const { return resolution_; }
	inline int get_width() const { return width_; }
	inline int get_height() const { return height_; }
	inline int get_depth() const { return depth_; }
	inline cv::Point2d get_extents_min() { return extents_min_; }
	inline cv::Point2d get_extents_range() { return extents_range_; }
	inline cv::Point2d get_convert_ratio() { return convert_ratio_; }

	// dangerous access
	inline double** get_grd_ptr ( int index ) { return grd_[index]; }

	// for tests
	void SetSymbol(double xmin, double xmax, double ymin, double ymax, int depth);

protected:
	std::vector<double**> grd_;  /*!< Facade grid. */
	cv::Point2d extents_min_, extents_range_, convert_ratio_; /*!< Facade extent and convert ratio. */
	double resolution_; /*!< Facade resolution. */
	int width_, height_, depth_; /*!< Facade grid extent and depth. */

	std::vector<double> vertical_feature, horizonal_feature;
	std::vector<int> vertical_wall_parameters, horizontal_wall_parameters, vertical_window_parameters, horizontal_window_parameters;

	std::vector<std::pair<std::string, int> > action_parameters_vertical_, action_parameters_horizontal_;

	std::shared_ptr<FacadeGrammar> facade_grammar_; /*!< Facade grammar. */
};

#endif // FACADE_GEOGRAPHY_H