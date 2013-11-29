
#include "cmd_utility.h"
#include "pcl\io\pcd_io.h"

#include <fstream>
#include <iostream>

CommandLineArgument cmd_arguments;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool load_pcd_from_txt(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

int main(int argc, char** argv) 
{
	// Step 0
	if (!cmd_arguments.ParseCommandLineArgument(argc, argv))
		return -1;

	// Step 1
	std::cout << "Convert .txt format to .pcd format ...\n";
	if (!load_pcd_from_txt(cmd_arguments.raw_txt_,  cloud))
	{
		std::cout  << ".txt to .pcd fail!\n";
	}

	// Step 2
	std::cout << "\nSave ... \n";
	pcl::io::savePCDFile(cmd_arguments.raw_pcd_, *cloud);
	
	return 0;
}

bool load_pcd_from_txt(const std::string& filePath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// fill in the cloud data
	std::fstream ifs(filePath);

	if(ifs.is_open())
	{
		while(!ifs.eof())
		{
			pcl::PointXYZ point;
			ifs >> point.x >> point.y >> point.z;

			cloud->push_back(point);
		}

		ifs.close();

		return true;
	}
	return false;
}