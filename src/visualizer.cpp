#include <fstream>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "occupancy_grid_adaptive.hpp"
#include "occupancy_grid_fixed.hpp"
#include "gaussian_noise_filter.hpp"
#include "occlusion_filter.hpp"

const std::string PO_HELP					= "help";
const std::string PO_MESH					= "mesh";
const std::string PO_CLOUD				= "cloud";
const std::string	PO_GRID					= "grid";
const std::string PO_GRID_BOX			= "grid_box";
const std::string PO_NOISE_STDDEV = "noise_stddev";
const std::string PO_OCCLUSION		= "occlusion";

bool parse_program_options (
		boost::program_options::variables_map & rVariablesMap,
		const int & crArgc,
		char ** ppArgv)
{
	try
	{
		boost::program_options::options_description description_("Allowed options");

		description_.add_options()
			(PO_HELP.c_str(), "Produce help message")
			(PO_MESH.c_str(), boost::program_options::value<std::string>()->default_value(""), "PLY mesh file")
			(PO_CLOUD.c_str(), boost::program_options::value<std::string>()->default_value(""), "PCD cloud file")
			(PO_GRID.c_str(), boost::program_options::value<std::string>()->default_value(""), "OCC grid file")
			(PO_GRID_BOX.c_str(), boost::program_options::value<bool>()->default_value(false), "Show only grid box")
			(PO_NOISE_STDDEV.c_str(), boost::program_options::value<double>()->default_value(0.0), "Noise standard deviation for cloud Gaussian Noise in z-axis")
			(PO_OCCLUSION.c_str(), boost::program_options::value<float>()->default_value(0.0f), "Occlusion percentage");

		boost::program_options::store(
				boost::program_options::parse_command_line(crArgc, ppArgv, description_),
				rVariablesMap);

		if (rVariablesMap.count(PO_HELP))
		{
			std::cout << description_ << "\n";
			return true;
		}

		boost::program_options::notify(rVariablesMap);
	}
	catch (std::exception & e_)
	{
		std::cerr << "[Error parsing program options]: " << e_.what() << "...\n";
		return true;
	}

	return false;
}

void visualize_polygonal_mesh (
		const pcl::PolygonMesh & crMesh,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & crViewer)
{
	crViewer->addPolygonMesh(crMesh, "mesh", 0);
	crViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "mesh");
	crViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.35, 0.53, "mesh");

	std::cout << "Mesh added...\n";
}

void visualize_point_cloud (
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cpCloud,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & crViewer)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_(cpCloud, 0, 91, 130);
	crViewer->addPointCloud<pcl::PointXYZ>(cpCloud, single_color_, "cloud");
	crViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	std::cout << "Cloud added...\n";
}

void visualize_occupancy_grid (
		OccupancyGrid & crGrid,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & crViewer)
{
	for (size_t z = 0; z < crGrid.get_side(); ++z)
	{
		for (size_t y = 0; y < crGrid.get_side(); ++y)
		{
			for (size_t x = 0; x < crGrid.get_side(); ++x)
			{
				double cell_ = crGrid.get_cell(x, y, z);

				if (cell_ > 0.0)
				{
					std::string name_ = std::to_string(x) + " " +
															std::to_string(y) + " " +
															std::to_string(z);

					crViewer->addCube(
							crGrid.get_origin(0) + x * crGrid.get_leaf_size(0),
							crGrid.get_origin(0) + x * crGrid.get_leaf_size(0) + crGrid.get_leaf_size(0),
							crGrid.get_origin(1) + y * crGrid.get_leaf_size(1),
							crGrid.get_origin(1) + y * crGrid.get_leaf_size(1) + crGrid.get_leaf_size(1),
							crGrid.get_origin(2) + z * crGrid.get_leaf_size(2),
							crGrid.get_origin(2) + z * crGrid.get_leaf_size(2) + crGrid.get_leaf_size(2),
							0.0, (1.0-cell_) * 0.35, 0.56,//((double)z/(double)crGrid.get_side()),
							// Color for binary grids with gradient
							//0.0, 1.0 - ((double)j/(double)crGrid.get_side()), 1.0 - ((double)k/(double)crGrid.get_side()),
							name_);
				}
			}
		}
	}

	std::cout << "Grid added...\n";
}

void generate_cube_points(
		const float & crOX,
		const float & crOY,
		const float & crOZ,
		const float & crSX,
		const float & crSY,
		const float & crSZ,
		std::vector<pcl::PointXYZ> & rPoints)
{
	rPoints.clear();

	pcl::PointXYZ o_;
	o_.x = crOX;
	o_.y = crOY;
	o_.z = crOZ;

	pcl::PointXYZ p_(o_);

	rPoints.push_back(o_);
	p_.x += crSX;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.z += crSZ;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.x -= crSX;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.z -= crSZ;
	rPoints.push_back(p_);

	rPoints.push_back(p_);
	p_.y += crSY;
	rPoints.push_back(p_);
	p_.x += crSX;
	rPoints.push_back(p_);
	p_.y -= crSY;
	rPoints.push_back(p_);
	p_.z += crSZ;
	rPoints.push_back(p_);
	p_.y += crSY;
	rPoints.push_back(p_);
	p_.x -= crSX;
	rPoints.push_back(p_);
	p_.y -= crSY;
	rPoints.push_back(p_);

	p_ = o_;
	p_.y += crSY;
	rPoints.push_back(p_);
	p_.x += crSX;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.z += crSZ;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.x -= crSX;
	rPoints.push_back(p_);
	rPoints.push_back(p_);
	p_.z -= crSZ;
	rPoints.push_back(p_);
}

void visualize_occupancy_grid_box (
		OccupancyGrid & crGrid,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & crViewer)
{
	std::vector<pcl::PointXYZ> points_;
	points_.reserve(24);
	
	generate_cube_points(
			crGrid.get_origin(0),
			crGrid.get_origin(1),
			crGrid.get_origin(2),
			crGrid.get_side() * crGrid.get_leaf_size(0),
			crGrid.get_side() * crGrid.get_leaf_size(1),
			crGrid.get_side() * crGrid.get_leaf_size(2),
			points_);

	for (size_t i = 0; i < 12; ++i)
	{
		std::string line_name_ = "line" + std::to_string(i);

		crViewer->addLine(
				points_[i*2+0],
				points_[i*2+1],
				0.0,
				0.0,
				0.0,
				line_name_);
	}
}

void visualize_occupancy_grid_boxes (
		OccupancyGrid & crGrid,
		boost::shared_ptr<pcl::visualization::PCLVisualizer> & crViewer)
{
	for (size_t z = 0; z < crGrid.get_side(); ++z)
	{
		for (size_t y = 0; y < crGrid.get_side(); ++y)
		{
			for (size_t x = 0; x < crGrid.get_side(); ++x)
			{
					std::string name_ = std::to_string(x) + " " +
															std::to_string(y) + " " +
															std::to_string(z);

					crViewer->addCube(
							crGrid.get_origin(0) + x * crGrid.get_leaf_size(0),
							crGrid.get_origin(0) + x * crGrid.get_leaf_size(0) + crGrid.get_leaf_size(0),
							crGrid.get_origin(1) + y * crGrid.get_leaf_size(1),
							crGrid.get_origin(1) + y * crGrid.get_leaf_size(1) + crGrid.get_leaf_size(1),
							crGrid.get_origin(2) + z * crGrid.get_leaf_size(2),
							crGrid.get_origin(2) + z * crGrid.get_leaf_size(2) + crGrid.get_leaf_size(2),
							0.0, 0.0, 0.0,
							name_);
			}
		}
	}

	std::cout << "Grid added...\n";
}


int main (int argc, char ** argv)
{
	boost::program_options::variables_map variables_map_;
	if (parse_program_options(variables_map_, argc, argv))
		return -1;

	std::string mesh_			= variables_map_[PO_MESH].as<std::string>();
	std::string cloud_		= variables_map_[PO_CLOUD].as<std::string>();
	std::string grid_			= variables_map_[PO_GRID].as<std::string>();
	bool grid_box_				= variables_map_[PO_GRID_BOX].as<bool>();
	double noise_stddev_	= variables_map_[PO_NOISE_STDDEV].as<double>();
	float occlusion_			= variables_map_[PO_OCCLUSION].as<float>();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_(
			new pcl::visualization::PCLVisualizer("viewer"));

	viewer_->initCameraParameters();
	viewer_->setBackgroundColor(1.0, 1.0, 1.0);
	//viewer_->addCoordinateSystem(5.0, 0.0f, 0.0f, 0.0f);
	viewer_->setSize(1024, 1024);

	if (!mesh_.empty())
	{
		pcl::PolygonMesh polygon_mesh_;
		pcl::PLYReader ply_reader_;

		if (ply_reader_.read(mesh_, polygon_mesh_) < 0)
		{
			pcl::console::print_error("Error reading PLY file %s...\n",
																mesh_.c_str());
			return false;
		}

		visualize_polygonal_mesh(polygon_mesh_, viewer_);
	}

	if (!cloud_.empty())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_(
				new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PCDReader pcd_reader_;

		if (pcd_reader_.read(cloud_, *point_cloud_) < 0)
		{
			pcl::console::print_error("Error reading PCD file %s...\n",
																cloud_.c_str());
			return false;
		}

		if (occlusion_ > 0.0f)
		{
			pcl::OcclusionFilter<pcl::PointXYZ> of_;
			of_.setInputCloud(point_cloud_);
			of_.setOcclusionPercentage(occlusion_);
			of_.filter(*point_cloud_);
		}

		if (noise_stddev_ > 0.0)
		{
			pcl::GaussianNoiseFilter<pcl::PointXYZ> gnf_;
			gnf_.setInputCloud(point_cloud_);
			gnf_.setNoiseMean(0.0);
			gnf_.setNoiseStdDev(noise_stddev_);
			gnf_.filter(*point_cloud_);
		}

		visualize_point_cloud(point_cloud_, viewer_);
	}

	if (!grid_.empty())
	{
		OccupancyGridAdaptive occupancy_grid_(0, 0);
		occupancy_grid_.load(grid_);

		if (grid_box_)
		{
			visualize_occupancy_grid_box(occupancy_grid_, viewer_);
			visualize_occupancy_grid(occupancy_grid_, viewer_);
		}
		else
		{
			visualize_occupancy_grid_boxes(occupancy_grid_, viewer_);
		}
	}

	while (!viewer_->wasStopped())
	{
		viewer_->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
