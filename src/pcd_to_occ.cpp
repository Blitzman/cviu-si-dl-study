#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <string>

#include <boost/program_options.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "gaussian_noise_filter.hpp"
#include "occlusion_filter.hpp"
#include "occupancy_grid_adaptive.hpp"
#include "occupancy_grid_fixed.hpp"

#define PO_HELP								"help"
#define PO_BATCH							"b"
#define PO_DIRECTORY					"d"
#define PO_MANIFEST						"m"
#define PO_LEAF_SIZE					"ls"
#define PO_GRID_SIZE					"gs"
#define PO_PADDING						"p"
#define PO_TGT_DIRECTORY			"td"
#define PO_SCALE							"s"
const std::string PO_GRID_TYPE						= "gt";
const std::string PO_NOISE_STDDEV					= "noise_stddev";
const std::string PO_OCCLUSION_PERCENTAGE = "occlusion";
#define PO_MODEL_START				"ms"
#define PO_MEAN_NORMALIZATION	"mn"

bool parse_program_options (
		boost::program_options::variables_map & rVariablesMap,
		const int & crArgc,
		char ** ppArgv)
{
	try
	{
		boost::program_options::options_description description_("Allowed options");

		description_.add_options()
			(PO_HELP, "Produce help message")
			(PO_BATCH, boost::program_options::value<bool>()->required(), "Use batch mode")
			(PO_DIRECTORY, boost::program_options::value<std::string>()->required(), "Model files directory")
			(PO_MANIFEST, boost::program_options::value<std::string>()->required(), "Model manifest file")
			(PO_LEAF_SIZE, boost::program_options::value<float>()->required(), "Leaf size")
			(PO_GRID_SIZE, boost::program_options::value<float>()->required(), "Grid size")
			(PO_PADDING, boost::program_options::value<int>()->default_value(0), "Padding")
			(PO_TGT_DIRECTORY, boost::program_options::value<std::string>()->required(), "Target directory")
			(PO_SCALE, boost::program_options::value<bool>()->default_value(false), "Scale models to fit grid")
			(PO_GRID_TYPE.c_str(), boost::program_options::value<int>()->default_value(0), "Grid type")
			(PO_NOISE_STDDEV.c_str(), boost::program_options::value<double>()->default_value(0.0), "Gaussian noise standard deviation")
			(PO_OCCLUSION_PERCENTAGE.c_str(), boost::program_options::value<float>()->default_value(0.0f), "Occlusion percentage")
			(PO_MODEL_START, boost::program_options::value<int>()->default_value(0), "Model start")
			(PO_MEAN_NORMALIZATION, boost::program_options::value<bool>()->default_value(false), "Mean normalization");

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

bool get_manifest_models (
		const std::string & crFileName,
		std::vector<std::string> & crFileNames,
		std::vector<int> & crLabels)
{
	crFileNames.clear();
	crLabels.clear();

	std::ifstream manifest_file_(crFileName);

	if (!manifest_file_.is_open())
	{
		pcl::console::print_error("Error opening manifest file %s...\n",
															crFileName.c_str());
		return false;
	}

	std::string line_ = "";
	while (std::getline(manifest_file_, line_))
	{
		std::string model_name_ = "";
		int model_label_				= -1;

		std::istringstream iss_(line_);
		if (!(iss_ >> model_name_ >> model_label_))
		{
			pcl::console::print_error("Error reading line %s...\n",
																line_.c_str());
			return false;
		}

		crFileNames.push_back(model_name_);
		crLabels.push_back(model_label_);
	}

	return true;
}

bool generate_manifest (
		const std::string & crFileName,
		const std::vector<std::string> & crFileNames,
		const std::vector<int> & crLabels)
{
	std::ofstream manifest_file_(crFileName);

	if (!manifest_file_.is_open())
	{
		pcl::console::print_error("Error opening manifest file %s...\n",
															crFileName.c_str());
		return false;
	}

	for (unsigned int i = 0; i < crFileNames.size(); ++i)
	{
		manifest_file_ << crFileNames[i];
		manifest_file_ << ".occ";
		manifest_file_ << " " << crLabels[i] << "\n";
	}

	return true;
}

bool occ_model (
		const std::string & crSource,
		const std::string & crTarget,
		const float & crLeafSize,
		const float & crGridSize,
		const bool & crScale,
		const double & crNoiseStdDev,
		const float & crOcclusion,
		OccupancyGrid & rOccupancyGrid)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_(
			new pcl::PointCloud<pcl::PointXYZ>());

	pcl::PCDReader pcd_reader_;
	if (pcd_reader_.read(crSource, *src_cloud_) < 0)
	{
		pcl::console::print_error("Error reading PCD file %s...\n",
															crSource.c_str());
		return false;
	}

	pcl::GaussianNoiseFilter<pcl::PointXYZ> gnf;
	gnf.setNoiseMean(0.0);
	gnf.setNoiseStdDev(crNoiseStdDev);
	gnf.setNoiseMagnitudes(Eigen::Vector3f(1.0f, 1.0f, 1.0f));
	pcl::OcclusionFilter<pcl::PointXYZ> of;
	of.setOcclusionPercentage(crOcclusion);

	if (crOcclusion > 0.0f)
	{
		of.setInputCloud(src_cloud_);
		of.filter(*src_cloud_);
	}

	if (crNoiseStdDev > 0.0)
	{
		std::cout << "Nosifiying input cloud " << src_cloud_->size() << " points with stddev= " << crNoiseStdDev << "...\n";

		gnf.setInputCloud(src_cloud_);
		gnf.filter(*src_cloud_);

		std::cout << "Noisy cloud has " << src_cloud_->size() << " points...\n";
	}

	{
		pcl::ScopeTime timer_("Occupancy grid computation:");
		rOccupancyGrid.compute(src_cloud_);
		rOccupancyGrid.save(crTarget + ".occ");
	}

	pcl::console::print_info("Output written in OCC format to %s...\n",
														(crTarget + ".occ").c_str());

	return true;
}

int main (int argc, char ** argv)
{
	boost::program_options::variables_map variables_map_;
	if (parse_program_options(variables_map_, argc, argv))
		return -1;

	const bool batch_mode_								= variables_map_[PO_BATCH].as<bool>();
	const std::string source_							= variables_map_[PO_DIRECTORY].as<std::string>();
	const std::string manifest_filename_	= variables_map_[PO_MANIFEST].as<std::string>();
	const float leaf_size_								= variables_map_[PO_LEAF_SIZE].as<float>();
	const float grid_size_								= variables_map_[PO_GRID_SIZE].as<float>();
	const int padding_										= variables_map_[PO_PADDING].as<int>();
	const std::string target_							= variables_map_[PO_TGT_DIRECTORY].as<std::string>();
	const bool scale_											= variables_map_[PO_SCALE].as<bool>();
	const int grid_type_									= variables_map_[PO_GRID_TYPE].as<int>();
	const double noise_stddev_						= variables_map_[PO_NOISE_STDDEV].as<double>();
	const double occlusion_percentage_		= variables_map_[PO_OCCLUSION_PERCENTAGE].as<float>();
	const int model_start_								= variables_map_[PO_MODEL_START].as<int>();
	const bool mean_normalization_				= variables_map_[PO_MEAN_NORMALIZATION].as<bool>();

	if (batch_mode_)
	{
		std::vector<std::string> filenames_;
		std::vector<int> labels_;
		
		get_manifest_models(source_ + manifest_filename_, filenames_, labels_);
		generate_manifest(target_ + manifest_filename_, filenames_, labels_);

		for (unsigned int i = model_start_; i < filenames_.size(); ++i)
		{
			pcl::console::print_highlight("Processing model %d out of %d...\n",
																			i, filenames_.size());

			//OccupancyGridFixed occupancy_grid_((int)(grid_size_/leaf_size_),
			//																		grid_size_, leaf_size_,
			//																		scale_);
			//

			OccupancyGridAdaptive occupancy_grid_(
					grid_size_/leaf_size_,
					padding_);
			OccupancyGridFixed occupancy_grid_fixed_(
					(int)(grid_size_/leaf_size_),
					grid_size_, leaf_size_,
					scale_);

			if (grid_type_ == 0)
				occ_model(source_ + filenames_[i],
								target_ + filenames_[i],
								leaf_size_,
								grid_size_,
								scale_,
								noise_stddev_,
								occlusion_percentage_,
								occupancy_grid_);
			else
				occ_model(source_ + filenames_[i],
								target_ + filenames_[i],
								leaf_size_,
								grid_size_,
								scale_,
								noise_stddev_,
								occlusion_percentage_,
								occupancy_grid_fixed_);

			pcl::console::print_highlight("Model %s OCCed...\n",
																		(source_ + filenames_[i]).c_str());

		}

		if (mean_normalization_)
		{
			OccupancyGridFixed mean_grid_((int)(grid_size_/leaf_size_),
																		grid_size_, leaf_size_,
																		scale_);
			OccupancyGridFixed max_grid_((int)(grid_size_/leaf_size_),
																		grid_size_, leaf_size_,
																		scale_, std::numeric_limits<double>::min());
			OccupancyGridFixed min_grid_((int)(grid_size_/leaf_size_),
																		grid_size_, leaf_size_,
																		scale_, std::numeric_limits<double>::max());
			OccupancyGridFixed range_grid_((int)(grid_size_/leaf_size_),
																		grid_size_, leaf_size_,
																		scale_);

			pcl::console::print_highlight("Computing means and ranges...\n");

			for (std::string f : filenames_)
			{
				OccupancyGridFixed occupancy_grid_((int)(grid_size_/leaf_size_),
																					grid_size_, leaf_size_,
																					scale_);

				pcl::console::print_highlight("Loading model %s...\n",
																				f.c_str());

				occupancy_grid_.load(target_ + f + ".occ");

				std::transform(mean_grid_.begin(),
												mean_grid_.end(),
												occupancy_grid_.begin(),
												mean_grid_.begin(),
												std::plus<double>());	
				std::transform(max_grid_.begin(),
												max_grid_.end(),
												occupancy_grid_.begin(),
												max_grid_.begin(),
												[](double a, double b) { return std::max(a, b); });
				std::transform(min_grid_.begin(),
												min_grid_.end(),
												occupancy_grid_.begin(),
												min_grid_.begin(),
												[](double a, double b) { return std::min(a, b); });
			}

			std::transform(mean_grid_.begin(),
											mean_grid_.end(),
											mean_grid_.begin(),
											[&filenames_](double a) { return a/filenames_.size(); });

			std::transform(max_grid_.begin(),
											max_grid_.end(),
											min_grid_.begin(),
											range_grid_.begin(),
											std::minus<double>());

			for (unsigned int i = 0; i < filenames_.size(); ++i)
			{
				pcl::console::print_highlight("Normalizing model %d out of %d...\n",
																			i, filenames_.size());

				OccupancyGridFixed occupancy_grid_((int)(grid_size_/leaf_size_),
																						grid_size_, leaf_size_,
																						scale_);

				occupancy_grid_.load(target_ + filenames_[i] + ".occ");

				std::transform(occupancy_grid_.begin(),
												occupancy_grid_.end(),
												mean_grid_.begin(),
												occupancy_grid_.begin(),
												std::minus<double>());

				std::transform(occupancy_grid_.begin(),
												occupancy_grid_.end(),
												range_grid_.begin(),
												occupancy_grid_.begin(),
												std::divides<double>());

				occupancy_grid_.save(target_ + filenames_[i] + ".occ");
			}
		}
	}
	else
	{
		pcl::console::print_highlight("Processing model %s ...\n",
																		(source_.c_str()));

		//OccupancyGridFixed occupancy_grid_((int)(grid_size_/leaf_size_),
		//																			grid_size_, leaf_size_,
		//																			scale_);

		OccupancyGridAdaptive occupancy_grid_(
				grid_size_/leaf_size_,
				padding_);
		OccupancyGridFixed occupancy_grid_fixed_(
					(int)(grid_size_/leaf_size_),
					grid_size_, leaf_size_,
					scale_);


		if (grid_type_ == 0)
			occ_model(source_, target_,
								leaf_size_,
								grid_size_,
								scale_,
								noise_stddev_,
								occlusion_percentage_,
								occupancy_grid_);
		else
			occ_model(source_, target_,
								leaf_size_,
								grid_size_,
								scale_,
								noise_stddev_,
								occlusion_percentage_,
								occupancy_grid_fixed_);


		pcl::console::print_highlight("Model %s OCCed...\n",
																		(source_.c_str()));
	}

	return 0;
}
