#include <algorithm>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include <boost/program_options.hpp>

#include <pcl/console/print.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "mesh_sampler.hpp"
#include "off_handler.hpp"
#include "ply_handler.hpp"

const std::string PO_HELP									= "help";
const std::string PO_BATCH_MODE						= "batch";
const std::string PO_SOURCE								= "src";
const std::string PO_TARGET								= "tgt";
const std::string PO_TESS_LEVEL						= "tl";
const std::string PO_RESOLUTION						= "res";
const std::string PO_USE_VERTICES					= "vert";
const std::string PO_NOISE_STDDEV					= "noise_stddev";
const std::string PO_OCCLUSION						= "occlusion";
const std::string PO_MODEL_START_TRAINING	= "ms_training";
const std::string PO_MODEL_START_TEST			= "ms_test";
const std::string PO_FOLDS								= "folds";
const std::string PO_TEST_PERCENTAGE			= "test_percent";

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
			(PO_BATCH_MODE.c_str(), boost::program_options::value<bool>()->required(), "Batch mode")
			(PO_SOURCE.c_str(), boost::program_options::value<std::string>()->required(), "Source")
			(PO_TARGET.c_str(), boost::program_options::value<std::string>()->required(), "Target directory")
			(PO_TESS_LEVEL.c_str(), boost::program_options::value<int>()->default_value(1), "Tessellation level")
			(PO_RESOLUTION.c_str(), boost::program_options::value<int>()->default_value(100), "Resolution")
			(PO_USE_VERTICES.c_str(), boost::program_options::value<bool>()->default_value(true), "Use vertices of the tessellated sphere")
			(PO_NOISE_STDDEV.c_str(), boost::program_options::value<double>()->default_value(0.01), "Gaussian noise standard deviation")
			(PO_OCCLUSION.c_str(), boost::program_options::value<float>()->default_value(0.0f), "Occlusion percentage")
			(PO_MODEL_START_TRAINING.c_str(), boost::program_options::value<int>()->default_value(0), "Model start training")
			(PO_MODEL_START_TEST.c_str(), boost::program_options::value<int>()->default_value(0), "Model start test")
			(PO_FOLDS.c_str(), boost::program_options::value<int>()->default_value(5), "Folds for k-fold validation")
			(PO_TEST_PERCENTAGE.c_str(), boost::program_options::value<float>()->default_value(10.0f), "Test percentage per fold");

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

// Gets filenames and paths for all the OFF models inside the specified folder.
//
// Results are stored in independent vectors for those OFF models depending on
// them being part of the training or test sets.
//
void get_off_filenames_paths (
	const std::string & crDirectoryPath, 
	std::vector<std::string> & rTrainFileNames,
	std::vector<std::string> & rTrainFilePaths,
	std::vector<std::string> & rTestFileNames,
	std::vector<std::string> & rTestFilePaths)
{
	boost::filesystem::path directory_(crDirectoryPath);
	rTrainFileNames.clear();
	rTrainFilePaths.clear();
	rTestFileNames.clear();
	rTestFilePaths.clear();

	if (!boost::filesystem::exists(directory_) || 
			!boost::filesystem::is_directory(directory_))
		return;

	boost::filesystem::recursive_directory_iterator it_(directory_);
	boost::filesystem::recursive_directory_iterator endit_;

	while (it_ != endit_)
	{
		if (boost::filesystem::is_regular_file(*it_) && 
				it_->path().extension() == ".off")
		{
			std::string file_name_ = it_->path().filename().string();
			std::cout << file_name_ << "\n";
			
			std::string file_path_ = it_->path().string();
			file_path_.erase(file_path_.size() - file_name_.size());
			std::cout << file_path_ << "\n";

			if (it_->path().string().find("train") != std::string::npos)
			{
				rTrainFileNames.push_back(file_name_);
				rTrainFilePaths.push_back(file_path_);
			}
			else if (it_->path().string().find("test") != std::string::npos)
			{
				rTestFileNames.push_back(file_name_);
				rTestFilePaths.push_back(file_path_);
			}
		}

		++it_;
	}
}

// Assigns an integer label to each unique element  of a given vector of
// file paths. The labels are assigned in increasing order as new unique
// paths are iterated over.
//
// The result is stored in a given unordered map whose content is cleared
// before starting the assignment process.
//
void assign_labels (
		const std::vector<std::string> & rFilePaths,
		std::unordered_map<std::string, int> & rPathLabels)
{
	int current_label_ = 0;

	rPathLabels.clear();

	for (const std::string & file_path_ : rFilePaths)
	{
		if (rPathLabels.find(file_path_) == rPathLabels.end())
			rPathLabels.insert(std::make_pair(file_path_, current_label_++));
	}
}

// Reorders a vector given a set of indices which indicate the new positions
// of the elements.
//
// The input vector elements are replaced by the ordered elements.
//
template <class T>
void reorder_vector (
		std::vector<T> & rVector,
		const std::vector<size_t> & crIndices)
{
	std::vector<T> aux_(rVector);
	
	for (size_t i = 0; i < rVector.size(); ++i)
		rVector[i] = aux_[crIndices[i]];
}

// Shuffles two vectors simultaneously and identically.
//
// The input vectors elements are replaced by the shuffled ones.
//
template <class T>
void shuffle_two_vectors (
		std::vector<T> & rVectorA,
		std::vector<T> & rVectorB)
{
	std::cout << "Shuffling...\n";

	std::vector<size_t> indices_;
	indices_.reserve(rVectorA.size());

	for (unsigned int i = 0; i < rVectorA.size(); ++i)
		indices_.push_back(i);

	std::random_shuffle(indices_.begin(), indices_.end());

	reorder_vector(rVectorA, indices_);
	reorder_vector(rVectorB, indices_);
}

void output_manifest (
		const std::string & crManifestFilename,
		const std::vector<std::string> & crFilenames,
		const std::vector<std::string> & crPaths,
		const std::unordered_map<std::string, int> & crPathLabels)
{
	std::cout << "Outputting manifest " << crManifestFilename << "...\n";

	std::ofstream manifest_file_(crManifestFilename);
	for (unsigned int i = 0; i < crFilenames.size(); ++i)
	{
		manifest_file_ << crFilenames[i] << " ";	
		manifest_file_ << crPathLabels.at(crPaths[i]) << "\n";
	}
	manifest_file_.close();
}

// Outputs a manifest file which consists of lines containing a model path,
// a white space, and its corresponding label. The contents are optionally
// shuffled by enabling the adequate flag.
//
// The contents of the file are replaced by the new ones.
//
void generate_manifest (
		const std::string & crModelsManifestFilename,
		const std::string & crViewsManifestFilename,
		const std::vector<std::string> & crFilenames,
		const std::vector<std::string> & crPaths,
		const bool & crShuffle,
		const int & crViewsPerModel)
{
	std::vector<std::string> models_filenames_(crFilenames);
	std::vector<std::string> views_filenames_;
	std::vector<std::string> models_paths_(crPaths);
	std::vector<std::string> views_paths_;

	// Assign labels to models file paths
	std::unordered_map<std::string, int> path_labels_;
	assign_labels(models_paths_, path_labels_);

	std::cout << "Generating views filenames and paths...\n";
	// Generate views filenames and paths
	views_filenames_.resize(crViewsPerModel * models_filenames_.size());
	views_paths_.resize(crViewsPerModel * models_paths_.size());

	for (size_t i = 0; i < models_filenames_.size(); ++i)
	{
		for (size_t j = 0; j < crViewsPerModel; ++j)
		{
			std::string view_name_ = "_view" + std::to_string(j) + ".pcd";
			views_filenames_[i*crViewsPerModel+j] = models_filenames_[i] + view_name_;
			views_paths_[i*crViewsPerModel+j] = models_paths_[i];
		}
	}
	// Generate models filenames
	for (auto it = models_filenames_.begin(); it != models_filenames_.end(); ++it)
		(*it) = (*it) + "_sampled.pcd";

	// Shuffle filenames and paths, this is usually required by the training
	// set in order not to bias the training process
	if (crShuffle)
	{
		std::cout << "Shuffling...\n";

		for (auto & x: path_labels_)
			std::cout << x.first << ": " << x.second << "\n";

		shuffle_two_vectors(models_filenames_, models_paths_);
		shuffle_two_vectors(views_filenames_, views_paths_);
	}

	// Output models filenames and labels to manifest file
	output_manifest(
			crModelsManifestFilename,
			models_filenames_,
			models_paths_,
			path_labels_);
	// Output views filenames and labels to manifest file
	output_manifest(
			crViewsManifestFilename,
			views_filenames_,
			views_paths_,
			path_labels_);
}

void sample_model_class (
		std::vector<std::string> & rFilenames,
		std::vector<std::string> & rPaths,
		const std::unordered_map<std::string, int> & rPathLabels,
		const int & crClass,
		std::string & rModelFilename,
		std::string & rModelPath)
{
	for (size_t i = 0; i < rFilenames.size(); ++i)
	{
		if (rPathLabels.at(rPaths.at(i)) == crClass)
		{
			rModelFilename = rFilenames.at(i);
			rModelPath = rPaths.at(i);

			rFilenames.erase(rFilenames.begin()+i);
			rPaths.erase(rPaths.begin()+i);

			break;
		}
	}
}

void generate_kfold_manifests (
		const std::string & crModelsManifestFilename,
		const std::string & crViewsManifestFilename,
		const std::vector<std::string> & crFilenames,
		const std::vector<std::string> & crPaths,
		const float & crTestPercentage,
		const int & crViewsPerModel,
		const int & crFolds)
{
	std::cout << "Generating k-fold manifests...\n";

	// Assign labels to models file paths
	std::unordered_map<std::string, int> path_labels_;
	assign_labels(crPaths, path_labels_);

	const int test_elements_ = (int)(crFilenames.size() / crTestPercentage);
	const int num_classes_ = path_labels_.size();

	for (unsigned int k = 0; k < crFolds; ++k)
	{
		std::cout << "Generating fold " << k << "...\n";

		std::vector<std::string> models_train_filenames_(crFilenames);
		std::vector<std::string> models_train_paths_(crPaths);
		std::vector<std::string> models_test_filenames_(test_elements_);
		std::vector<std::string> models_test_paths_(test_elements_);

		shuffle_two_vectors(models_train_filenames_, models_train_paths_);

		int class_ = 0;
		for (unsigned int i = 0; i < test_elements_; ++i)
		{
			std::string test_model_filename_;
			std::string test_model_path_;

			sample_model_class(
					models_train_filenames_,
					models_train_paths_,
					path_labels_,
					class_,
					test_model_filename_,
					test_model_path_);

			models_test_filenames_[i] = test_model_filename_;
			models_test_paths_[i] = test_model_path_;

			class_ = (class_ + 1) % num_classes_;
		}

		output_manifest(
				crModelsManifestFilename + "_train_fold" + std::to_string(k) + ".txt",
				models_train_filenames_,
				models_train_paths_,
				path_labels_);

		output_manifest(
				crModelsManifestFilename + "_test_fold" + std::to_string(k) + ".txt",
				models_test_filenames_,
				models_test_paths_,
				path_labels_);
	}
}

// Converts a specified OFF model mesh to a PCD/PLY point cloud by sampling
// the model from different points of view of a tessellated sphere with a given
// tessellation level and render resolution.
//
// The point cloud is written to the specified target in PCD or PLY format.
//
bool convert_model (
		const std::string & crSource,
		const std::string & crModelTarget,
		const std::string & crViewsTarget,
		const int & crTessellationLevel,
		const int & crResolution,
		const double & crNoiseStdDev,
		const float & crOcclusion,
		const bool & crUseVertices)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr off_cloud_ptr_(
			new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<pcl::Vertices> off_vertices_;

	OFFHandler off_handler_;
	if (off_handler_.read(crSource, off_cloud_ptr_, off_vertices_) < 0)
	{
		pcl::console::print_error("Error while reading %s...\n", crSource.c_str());
		return false;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud_ptr_(
			new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> sampled_views_;
	std::vector<pcl::Vertices> sampled_vertices_;

	// Sample OFF mesh
	MeshSampler mesh_sampler_;
	mesh_sampler_.sample_mesh_raytracing(off_cloud_ptr_,
																				off_vertices_,
																				crTessellationLevel,
																				crResolution,
																				crNoiseStdDev,
																				crOcclusion,
																				crUseVertices,
																				sampled_views_,
																				sampled_cloud_ptr_);

	pcl::PCDWriter pcd_writer_;
	// Output full cloud
	pcd_writer_.write(crModelTarget + "_sampled.pcd", *sampled_cloud_ptr_);
	pcl::console::print_highlight("Model sampled and stored in PCD format %s...\n",
																	(crModelTarget + "_sampled.pcd").c_str());
	// Output view clouds
	for (size_t i = 0; i < sampled_views_.size(); ++i)
	{
		std::string view_name_ = "_view" + std::to_string(i) + ".pcd";
		pcd_writer_.write(crViewsTarget + view_name_, *(sampled_views_[i]), true);
		pcl::console::print_highlight("View sampled and stored in PCD format %s...\n",
																		view_name_.c_str());
	}

	return true;
}

// Main entry point
//
int main (int argc, char **argv)
{
	// Parse program options
	boost::program_options::variables_map variables_map_;
	if (parse_program_options(variables_map_, argc, argv))
		return 1;

	const bool batch_mode_					= variables_map_[PO_BATCH_MODE].as<bool>();
	const std::string source_				= variables_map_[PO_SOURCE].as<std::string>();
	const std::string target_				= variables_map_[PO_TARGET].as<std::string>();
	const int tess_level_						= variables_map_[PO_TESS_LEVEL].as<int>();
	const int resolution_						= variables_map_[PO_RESOLUTION].as<int>();
	const double noise_stddev_			= variables_map_[PO_NOISE_STDDEV].as<double>();
	const float occlusion_					= variables_map_[PO_OCCLUSION].as<float>();
	const bool use_vertices_				= variables_map_[PO_USE_VERTICES].as<bool>();
	const int model_start_training_	= variables_map_[PO_MODEL_START_TRAINING].as<int>();
	const int model_start_test_			= variables_map_[PO_MODEL_START_TEST].as<int>();
	const int folds_								= variables_map_[PO_FOLDS].as<int>();
	const int test_percentage_			= variables_map_[PO_TEST_PERCENTAGE].as<float>();

	const int num_views_			= (use_vertices_) ? (40 * (int)(pow(4.0, tess_level_-1)) + 2)
																: 20 * (int)(pow(4.0, tess_level_));

	// Batch mode will process all the OFF models of the specified directory in
	// the PO_SOURCE program option and then output all the files in PO_TARGET
	if (batch_mode_)
	{
		// Get training and test OFF models filenames and paths
		pcl::console::print_highlight("Obtaining OFF filenames and paths...\n");
		std::vector<std::string> off_train_filenames_, off_test_filenames_;
		std::vector<std::string> off_train_paths_, off_test_paths_;
		get_off_filenames_paths(source_,
														off_train_filenames_,
														off_train_paths_,
														off_test_filenames_,
														off_test_paths_);

		// Generate manifest files
		pcl::console::print_highlight("Generating manifest files...\n");
		generate_manifest(target_ + "/manifests/models_training_manifest.txt",
										target_ + "/manifests/views_training_manifest.txt",
										off_train_filenames_,
										off_train_paths_,
										false,
										num_views_);
		generate_manifest(target_ + "/manifests/models_validation_manifest.txt",
										target_ + "/manifests/views_validation_manifest.txt",
										off_test_filenames_,
										off_test_paths_,
										false,
										num_views_);

		// Generate k-fold manifests
		// generate_kfold_manifests(target_ + "models",
		//												target_ + "views",
		//												off_train_filenames_,
		//												off_train_paths_,
		//												test_percentage_,
		//												num_views_,
		//												folds_);

		// Convert training models
		for (size_t i = model_start_training_; i < off_train_filenames_.size(); ++i)
		{
			pcl::console::print_highlight("Processing training model %d out of %d...\n",
																i, off_train_filenames_.size()-1);

			convert_model(off_train_paths_[i] + off_train_filenames_[i],
										target_ + "/model/training/" + off_train_filenames_[i],
										target_ + "/views/training/" + off_train_filenames_[i],
										tess_level_,
										resolution_,
										noise_stddev_,
										occlusion_,
										use_vertices_);
		}

		// Convert validation models
		for (size_t i = model_start_test_; i < off_test_filenames_.size(); ++i)
		{
			pcl::console::print_highlight("Processing validation model %d out of %d...\n",
																i, off_test_filenames_.size()-1);

			convert_model(off_test_paths_[i] + off_test_filenames_[i],
										target_ + "/model/validation/" + off_test_filenames_[i],
										target_ + "/views/validation/" + off_test_filenames_[i],
										tess_level_,
										resolution_,
										noise_stddev_,
										occlusion_,
										use_vertices_);
		}
	}
	// If batch mode is not enabled, the OFF file specified by PO_SOURCE will be
	// processed and its output will be PO_TARGET
	else
	{
		pcl::console::print_info("Batch mode DISABLED...\n");
		pcl::console::print_highlight("Processing model %s...\n",
															source_.c_str());

		convert_model(source_,
									target_,
									target_,
									tess_level_,
									resolution_,
									noise_stddev_,
									occlusion_,
									use_vertices_);
	}
}
