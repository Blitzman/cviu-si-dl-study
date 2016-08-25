#include <algorithm>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include <boost/program_options.hpp>

const std::string PO_HELP									= "help";
const std::string PO_SOURCE								= "src";
const std::string PO_TARGET								= "tgt";
const std::string PO_FOLDS								= "folds";
const std::string PO_VIEWS								= "views";
const std::string PO_CLASSES							= "classes";

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
			(PO_SOURCE.c_str(), boost::program_options::value<std::string>()->required(), "Source")
			(PO_TARGET.c_str(), boost::program_options::value<std::string>()->required(), "Target directory")
			(PO_FOLDS.c_str(), boost::program_options::value<int>()->default_value(5), "Folds for k-fold validation")
			(PO_VIEWS.c_str(), boost::program_options::value<int>()->default_value(80), "Views per model")
			(PO_CLASSES.c_str(), boost::program_options::value<int>()->default_value(10), "Total classes");

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

bool read_manifest (
		const std::string & crManifestFilename,
		std::vector<std::string> & rFilenames,
		std::vector<int> & rLabels)
{
	std::ifstream manifest_file_(crManifestFilename);

	if (!manifest_file_.is_open())
	{
		std::cerr << "Error opening manifest file " << crManifestFilename << "...\n";
		return false;
	}

	std::string line_ = "";
	while (std::getline(manifest_file_, line_))
	{
		std::string model_	= "";
		int label_					= -1;

		std::istringstream iss_(line_);
		if (!(iss_ >> model_ >> label_))
		{
			std::cerr << "Error reading line " << line_ << "...\n";
			return false;
		}

		rFilenames.push_back(model_);
		rLabels.push_back(label_);
	}
}

void output_manifest (
		const std::string & crManifestFilename,
		const std::vector<std::string> & crFilenames,
		const std::vector<int> & crLabels)
{
	std::cout << "Outputting manifest " << crManifestFilename << "...\n";

	std::ofstream manifest_file_(crManifestFilename);
	for (unsigned int i = 0; i < crFilenames.size(); ++i)
	{
		manifest_file_ << crFilenames[i] << " ";	
		manifest_file_ << crLabels[i] << "\n";
	}
	manifest_file_.close();
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
template <class T, class K>
void shuffle_two_vectors (
		std::vector<T> & rVectorA,
		std::vector<K> & rVectorB)
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

bool sample_model_class (
		std::vector<std::string> & rFilenames,
		std::vector<int> & rLabels,
		const int & crClass,
		const int & crViewsPerModel,
		std::vector<std::string> & rFoldFilenames,
		std::vector<int> & rFoldLabels)
{
	for (size_t i = 0; i < rFilenames.size(); ++i)
	{
		if (rLabels.at(i) == crClass)
		{
			for (size_t j = i; j < i+crViewsPerModel; ++j)
			{
				rFoldFilenames.push_back(rFilenames.at(i));
				rFoldLabels.push_back(rLabels.at(i));

				rFilenames.erase(rFilenames.begin()+i);
				rLabels.erase(rLabels.begin()+i);
			}

			return true;
		}
	}

	return false;
}

void generate_kfold_manifests (
		const std::string & crManifestFilename,
		const std::vector<std::string> & crFilenames,
		const std::vector<int> & crLabels,
		const int & crViewsPerModel,
		const int & crClasses,
		const int & crFolds)
{
	std::cout << "Generating k-fold manifests...\n";

	int num_models_ = crFilenames.size() / crViewsPerModel;

	std::vector<std::string> filenames_(crFilenames);
	std::vector<int> labels_(crLabels);

	std::vector<std::vector<std::string>> folds_filenames_;
	std::vector<std::vector<int>> folds_labels_;

	folds_filenames_.resize(crFolds);
	folds_labels_.resize(crFolds);

	std::cout << "Filling folds...\n";

	int fold_ = 0;
	for (size_t i = 0; i < num_models_;)
	{
		for (int c = 0; c < crClasses; ++c)
		{
			std::cout << "Models " << i << "...\n";
			if (sample_model_class (
						filenames_,
						labels_,
						c,
						crViewsPerModel,
						folds_filenames_[fold_],
						folds_labels_[fold_]))
				++i;
		}

		fold_ = (fold_ + 1) % crFolds;
	}

	std::cout << "Generating output folds...\n";

	//  Output folds manifests
	std::vector<std::string> train_filenames_;
	std::vector<int> train_labels_;
	std::vector<std::string> test_filenames_;
	std::vector<int> test_labels_;

	for (size_t k = 0; k < crFolds; ++k)
	{
		std::cout << "Fold " << k << " is the test fold...\n";

		for (size_t j = 0; j < crFolds; ++j)
		{
			if (j == k)
			{
				test_filenames_.insert(test_filenames_.end(),
																folds_filenames_[j].begin(),
																folds_filenames_[j].end());
				test_labels_.insert(test_labels_.end(),
																folds_labels_[j].begin(),
																folds_labels_[j].end());
			}
			else
			{
				train_filenames_.insert(train_filenames_.end(),
																folds_filenames_[j].begin(),
																folds_filenames_[j].end());
				train_labels_.insert(train_labels_.end(),
																folds_labels_[j].begin(),
																folds_labels_[j].end());
			}
		}

		//shuffle_two_vectors(train_filenames_, test_filenames_);

		std::cout << "Outputting folds...\n";

		output_manifest(crManifestFilename + "_train_" + std::to_string(k),
										train_filenames_, train_labels_);
		output_manifest(crManifestFilename + "_test_" + std::to_string(k),
										test_filenames_, test_labels_);

		test_filenames_.clear();
		test_labels_.clear();
		train_filenames_.clear();
		train_labels_.clear();
	}
}

// Main entry point
//
int main (int argc, char **argv)
{
	// Parse program options
	boost::program_options::variables_map variables_map_;
	if (parse_program_options(variables_map_, argc, argv))
		return 1;

	const std::string source_				= variables_map_[PO_SOURCE].as<std::string>();
	const std::string target_				= variables_map_[PO_TARGET].as<std::string>();
	const int folds_								= variables_map_[PO_FOLDS].as<int>();
	const int views_								= variables_map_[PO_VIEWS].as<int>();
	const int classes_							= variables_map_[PO_CLASSES].as<int>();

	std::vector<std::string> models_;
	std::vector<int> labels_;

	read_manifest(source_, models_, labels_);
	generate_kfold_manifests(
			target_,
			models_,
			labels_,
			views_,
			classes_,
			folds_);
}
