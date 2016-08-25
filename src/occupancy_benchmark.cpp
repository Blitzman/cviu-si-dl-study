#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/timer.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "occupancy_grid_adaptive.hpp"


double compute_resolution
	(
			const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pCloud
				)
{
				double resolution = 0.0;
					int points = 0;
						int nres;

							std::vector<int> indices(2);
								std::vector<float> sqrDistances(2);
									pcl::search::KdTree<pcl::PointXYZ> kdtree;
										kdtree.setInputCloud(pCloud);

											for (size_t i = 0; i < pCloud->size(); ++i)
													{
																if (!pcl_isfinite((*pCloud)[i].x))
																				continue;

																		nres = kdtree.nearestKSearch(i, 2, indices, sqrDistances);

																				if (nres == 2)
																							{
																											resolution += sqrt(sqrDistances[1]);
																														++points;
																																}
																					}

												if (points != 0)
															resolution /= points;

	return resolution;
}

int main (void)
{
	const int grid_side_ = 16;
	const int padding_ = 2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("example.pcd", *cloud_) == -1) //* load the file
	{
		std::cerr << "Couldn't read file test_pcd.pcd \n";
		return (-1);
	}

	// Cloud size test
	std::vector<std::string> results_;
	/*for (size_t i = 1; i < 24; ++i)
	{
		std::cout << "Iteration: " << i << "\n";

		float radius_search_ = (24-i) * compute_resolution(cloud_);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_d_(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PointCloud<int> sampled_indices_;
		pcl::UniformSampling<pcl::PointXYZ> us_;
		us_.setInputCloud(cloud_);
		us_.setRadiusSearch(radius_search_);
		us_.compute(sampled_indices_);
		pcl::copyPointCloud(*cloud_, sampled_indices_.points, *cloud_d_);

		std::cout << "Testing cloud with " << cloud_d_->size() << " points...\n";

		std::string filename_ = "test_cloud_" + std::to_string(i) + ".pcd";
		pcl::io::savePCDFileASCII (filename_, *cloud_d_);

		double mean_time_ = 0.0;

		for (size_t k = 0; k < 3; ++k)
		{
			OccupancyGridAdaptive occupancy_grid_(
				grid_side_,
				padding_);

			boost::timer t_;
			occupancy_grid_.compute(cloud_d_);
			double elapsed_time_ = t_.elapsed();
			mean_time_ += elapsed_time_;
		}

		std::cout << "Cloud tested...\n";

		mean_time_ /= 10.0;
		std::cout << (std::to_string(cloud_d_->size()) + " " + std::to_string(mean_time_)) << "\n";
		results_.push_back(std::to_string(cloud_d_->size()) + " " + std::to_string(mean_time_));
	}*/

	// Print summary
	for (auto it = results_.begin(); it != results_.end(); ++it)
		std::cout << (*it) << "\n";

	results_.clear();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_d_(new pcl::PointCloud<pcl::PointXYZ>);

	float radius_search_ = 8 * compute_resolution(cloud_);
	pcl::PointCloud<int> sampled_indices_;
	pcl::UniformSampling<pcl::PointXYZ> us_;
	us_.setInputCloud(cloud_);
	us_.setRadiusSearch(radius_search_);
	us_.compute(sampled_indices_);
	pcl::copyPointCloud(*cloud_, sampled_indices_.points, *cloud_d_);

	std::cout << "Testing cloud with " << cloud_d_->size() << " points...\n";

	// Grid size tests
	for (size_t i = 8; i < 64; ++i)
	{
		std::cout << "Grid side " << i << "...\n";

		double mean_time_ = 0.0;
		for (size_t k = 0; k < 3; ++k)
		{
			OccupancyGridAdaptive occupancy_grid_(
				i,
				padding_);

			boost::timer t_;
			occupancy_grid_.compute(cloud_d_);
			double elapsed_time_ = t_.elapsed();
			mean_time_ += elapsed_time_;
		}

		std::cout << "Cloud tested...\n";

		mean_time_ /= 3.0;
		std::cout << (std::to_string(i) + " " + std::to_string(mean_time_)) << "\n";
		results_.push_back(std::to_string(i) + " " + std::to_string(mean_time_));
	}

	// Print summary
	for (auto it = results_.begin(); it != results_.end(); ++it)
		std::cout << (*it) << "\n";

}
