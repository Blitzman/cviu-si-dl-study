#ifndef PCL_FILTERS_OCCLUSION_FILTER_HPP_
#define PCL_FILTERS_OCCLUSION_FILTER_HPP_

#include <random>

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
	template <typename PointT>
	class OcclusionFilter : public Filter<PointT>
	{
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;

		public:
			OcclusionFilter() : m_occlusion_percentage(0.0f)
			{

			}

			void applyFilter (PointCloud & rCloud)
			{
				const int K_ = (int)(input_->size() * m_occlusion_percentage / 100.0f);

				// Sample a random point
				pcl::PointXYZ search_point_;

				std::random_device rd_;
				std::mt19937 gen_(rd_());
				std::uniform_int_distribution<> dis_(0, input_->size()-1);
				const int SEARCH_POINT_INDEX_ = dis_(gen_);
				search_point_ = input_->at(SEARCH_POINT_INDEX_);
				
				// Find K nearest neighbors to the sample point
				std::vector<int> neighbors_indices_(K_);
				std::vector<float> neighbors_squared_dist_(K_);

				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
				kdtree_.setInputCloud(input_);

				kdtree_.nearestKSearch (
						search_point_,
						K_,
						neighbors_indices_,
						neighbors_squared_dist_);

				// Remove the neighbors and the point
				pcl::PointIndices::Ptr indices_(new pcl::PointIndices());
				indices_->indices = neighbors_indices_;
				indices_->indices.push_back(SEARCH_POINT_INDEX_);

				pcl::ExtractIndices<pcl::PointXYZ> ei_;
				ei_.setInputCloud(input_);
				ei_.setIndices(indices_);
				ei_.setNegative(true);
				ei_.filter(rCloud);
			}

			float getOcclusionPercentage () const
			{
				return m_occlusion_percentage;
			}

			void setOcclusionPercentage (const float & crOcclusionPercentage)
			{
				m_occlusion_percentage = crOcclusionPercentage;
			}

		private:
			float m_occlusion_percentage;
	};
}

#endif
