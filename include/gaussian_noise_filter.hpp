#ifndef PCL_FILTERS_GAUSSIAN_NOISE_FILTER_HPP_
#define PCL_FILTERS_GAUSSIAN_NOISE_FILTER_HPP_

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace pcl
{
	template <typename PointT>
	class GaussianNoiseFilter : public Filter<PointT>
	{
		using Filter<PointT>::input_;
		typedef typename Filter<PointT>::PointCloud PointCloud;

		public:
			GaussianNoiseFilter() : m_noise_mean (0.0),
															m_noise_stddev (0.001),
															m_noise_magnitudes (0.0f, 0.0f, 0.0f)
			{

			}

			void applyFilter (PointCloud & rCloud)
			{
				struct timeval start_;
				gettimeofday(&start_, NULL);
				boost::mt19937 rng_;
				rng_.seed(start_.tv_usec);
				boost::normal_distribution<> nd_(m_noise_mean, m_noise_stddev);
				boost::variate_generator<boost::mt19937&, boost::normal_distribution<>>
					var_nor_(rng_, nd_);

				for (auto it = rCloud.begin(); it != rCloud.end(); ++it)
				{
					(*it).x += var_nor_() * m_noise_magnitudes[0];
					(*it).y += var_nor_() * m_noise_magnitudes[1];
					(*it).z += var_nor_() * m_noise_magnitudes[2];
				}
			}

			void setNoiseMean (const double & crNoiseMean)
			{
				m_noise_mean = crNoiseMean;
			}
			void setNoiseStdDev (const double & crNoiseStdDev)
			{
				m_noise_stddev = crNoiseStdDev;
			}
			void setNoiseMagnitudes (const Eigen::Vector3f crNoiseMagnitudes)
			{
				m_noise_magnitudes = crNoiseMagnitudes;
			}

			double getNoiseMean () const
			{
				return m_noise_mean;
			}
			double getNoiseStdDev () const
			{
				return m_noise_stddev;
			}
			Eigen::Vector3f getNoiseMagnitudes () const
			{
				return m_noise_magnitudes;
			}

		private:
			double m_noise_mean;
			double m_noise_stddev;
			Eigen::Vector3f m_noise_magnitudes;
	};
}

#endif
