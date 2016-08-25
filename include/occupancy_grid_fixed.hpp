#ifndef OCCUPANCY_GRID_FIXED_HPP_
#define OCCUPANCY_GRID_FIXED_HPP_

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include <boost/multi_array.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "occupancy_grid.hpp"

class OccupancyGridFixed : public OccupancyGrid
{
	public:
		
		typedef boost::multi_array<double, 3> grid_array_type;
		typedef grid_array_type::index grid_array_index;

		OccupancyGridFixed	(	const int & crGridSide,
													const float & crGridSize,
													const float & crLeafSize,
													const bool & crScale,
													const int & crPadding = 0)
			: OccupancyGrid(crGridSide, crPadding),
				m_scale			(crScale)
		{
			m_grid_size = {crGridSize, crGridSize, crGridSize};
			m_leaf_size = {crLeafSize, crLeafSize, crLeafSize};
		}

		~OccupancyGridFixed	()
		{
		}

		virtual void compute	(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(
					new pcl::PointCloud<pcl::PointXYZ>());
			*cloud_ = *cpcrCloud;

			if (m_scale)
			{
					float scale_factor_ = get_scale_factor(
							cloud_,
							m_grid_size[0],
							m_grid_size[1],
							m_grid_size[2]);
					scale_point_cloud(scale_factor_, cloud_);

					//pcl::io::savePCDFileASCII("test.pcd", *cloud_);
			}

			Eigen::Vector4f min_p_, max_p_;
			pcl::getMinMax3D<pcl::PointXYZ>(*cloud_, min_p_, max_p_);

			m_origin[0] = min_p_[0];
			m_origin[1] = min_p_[1];
			m_origin[2] = min_p_[2];

			compute_occupancy_density(cloud_);
		}

		virtual void compute	(	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpCloud,
														const pcl::Vertices & cpFaces)
		{
			compute(cpCloud);
		}

	private:

		bool m_scale;
		
	float get_scale_factor(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr cpCloud,
		const float & crMaxX,
		const float & crMaxY,
		const float & crMaxZ)
	{
		Eigen::Vector4f min_p_, max_p_;
		pcl::getMinMax3D<pcl::PointXYZ>(*cpCloud, min_p_, max_p_);

	  float d_x_ = max_p_[0] - min_p_[0];
		float d_y_ = max_p_[1] - min_p_[1];
	  float d_z_ = max_p_[2] - min_p_[2];

		float scale_ = 0;

	  if(d_x_ > d_y_ && d_x_ > d_z_)
		  scale_ = crMaxX / d_x_;
	  else if (d_y_ > d_x_ && d_y_ > d_z_)
		  scale_ = crMaxY / d_y_;
	  else if (d_z_ > d_x_ && d_z_ > d_y_)
			scale_ = crMaxZ / d_z_;
 
		return scale_;
	}

	void scale_point_cloud(
		const float & crFactor,
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
	{
		for(unsigned int i = 0; i < pCloud->points.size(); ++i)
	  {
		  pCloud->points[i].x *= crFactor;
			pCloud->points[i].y *= crFactor;
			pCloud->points[i].z *= crFactor;
		}
	}
};

#endif
