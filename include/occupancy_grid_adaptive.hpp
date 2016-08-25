#ifndef OCCUPANCY_GRID_ADAPTABLE_HPP_
#define OCCUPANCY_GRID_ADAPTABLE_HPP_

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include <boost/multi_array.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "occupancy_grid.hpp"

class OccupancyGridAdaptive : public OccupancyGrid
{
	public:
		
		OccupancyGridAdaptive	(	const int & crGridSide,
														const int & crPadding = 0)
			: OccupancyGrid(crGridSide, crPadding)
		{
		}

		~OccupancyGridAdaptive()
		{
		}

		virtual void compute(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			std::cout << "Computing adaptive occupancy grid...\n";

			Eigen::Vector4f min_p_, max_p_;
			pcl::getMinMax3D<pcl::PointXYZ>(*cpcrCloud, min_p_, max_p_);

			m_origin[0] = min_p_[0];
			m_origin[1] = min_p_[1];
			m_origin[2] = min_p_[2];

			m_grid_size[0] = max_p_[0] - min_p_[0];
			m_grid_size[1] = max_p_[1] - min_p_[1];
			m_grid_size[2] = max_p_[2] - min_p_[2];

			m_leaf_size[0] = m_grid_size[0] / (float)(m_grid_side -  2 * m_padding);
			m_leaf_size[1] = m_grid_size[1] / (float)(m_grid_side -  2 * m_padding);
			m_leaf_size[2] = m_grid_size[2] / (float)(m_grid_side -  2 * m_padding);

			// Recompute grid size to include padding
			m_grid_size[0] += 2.0f * (float)m_padding * m_leaf_size[0];
			m_grid_size[1] += 2.0f * (float)m_padding * m_leaf_size[1];
			m_grid_size[2] += 2.0f * (float)m_padding * m_leaf_size[2];

			// Translate origin to include padding
			m_origin[0] -= (float)m_padding * m_leaf_size[0];
			m_origin[1] -= (float)m_padding * m_leaf_size[1];
			m_origin[2] -= (float)m_padding * m_leaf_size[2];

			compute_occupancy_surface(cpcrCloud);

			// Check padding
			/*for (size_t z = 0; z < m_grid_side; ++z)
			{
				for (size_t y = 0; y < m_grid_side; ++y)
				{
					for (size_t x = 0; x < m_grid_side; ++x)
					{
						if (x < m_padding || x > m_grid_side - m_padding ||
								y < m_padding || y > m_grid_side - m_padding ||
								z < m_padding || z > m_grid_side - m_padding)
						{
							if (m_grid_array[z][y][x] > 0.0)
								std::cerr << "Bad padding (x,y,z): " << x << " " << y << " " << z << "\n";
						}
					}
				}
			}*/
		}

	private:
		
};

#endif
