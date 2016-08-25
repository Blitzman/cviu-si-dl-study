#ifndef OCCUPANCY_GRID_HPP_
#define OCCUPANCY_GRID_HPP_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include <boost/multi_array.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

class OccupancyGrid
{
	using grid_array_type		= boost::multi_array<double, 3>;
	using grid_array_index	=  grid_array_type::index;

	public:

		OccupancyGrid	(	const int & crGridSide,
										const int & crPadding = 0)
			: m_leaf_size			{0.0f, 0.0f, 0.0f},
				m_grid_size			{0.0f, 0.0f, 0.0f},
				m_grid_side			(crGridSide),
				m_origin				{0.0f, 0.0f, 0.0f},
				m_padding				(crPadding),
				m_max_occupancy	{0.0}
		{
			pcl::console::print_info("Creating occupancy grid with side %d...\n",
																m_grid_side);

			fill_grid(0.0);

			pcl::console::print_info("Grid filled with default value %f...\n",
																0.0);
		}

		virtual ~OccupancyGrid()
		{
		}

		virtual void compute(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpCloud) = 0;

		void compute_occupancy_binary(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			for (auto it = cpcrCloud->begin(); it != cpcrCloud->end(); ++it)
			{
				size_t x = compute_voxel_index(m_origin[0], (*it).x, m_leaf_size[0]);
				size_t y = compute_voxel_index(m_origin[1], (*it).y, m_leaf_size[1]);
				size_t z = compute_voxel_index(m_origin[2], (*it).z, m_leaf_size[2]);

				m_grid_array[z][y][x] = 1.0;
			}

			m_max_occupancy = 1.0;
		}
		void compute_occupancy_density_bruteforce(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			double max_occupancy_ = -1.0 * std::numeric_limits<double>::max();

			grid_array_index i_idx = 0;
			grid_array_index j_idx = 0;
			grid_array_index k_idx = 0;

			double x_o_ = m_origin[0];
			double y_o_ = m_origin[1];
			double z_o_ = m_origin[2];

			for (double i = x_o_; i < x_o_ + m_grid_size[0]; i += m_leaf_size[0])
			{
				for (double j = y_o_; j < y_o_ + m_grid_size[1]; j += m_leaf_size[1])
				{
					for (double k = z_o_; k < z_o_ + m_grid_size[2]; k += m_leaf_size[2])
					{
						for (unsigned int m = 0; m < cpcrCloud->size(); ++m)
						{
							double x_ = cpcrCloud->points[m].x;
							double y_ = cpcrCloud->points[m].y;
							double z_ = cpcrCloud->points[m].z;

							if (x_ > i && x_ < i + m_leaf_size[0] &&
									y_ > j && y_ < j + m_leaf_size[1] &&
									z_ > k && z_ < k + m_leaf_size[2])
							{
								m_grid_array[i_idx][j_idx][k_idx] += 1.0;
							}

							max_occupancy_ = (m_grid_array[i_idx][j_idx][k_idx] > max_occupancy_) ?
								m_grid_array[i_idx][j_idx][k_idx] :
								max_occupancy_;
						}
						++k_idx;
					}
					k_idx = 0;
					++j_idx;
				}
				j_idx = 0;
				++i_idx;
			}

			m_max_occupancy = max_occupancy_;

			normalize();
		}
		void compute_occupancy_density(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			std::cout << "Computing density...\n";

			double max_occupancy_ = -1.0 * std::numeric_limits<double>::max();

			for (auto it = cpcrCloud->begin(); it != cpcrCloud->end(); ++it)
			{
				size_t x = compute_voxel_index(m_origin[0], (*it).x, m_leaf_size[0]);
				size_t y = compute_voxel_index(m_origin[1], (*it).y, m_leaf_size[1]);
				size_t z = compute_voxel_index(m_origin[2], (*it).z, m_leaf_size[2]);

				m_grid_array[z][y][x] += 1.0;

				max_occupancy_ = (m_grid_array[z][y][x] > max_occupancy_) ?
							m_grid_array[z][y][x] : max_occupancy_;
			}

			m_max_occupancy = max_occupancy_;

			//normalize();
		}
		void compute_occupancy_surface(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud)
		{
			double max_occupancy_ = -1.0 * std::numeric_limits<double>::max();

			std::vector<pcl::Vertices> triangles_;

			triangulate_cloud(cpcrCloud, triangles_);

			double x_o_ = m_origin[0];
			double y_o_ = m_origin[1];
			double z_o_ = m_origin[2];

			for (grid_array_index z = 0; z < m_grid_side; ++z)
			{
				for (grid_array_index y = 0; y < m_grid_side; ++y)
				{
					for (grid_array_index x = 0; x < m_grid_side; ++x)
					{
						double x_min_ = x_o_ + (x * m_leaf_size[0]);
						double y_min_ = y_o_ + (y * m_leaf_size[1]);
						double z_min_ = z_o_ + (z * m_leaf_size[2]);
						double x_max_ = x_min_ + m_leaf_size[0];
						double y_max_ = y_min_ + m_leaf_size[1];
						double z_max_ = z_min_ + m_leaf_size[2];

						double area_ = 0.0;

						for (auto it = triangles_.begin(); it != triangles_.end(); ++it)
							area_ += clip_triangle(cpcrCloud, *it,
																			x_min_, x_max_,
																			y_min_, y_max_,
																			z_min_, z_max_);

						m_grid_array[z][y][x] = area_;

						max_occupancy_ = (area_ > max_occupancy_) ? area_ : max_occupancy_;
					}
				}
			}

			m_max_occupancy = max_occupancy_;
		}

		void normalize()
		{
			for (grid_array_index z = 0; z < m_grid_side; ++z)
				for (grid_array_index y = 0; y < m_grid_side; ++y)
					for (grid_array_index x = 0; x < m_grid_side; ++x)
						m_grid_array[z][y][x] /= m_max_occupancy;
		}

		bool save(const std::string & crFileName)
		{
			std::ofstream file_(crFileName);

			if(!file_.is_open())
			{
				pcl::console::print_error("Error opening file %s...\n",
																	crFileName.c_str());
				return false;
			}

			file_ << m_origin[0] << " " << m_origin[1] << " " << m_origin[2] << "\n";
			file_ << m_leaf_size[0] << " " << m_leaf_size[1] << " " << m_leaf_size[2] << "\n";
		  file_ << m_grid_size[0] << " " << m_grid_size[1] << " " << m_grid_size[2] << "\n";
			file_ << m_grid_side << " " << m_grid_side << " " << m_grid_side << "\n";

			for (grid_array_index z = 0; z < m_grid_side; ++z)
			{
				for (grid_array_index y = 0; y < m_grid_side; ++y)
				{
					for (grid_array_index x = 0; x < m_grid_side; ++x)
					{
						char sep = (x == m_grid_side-1) ? '\n' : ' ';
						file_ << m_grid_array[z][y][x] << sep;
					}
				}
			}

			file_.close();

			return true;
		}

		bool load(const std::string & crFileName)
		{
			std::ifstream file_(crFileName);

			if(!file_.is_open())
			{
				pcl::console::print_error("Error opening file %s...\n",
																		crFileName.c_str());
				return false;
			}

			file_ >> m_origin[0];
			file_ >> m_origin[1];
			file_ >> m_origin[2];

			file_ >> m_leaf_size[0];
			file_ >> m_leaf_size[1];
			file_ >> m_leaf_size[2];

			file_ >> m_grid_size[0];
			file_ >> m_grid_size[1];
			file_ >> m_grid_size[2];

			file_ >> m_grid_side;
			file_ >> m_grid_side;
			file_ >> m_grid_side;

			fill_grid(0.0);

			for (grid_array_index z = 0; z < m_grid_side; ++z)
			{
				for (grid_array_index y = 0; y < m_grid_side; ++y)
				{
					for (grid_array_index x = 0; x < m_grid_side; ++x)
					{
						file_ >> m_grid_array[z][y][x];
					}
				}
			}

			file_.close();

			return true;
		}

		inline grid_array_type::element* begin()
		{
			return m_grid_array.data();
		}
		inline grid_array_type::element* end()
		{
			return m_grid_array.data() + m_grid_array.num_elements();
		}

		inline double get(
				const unsigned int & crX,
				const unsigned int & crY,
				const unsigned int & crZ) const
		{
			// boost::multi_array already provides asserts for range checking
			return m_grid_array[crZ][crY][crX];
		}

		inline void set(
				const unsigned int &crX,
				const unsigned int &crY,
				const unsigned int &crZ,
				const double & crValue)
		{
			// boost::multi_array already provides asserts for range checking
			m_grid_array[crZ][crY][crX] = crValue;
		}

		inline double get_cell(const int & crX, const int & crY, const int & crZ)
		{
			assert(crX >= 0 && crX < m_grid_side);
			assert(crY >= 0 && crY < m_grid_side);
			assert(crZ >= 0 && crZ < m_grid_side);
			return m_grid_array[crZ][crY][crX];
		}
		inline float get_leaf_size(const int & crDim)
		{
			assert(crDim >= 0 && crDim < 3);
			return m_leaf_size[crDim];
		}
		inline float get_grid_size(const int & crDim)
		{
			assert(crDim >= 0 && crDim < 3);
			return m_grid_size[crDim];
		}
		inline float get_origin(const int & crDim)
		{
			assert(crDim >= 0 && crDim < 3);
			return m_origin[crDim];
		}
		inline int get_side()
		{
			return m_grid_side;
		}

	protected:
		
		grid_array_type m_grid_array;
		Eigen::Vector3f m_leaf_size;
		Eigen::Vector3f m_grid_size;
		Eigen::Vector3f m_origin;
		int m_grid_side;
		double m_max_occupancy;
		int m_padding;

		void fill_grid (const double & crFillValue)
		{
			m_grid_array.resize(boost::extents[m_grid_side][m_grid_side][m_grid_side]);
			std::fill(m_grid_array.origin(), 
								m_grid_array.origin() + m_grid_array.num_elements(),
								crFillValue);
		}

	private:

		template <typename T>
		T clip(const T& n, const T& lower, const T& upper) const
		{
			return std::max(lower, std::min(n, upper));
		}

		const int compute_voxel_index (
			const float & crOrigin,
			const float & crPosition,
			const float & crVoxelSize) const
		{
			return (int)clip((crPosition - crOrigin)/crVoxelSize, 0.0f, (float)(m_grid_side-1));
		}

		void triangulate_cloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud,
														std::vector<pcl::Vertices> & rTriangles) const
		{
			// Estimate normals using a search tree
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
			pcl::PointCloud<pcl::Normal>::Ptr normals_(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZ>);
			tree_->setInputCloud(cpcrCloud);
			ne_.setInputCloud(cpcrCloud);
			ne_.setSearchMethod(tree_);
			ne_.setKSearch(20);
			ne_.compute(*normals_);

			// Concatenate points and normals
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_ (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields (*cpcrCloud, *normals_, *cloud_with_normals_);

			// Create search tree for that cloud
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2_ (new pcl::search::KdTree<pcl::PointNormal>);
			tree2_->setInputCloud(cloud_with_normals_);

			// Triangulate point cloud
			pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3_;
			pcl::PolygonMesh triangles_;

			gp3_.setSearchRadius(2);
			gp3_.setMu(2.5);
			gp3_.setMaximumNearestNeighbors(200);
			gp3_.setMaximumSurfaceAngle(M_PI/4);
			gp3_.setMinimumAngle(M_PI/18);
			gp3_.setMaximumAngle(2*M_PI/3);
			gp3_.setNormalConsistency(false);

			gp3_.setInputCloud(cloud_with_normals_);
			gp3_.setSearchMethod(tree2_);
			gp3_.reconstruct(triangles_);

			//std::cout << "Triangulated " << triangles_.polygons.size() << " faces...\n";
			//std::cout << "The mesh has " << (triangles_.cloud.height * triangles_.cloud.width) << " vertices...\n";

			rTriangles.clear();
			triangles_.polygons.swap(rTriangles);

			//std::cout << "Swapped " << rTriangles.size() << " faces...\n";
			pcl::io::savePLYFile("test_mesh.ply", triangles_, 5);
		}

		const float clip_triangle (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cpcrCloud,
													const pcl::Vertices & crTriangle,
													const float & crMinX,
													const float & crMaxX,
													const float & crMinY,
													const float & crMaxY,
													const float & crMinZ,
													const float & crMaxZ) const
		{
			float area_ = 0.0f;

			pcl::PointXYZ pi,pj,pn;
			pcl::PointCloud<pcl::PointXYZ>::Ptr polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
			
			if(crTriangle.vertices.size() > 1)
			{
				for( int i=0; i<crTriangle.vertices.size(); i++)
				{
					polygon->points.push_back(cpcrCloud->points[crTriangle.vertices[i]]);
				}

				// CLIP LEFT PLANE
				// clip all the edges against all the planes
				int num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					//case 1: point i and j are outside of clipping area. then no point is written.
					if( pi.x < crMinX && pj.x < crMinX )
					{	
					}
					//case 2: points i and j are inside
					else if( pi.x >= crMinX && pj.x >= crMinX )
					{
						clipped_polygon->points.push_back(pi);
					}
					//Case 3: point "i" is is inside, i+1 is outside.
					else if( pi.x >= crMinX && pj.x < crMinX )
					{
						clipped_polygon->points.push_back(pi);
						pn.x = crMinX;
		        float t = (pn.x-pi.x)/(pj.x-pi.x);
		        pn.y = pi.y + t*(pj.y-pi.y);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.x < crMinX && pj.x >= crMinX )
					{
						pn.x = crMinX;
		        float t = (pn.x-pi.x)/(pj.x-pi.x);
		        pn.y = pi.y + t*(pj.y-pi.y);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
				}

				if( clipped_polygon->size() == 0 )
					return 0.0f;

				polygon = clipped_polygon;
				clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

				// CLIP RIGHT PLANE
			  num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					//case 1: point i and j are outside of clipping area. then no point is written.
					if( pi.x > crMaxX && pj.x > crMaxX )
					{	
					}
					//case 2: points i and j are inside
					else if( pi.x <= crMaxX && pj.x <= crMaxX )
					{
						clipped_polygon->points.push_back(pi);
					}
					//Case 3: point "i" is is inside, i+1 is outside.
					else if( pi.x <= crMaxX && pj.x > crMaxX )
					{
						clipped_polygon->points.push_back(pi);
						pn.x = crMaxX;
		        float t = (pn.x-pi.x)/(pj.x-pi.x);
		        pn.y = pi.y + t*(pj.y-pi.y);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.x > crMaxX && pj.x <= crMaxX )
					{
						pn.x = crMaxX;
		        float t = (pn.x-pi.x)/(pj.x-pi.x);
		        pn.y = pi.y + t*(pj.y-pi.y);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
				}

				if( clipped_polygon->size() == 0 )
					return 0.0f;

				polygon = clipped_polygon;
				clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

				// CLIP TOP PLANE
				num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					if( pi.y > crMaxY && pj.y > crMaxY ){}
					else if( pi.y <= crMaxY && pj.y <= crMaxY )
					{
						clipped_polygon->points.push_back(pi);
					}
					else if( pi.y <= crMaxY && pj.y > crMaxY )
					{
						clipped_polygon->points.push_back(pi);
						pn.y = crMaxY;
		        float t = (pn.y-pi.y)/(pj.y-pi.y);
		        pn.x = pi.x + t*(pj.x-pi.x);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.y > crMaxY && pj.y <= crMaxY )
					{
						pn.y = crMaxY;
		        float t = (pn.y-pi.y)/(pj.y-pi.y);
		        pn.x = pi.x + t*(pj.x-pi.x);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
				}

				if( clipped_polygon->size() == 0 )
					return 0.0f;

				polygon = clipped_polygon;
				clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

				// CLIP BOTTOM PLANE
				num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					if( pi.y < crMinY && pj.y < crMinY ){}
					else if( pi.y >= crMinY && pj.y >= crMinY )
					{
						clipped_polygon->points.push_back(pi);
					}
					else if( pi.y >= crMinY && pj.y < crMinY )
					{
						clipped_polygon->points.push_back(pi);
						pn.y = crMinY;
		        float t = (pn.y-pi.y)/(pj.y-pi.y);
		        pn.x = pi.x + t*(pj.x-pi.x);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.y < crMinY && pj.y >= crMinY )
					{
						pn.y = crMinY;
		        float t = (pn.y-pi.y)/(pj.y-pi.y);
		        pn.x = pi.x + t*(pj.x-pi.x);
		        pn.z = pi.z + t*(pj.z-pi.z);
						clipped_polygon->points.push_back(pn);
					}
				}

				if( clipped_polygon->size() == 0 )
					return 0.0f;

				polygon = clipped_polygon;
				clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

				// CLIP NEAR PLANE
				num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					if( pi.z < crMinZ && pj.z < crMinZ ){}
					else if( pi.z >= crMinZ && pj.z >= crMinZ )
					{
						clipped_polygon->points.push_back(pi);
					}
					else if( pi.z >= crMinZ && pj.z < crMinZ )
					{
						clipped_polygon->points.push_back(pi);
						pn.z = crMinZ;
		        float t = (pn.z-pi.z)/(pj.z-pi.z);
		        pn.y = pi.y + t*(pj.y-pi.y);
						pn.x = pi.x + t*(pj.x-pi.x);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.z < crMinZ && pj.z >= crMinZ )
					{
						pn.z = crMinZ;
		        float t = (pn.z-pi.z)/(pj.z-pi.z);
		        pn.y = pi.y + t*(pj.y-pi.y);
						pn.x = pi.x + t*(pj.x-pi.x);
						clipped_polygon->points.push_back(pn);
					}
				}

				if( clipped_polygon->size() == 0 )
					return 0.0f;

				polygon = clipped_polygon;
				clipped_polygon = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

				// CLIP FAR PLANE
				num_points = polygon->size();
				for( int i=0; i<polygon->size(); i++)
				{
					int j = (i + 1) % num_points;
					pi = polygon->points[i];
					pj = polygon->points[j];

					if( pi.z > crMaxZ && pj.z > crMaxZ ){}
					else if( pi.z <= crMaxZ && pj.z <= crMaxZ )
					{
						clipped_polygon->points.push_back(pi);
					}
					else if( pi.z <= crMaxZ && pj.z > crMaxZ )
					{
						clipped_polygon->points.push_back(pi);
						pn.z = crMaxZ;
		        float t = (pn.z-pi.z)/(pj.z-pi.z);
		        pn.y = pi.y + t*(pj.y-pi.y);
						pn.x = pi.x + t*(pj.x-pi.x);
						clipped_polygon->points.push_back(pn);
					}
					//Case 4: "i" is is outside, i+1 is inside
					else if( pi.z > crMaxZ && pj.z <= crMaxZ )
					{
						pn.z = crMaxZ;
		        float t = (pn.z-pi.z)/(pj.z-pi.z);
		        pn.y = pi.y + t*(pj.y-pi.y);
						pn.x = pi.x + t*(pj.x-pi.x);
						clipped_polygon->points.push_back(pn);
					}
				}
			}

			// intersected crTriangle
			if( clipped_polygon->size() > 2 )
			{
				area_ = calculate_polygon_area(*clipped_polygon);
			}

			return area_;
		}

		const float calculate_polygon_area (const pcl::PointCloud<pcl::PointXYZ> & crPolygon) const
		{
			float area_ = 0.0f;
			int num_vertices_ = crPolygon.size();			

			Eigen::Vector3f va_, vb_, res_;
			res_(0) = res_(1) = res_(2) = 0.0f;

			for (size_t i = 0; i < num_vertices_; ++i)
			{
				size_t j = (i + 1) % num_vertices_;

				va_ 	= crPolygon[i].getVector3fMap();
				vb_ 	= crPolygon[j].getVector3fMap();

				res_ += va_.cross(vb_);
			}

			area_ = res_.norm() * 0.5f;

			return area_;
		}
};

#endif
