#ifndef OFF_HANDLER_HPP_
#define OFF_HANDLER_HPP_

#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class OFFHandler
{
	public:

		OFFHandler() {}

		int read (
				const std::string & cFile,
				pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
				std::vector<pcl::Vertices> & rVertices)
		{
			size_t vertex_count_;
			size_t faces_count_;
			pCloud->clear();

			std::ifstream infile_(cFile);
			std::string line_;

			// Check file status
			#ifdef DEBUG_INFORMATION
			std::cout << "Reading " << cFile << "...\n";
			#endif

			if (!infile_.is_open())
			{
				std::cerr << "Error opening file " << cFile << "...\n";
				return -1;
			}

			// Read header
			// OFF
			// [int]number_of_vertices [int]number_of_faces [int]number_of_edges
			{
				int num_vertices_, num_faces_, num_edges_;
				// Discard OFF
				std::getline(infile_, line_);
				// Read line with vertices, faces, and edges information
				std::getline(infile_, line_);
				std::istringstream iss_(line_);
				// Split and store number of vertices, faces, and edges
				if (!(iss_ >> vertex_count_ >> faces_count_ >> num_edges_))
				{
					std::cerr << "Error [" + cFile + "]: Invalid OFF file format in header...\n";
					return -1;
				}

				#ifdef DEBUG_INFORMATION
				std::cout << "Number of vertices: " << vertex_count_ << "...\n";
				std::cout << "Number of faces: " << faces_count_ << "...\n";
				std::cout << "Number of edges: " << num_edges_ << "...\n";
				#endif

				pCloud->reserve(vertex_count_);
			}

			// Read vertices
			// [float]x [float]y [float]z
			size_t i = 0;
			while (i < vertex_count_ && std::getline(infile_, line_))
			{
				float x_, y_, z_;
				std::istringstream iss_(line_);
				if (!(iss_ >> x_ >> y_ >> z_))
				{
					std::cerr << "Error [" + cFile + "]: Invalid OFF file format in vertex " << i << "...\n";
					return -1;
				}

				pcl::PointXYZ point_(x_, y_, z_);
				pCloud->push_back(point_);
				++i;
			}

			#ifdef DEBUG_INFORMATION
			std::cout << pCloud->points.size() << " vertices read as XYZ points...\n";
			std::cout << cFile << " loaded into point cloud!\n";
			#endif

			// Read faces
			// [int]n [int]index_1 [int]index_2 ... [int]index_n
			i = 0;
			while (i < faces_count_ && std::getline(infile_, line_))
			{
				pcl::Vertices vertices_;

				size_t num_vertices_;
				std::istringstream iss_(line_);
				if (!(iss_ >> num_vertices_))
				{
					std::cerr << "Error [" + cFile + "]: Invalid OFF file format in face " << i << "...\n";
					return -1;
				}

				size_t j = 0;
				while (j < num_vertices_)
				{
					uint32_t vertex_index_;
					if (!(iss_ >> vertex_index_))
					{
						std::cerr << "Error [" + cFile + "]: Invalid OFF file format in face " << i << " vertex " << j << " ...\n";
						return -1;
					}

					vertices_.vertices.push_back(vertex_index_);
					++j;
				}

				rVertices.push_back(vertices_);
			}

			#ifdef DEBUG_INFORMATION
			std::cout << rVertices.size() << " faces read...\n";
			std::cout << cFile << " loaded into faces vector!\n";
			#endif

			// Cleanup
			infile_.close();

			return 0;
		}

	private:
};

#endif
