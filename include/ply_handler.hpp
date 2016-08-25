#ifndef PLY_HANDLER_HPP_
#define PLY_HANDLER_HPP_

#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class PLYHandler
{
	public:

		PLYHandler() {}

		void write (
				const pcl::PointCloud<pcl::PointXYZ>::Ptr cpCloud,
				const std::vector<pcl::Vertices> & crFaces,
				const std::string & crFile)
		{
			std::ofstream file_(crFile);

			// Write header
			file_ << "ply\n";
			file_ << "format ascii 1.0\n";
			file_ << "element vertex " << cpCloud->points.size() << "\n";
			file_ << "property float x\n";
			file_ << "property float y\n";
			file_ << "property float z\n";
			file_ << "element face " << crFaces.size() << "\n";
			file_ << "property list uchar int vertex_index\n";
			file_ << "end_header\n";

			// Write vertices
			for (size_t i = 0; i < cpCloud->points.size(); ++i)
			{
				file_ << cpCloud->points[i].x << " ";
				file_ << cpCloud->points[i].y << " ";
			  file_	<< cpCloud->points[i].z << "\n";
			}

			// Write faces
			for (size_t i = 0; i < crFaces.size(); ++i)
			{
				file_ << "3 ";
				file_	<< crFaces[i].vertices[0] << " ";
				file_	<< crFaces[i].vertices[1] << " ";
				file_	<< crFaces[i].vertices[2] << "\n";
			}

			// Cleanup
			file_.close();
		}

	private:
};

#endif
