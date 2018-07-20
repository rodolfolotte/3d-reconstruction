
#include "ProjectionMeshLabelling.h"
#include "MeshUtils.h"

#include <iostream>
#include <algorithm>

#include <vtkModifiedBSPTree.h>
#include <vtkCellTreeLocator.h>

#include <embree2/rtcore_ray.h>

void ProjectionMeshLabelling::labelMesh()
{
	computeVertexClassCandidates();

	classes->resize(mesh->GetNumberOfPolys());
	auto polys = mesh->GetPolys();

	// compute class for each face based on classes assigned to their verticies
	std::size_t i = 0;
	vtkIdType npts;
	vtkIdType *pts;
	for (polys->InitTraversal(); polys->GetNextCell(npts, pts); ++i)
	{
		// class number -> number of occurences
		std::map<int, int> triangleVertexClassesCount;

		// for each vertex
		for (int j = 0; j < npts; ++j)
		{
			auto &vertexClasses = vertexClassCandidates[pts[j]];
			int classNum = UNKNOWN_CLASS;
			if (!vertexClasses.empty())
			{
				// take class from the closest image
				auto minElem = std::min_element(vertexClasses.begin(), vertexClasses.end());
				classNum = minElem->classNum;
			}

			auto itr = triangleVertexClassesCount.find(classNum);
			if (itr == triangleVertexClassesCount.end())
				triangleVertexClassesCount[classNum] = 1;
			else
				++itr->second;
		}

		// CHANGE THE RULE HERE
		// find class that occurs the most
		auto classEntryWithMaxOccurence = std::max_element(triangleVertexClassesCount.begin(), triangleVertexClassesCount.end(),
														   [](const std::pair<int, int> &l, const std::pair<int, int> &r) { return l.second < r.second; });

		(*classes)[i] = classEntryWithMaxOccurence->first;
	}
}

void ProjectionMeshLabelling::limirize(vtkColor3ub &color)
{
	for (int i = 0; i < 3; i++)
	{
		if (color[i] > 0 && color[i] <= 70)
		{
			color[i] = 0;
		}

		if (color[i] > 70 && color[i] <= 140)
		{
			color[i] = 128;
		}

		if (color[i] > 140 && color[i] < 256)
		{
			color[i] = 255;
		}
	}
}

void ProjectionMeshLabelling::generalize(vtkColor3ub &color)
{

	if (color[0] == 0)
	{
		// blue variations
		if (color[1] == 0)
		{
			if (color[2] == 128)
			{
				color[2] = 255;
			}
		}

		// blue variations
		if (color[1] == 128)
		{
			if (color[2] == 255)
			{
				color[1] = 0;
			}

			if (color[2] == 0 || color[2] == 128)
			{
				color[1] = 255;
				color[2] = 0;
			}
		}

		// cyan and green variations
		if (color[1] == 255)
		{
			if (color[2] == 128)
			{
				color[2] = 0;
			}
		}
	}

	if (color[0] == 128)
	{
		// red and magenta variations
		if (color[1] == 0)
		{
			if (color[2] == 0)
			{
				color[0] = 255;
			}

			if (color[2] == 128 || color[2] == 255)
			{
				color[0] = 255;
				color[2] = 255;
			}
		}

		// magenta and yellow variations
		if (color[1] == 128)
		{
			if (color[2] == 0)
			{
				color[0] = 255;
				color[1] = 255;
			}

			if (color[2] == 128)
			{
				color[0] = 0;
				color[1] = 0;
				color[2] = 0;
			}

			if (color[2] == 255)
			{
				color[0] = 255;
				color[1] = 0;
			}
		}

		// green variations
		if (color[1] == 255)
		{
			if (color[2] == 0 || color[2] == 128)
			{
				color[0] = 0;
				color[2] = 0;
			}

			if (color[2] == 255)
			{
				color[0] = 0;
			}
		}
	}

	if (color[0] == 255)
	{
		// red variations
		if (color[1] == 0)
		{
			if (color[2] == 128)
			{
				color[2] = 0;
			}
		}

		// orange variations
		if (color[1] == 128)
		{
			if (color[2] == 255)
			{
				color[1] = 0;
			}

			if (color[2] == 128)
			{
				color[2] = 0;
			}
		}

		// orange variations
		if (color[1] == 255)
		{
			if (color[2] == 128 || color[2] == 255)
			{
				color[2] = 0;
			}
		}
	}
}

void ProjectionMeshLabelling::computeVertexClassCandidates()
{
	std::cout << "..building rtc geometry...";
	auto scene = rtcDeviceNewScene(rtcDevice, RTC_SCENE_STATIC, RTC_INTERSECT1);
	auto geometry = rtcTriangleMeshFromVtkPolyData(scene, mesh);
	std::cout << "OK" << std::endl;

	std::vector<Eigen::Vector3d> cameraCenters;
	cameraCenters.reserve(imgBlock->cameras.size());
	Eigen::Vector4d zero(0, 0, 0, 1);
	for (auto &camera : imgBlock->cameras)
		cameraCenters.push_back((camera.second.transformation * zero).hnormalized());

	auto numPoints = mesh->GetNumberOfPoints();
	auto points = mesh->GetPoints();

	vertexClassCandidates.clear();
	vertexClassCandidates.resize(numPoints);

	// stats
	unsigned numPositiveIntersections{};
	unsigned numNegativeIntersections{};
	unsigned numUnvisiblePoints{};
	unsigned onePercentOfPoints = 1 + (numPoints / 1000);

	for (vtkIdType i = 0; i < numPoints; ++i)
	{
		Eigen::Vector3d point;
		points->GetPoint(i, point.data());

		unsigned j = 0;
		for (auto &camera : imgBlock->cameras)
		{
			vtkColor3ub color;
			float distance;

			if (camera.second.getPixelByWorldPoint(point, color, distance)) // esta falhando aqui
			{
				//if (0 != locator->IntersectWithLine(cameraCenters[j].data(), point.data(), nullptr, nullptr)) {
				Eigen::Vector3d direction = cameraCenters[j] - point;

				RTCRay ray;
				assignToArray(point, ray.org);
				assignToArray(direction, ray.dir);

				ray.tnear = 0.001f;
				ray.tfar = direction.norm();
				ray.instID = RTC_INVALID_GEOMETRY_ID;
				ray.geomID = RTC_INVALID_GEOMETRY_ID;
				ray.primID = RTC_INVALID_GEOMETRY_ID;
				ray.mask = 0xFFFFFFFF;
				ray.time = 0.0f;

				rtcOccluded(scene, ray);
				if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
				{	
					// no intersection / occlusion
					++numPositiveIntersections;

					// channels limiarization
					limirize(color);

					// generalize colors
					generalize(color);

					auto itr = colorMap->find(color);

					if (itr == colorMap->end())
					{
						//vertexClasses[i][distance] = UNKNOWN_CLASS;
						std::cerr << "..Unknown class for color: " << (unsigned)color[0] << " " << (unsigned)color[1] << " " << (unsigned)color[2] << std::endl;
					}
					else
					{
						vertexClassCandidates[i].emplace_back(distance, itr->second);
					}
				}
				else
					++numNegativeIntersections;
			}
			else
				++numUnvisiblePoints;

			++j;
		}

		// // print stats
		// if (((i + 1) % onePercentOfPoints) == 0) {
		// 	std::cout << "points " << i + 1 << " / " << numPoints << "   " << (float)(i + 1) / numPoints * 100 << "%"
		// 		"\npositive intersections: " << numPositiveIntersections <<
		// 		"\nnegative intersections: " << numNegativeIntersections <<
		// 		"\nnum unvisible points:   " << numUnvisiblePoints <<
		// 		"\n\n";
		// }
	}

	rtcDeleteScene(scene);
}