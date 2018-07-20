#pragma once

#include "ImageBlock.h"

#include <vector>
#include <deque>
#include <map>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <embree2/rtcore.h>

class ProjectionMeshLabelling
{
public:
	void setMesh(vtkSmartPointer<vtkPolyData> &mesh_)
	{
		mesh = mesh_;
	}

	void setImageBlock(ImageBlock &imageBlock)
	{
		imgBlock = &imageBlock;
	}

	void setColorMap(const ColorMap &map)
	{
		colorMap = &map;
	}

	void setResultClassesVector(std::vector<int> &classes_)
	{
		classes = &classes_;
	}

	void setRTCDevice(RTCDevice rtcDevice_)
	{
		rtcDevice = rtcDevice_;
	}

	void labelMesh();

private:
	vtkSmartPointer<vtkPolyData> mesh;
	ImageBlock *imgBlock = nullptr;
	const ColorMap *colorMap = nullptr;
	std::vector<int> *classes = nullptr;
	RTCDevice rtcDevice = nullptr;

	struct VertexClassCandidate {
		VertexClassCandidate() = default;
		VertexClassCandidate(float distance, int classNum) :
			distance(distance), classNum(classNum)
		{}

		float distance;
		int classNum;

		bool operator<(const VertexClassCandidate &other)
		{
			return distance < other.distance;
		}
	};

	std::vector< std::deque<VertexClassCandidate> > vertexClassCandidates;

	void computeVertexClassCandidates();

	void limirize(vtkColor3ub &color);

	void generalize(vtkColor3ub &color);
};

