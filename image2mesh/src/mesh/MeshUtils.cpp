#include "MeshUtils.h"

#include <map>

#include <vtkColor.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellData.h>

vtkSmartPointer<vtkPolyData> buildColoredMesh(vtkPolyData *inMesh, const std::vector<int> &faceClasses, const ColorMap &colorMap)
{
	// reverse color map
	std::map<int, vtkColor3ub> reverseMap;
	for (auto &color : colorMap) {
		reverseMap[color.second] = color.first;
	}

	auto inPoints = inMesh->GetPoints();
	auto inPolys = inMesh->GetPolys();

	vtkSmartPointer<vtkPolyData> outMesh = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkPoints> outPoints = vtkSmartPointer<vtkPoints>::New();
	outPoints->SetDataType(inPoints->GetDataType());
	outPoints->SetNumberOfPoints(inPolys->GetNumberOfCells() * 3);
	outMesh->SetPoints(outPoints);

	vtkSmartPointer<vtkCellArray> outPolys = vtkSmartPointer<vtkCellArray>::New();
	outPolys->Allocate(outPolys->EstimateSize(inPolys->GetNumberOfCells(), 3));
	outMesh->SetPolys(outPolys);

	auto inPointData = inMesh->GetPointData();
	auto outPointData = outMesh->GetPointData();

	outPointData->CopyAllOn();
	outPointData->CopyFieldOff("RGB");
	outPointData->CopyAllocate(inPointData, outPoints->GetNumberOfPoints());

	auto outRGB{ vtkSmartPointer<vtkUnsignedCharArray>::New() };
	outRGB->SetNumberOfComponents(3);
	outRGB->SetNumberOfTuples(outPoints->GetNumberOfPoints());
	outRGB->SetName("RGB");

	outPointData->AddArray(outRGB);

	// compute color for each face
	// duplicate verticies with colors based on their class number
	vtkIdType npts;
	vtkIdType *pts;
	unsigned i = 0;
	unsigned currentNewVertexNum = 0;
	for (inPolys->InitTraversal(); inPolys->GetNextCell(npts, pts); ++i) {
		auto &color = reverseMap[faceClasses[i]];

		if (npts != 3)
			throw std::runtime_error("not a triangle occured");

		outPolys->InsertNextCell(npts);

		for (int j = 0; j < npts; ++j) {
			// copy point
			double point[3];
			inPoints->GetPoint(pts[j], point);
			outPoints->SetPoint(currentNewVertexNum, point);

			// copy point attributes
			outPointData->CopyData(inPointData, pts[j], currentNewVertexNum);

			// set point color
			outRGB->SetTypedTuple(currentNewVertexNum, color.GetData());

			// add this point to polygon
			outPolys->InsertCellPoint(currentNewVertexNum);

			++currentNewVertexNum;
		}
	}

	return outMesh;
}

unsigned rtcTriangleMeshFromVtkPolyData(RTCScene scene, vtkPolyData *mesh)
{
	unsigned rtcmesh = rtcNewTriangleMesh2(scene, RTC_GEOMETRY_STATIC, mesh->GetNumberOfPolys(), mesh->GetNumberOfPoints());

	float *vertices = (float *)rtcMapBuffer(scene, rtcmesh, RTC_VERTEX_BUFFER);
	// vertex stride is 16 bytes (4 floats)
	auto points = mesh->GetPoints();
	for (unsigned i = 0; i < points->GetNumberOfPoints(); ++i)
	{
		double tmp[3];
		points->GetPoint(i, tmp);
		*(vertices++) = tmp[0];
		*(vertices++) = tmp[1];
		*(vertices) = tmp[2];
		vertices += 2;
	}
	rtcUnmapBuffer(scene, rtcmesh, RTC_VERTEX_BUFFER);

	int *indicies = (int *)rtcMapBuffer(scene, rtcmesh, RTC_INDEX_BUFFER);
	auto polys = mesh->GetPolys();
	vtkIdType npts;
	vtkIdType *pts;
	for (polys->InitTraversal(); polys->GetNextCell(npts, pts);)
	{
		if (npts != 3)
		{
			std::cerr << "invalid geometry in poly data (" << npts << ") points\n";
			continue;
		}

		*(indicies++) = pts[0];
		*(indicies++) = pts[1];
		*(indicies++) = pts[2];
	}
	rtcUnmapBuffer(scene, rtcmesh, RTC_INDEX_BUFFER);

	rtcCommit(scene);

	return rtcmesh;
}