#pragma once

#include "Color.h"
#include <vector>
#include <Eigen/Eigen>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <embree2/rtcore.h>

vtkSmartPointer<vtkPolyData> buildColoredMesh(vtkPolyData *inMesh, const std::vector<int> &faceClasses, const ColorMap &colorMap);

unsigned rtcTriangleMeshFromVtkPolyData(RTCScene scene, vtkPolyData *mesh);

template<typename Derived, typename ScalarType>
inline static void assignToArray(const Eigen::MatrixBase<Derived> &vec, ScalarType *arr)
{
	EIGEN_STATIC_ASSERT_VECTOR_ONLY(Eigen::MatrixBase<Derived>)

	for (int i = 0; i < Eigen::MatrixBase<Derived>::SizeAtCompileTime; ++i)
		arr[i] = vec[i];
}
