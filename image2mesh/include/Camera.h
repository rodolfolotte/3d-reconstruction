#pragma once

#include "Sensor.h"
#include "Color.h"

#include <string>

#include <Eigen/Eigen>

#include <vtkSmartPointer.h>
#include <vtkColor.h>
#include <vtkImageData.h>


struct Camera
{
	std::string id;
	std::string file_name;
	Sensor *sensor = nullptr;

	Eigen::Matrix4d transformation;

	vtkSmartPointer<vtkImageData> image_data;

	bool readImageData(const std::string &path);
	void getPixelColor(const Eigen::Vector2i &pos, vtkColor3ub &color);
	bool getPixelByWorldPoint(const Eigen::Vector3d &pos, vtkColor3ub &color, float &distance);

	inline vtkColor3ub getPixelColor(const Eigen::Vector2i &pos)
	{
		vtkColor3ub c;
		getPixelColor(pos, c);
		return c;
	}

	
};