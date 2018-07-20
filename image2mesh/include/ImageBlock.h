#pragma once

#include "Camera.h"
#include "Sensor.h"

#include <Eigen/Eigen>
#include <string>

struct Transform
{
	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;
	double scale = 1;

	Eigen::Matrix4d transform_matrix;
	void updateTransformMatrix()
	{
		// TODO something is wrong here
		/*transform_matrix = Eigen::Matrix4d::Identity();
		transform_matrix.block(0, 0, 3, 3) = rotation;
		transform_matrix *= scale;
		transform_matrix.col(3) = translation;*/
	}
};

struct ImageBlock
{
	Sensor sensor; // now only one sensor
	std::map<std::string, Camera> cameras;

	bool is_world_transform = false;
	Transform to_world_transform;

	bool is_reference_cs = false;
	std::string reference_wkt;

	void readAllImages(const std::string &path);


};
