#include "Camera.h"

#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>

using namespace std;

bool Camera::readImageData(const string &path)
{
	string fullPath = path + file_name;
	
	vtkSmartPointer<vtkImageReader2Factory> readerFactory = vtkSmartPointer<vtkImageReader2Factory>::New();
	vtkSmartPointer<vtkImageReader2> imageReader = readerFactory->CreateImageReader2(fullPath.c_str());

	cout << fullPath.c_str() << endl;
	if (!imageReader)
	{
		std::cerr << "..Can't read file \n";
		return false;
	}
	imageReader->SetFileName(fullPath.c_str());
	imageReader->Update();

	image_data = imageReader->GetOutput();

	int *dims = image_data->GetDimensions();
	if (dims[0] != sensor->resolution_width || dims[1] != sensor->resolution_height)
	{
		std::cerr << "..Invalid size of image \n";
		return false;
	}
}

void Camera::getPixelColor(const Eigen::Vector2i &pos, vtkColor3ub &color)
{
	int *dims = image_data->GetDimensions();
	uint8_t *ptr = static_cast<uint8_t *>(image_data->GetScalarPointer(pos[0], dims[1] - pos[1] - 1, 0));
	color[0] = ptr[0];
	color[1] = ptr[1];
	color[2] = ptr[2];
}

bool Camera::getPixelByWorldPoint(const Eigen::Vector3d &pos, vtkColor3ub &color, float &distance)
{
	// convert points from world to camera space by multiplying points in world space and the inverse of the camera-to-world matrix
	Eigen::Vector3d camPos = (transformation.inverse() * pos.homogeneous()).hnormalized();

	// point on back side of camera
	if (camPos[2] <= 0)		
		return false;

	Eigen::Vector2i sensorPos = sensor->projectPoint(camPos).cast<int>();
	
	// std::cout << camPos[0] << " " << camPos[1] << " " << camPos[2] << " ---- " << sensorPos[0] << " " << sensorPos[1] << " ---- " << camPos.norm() << "\n";

	int *dims = image_data->GetDimensions();

	if (sensorPos[0] >= 0 && sensorPos[0] < dims[0] && sensorPos[1] >= 0 && sensorPos[1] < dims[1])	
	{		
		distance = camPos.norm();
		getPixelColor(sensorPos, color);

		return true;
	}
	
	return false;
}