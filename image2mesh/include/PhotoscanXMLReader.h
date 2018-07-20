#pragma once

#include <string>
#include <pugixml.hpp>

#include "ImageBlock.h"


class PhotoscanXMLReader
{
public:
	void readImageBlock(const std::string &file, ImageBlock &block);
	
private:
	void readSensor(Sensor &sensor, pugi::xml_node node);
	void readCameras(ImageBlock &block, pugi::xml_node node);
	void readTransform(Transform &t, pugi::xml_node node);

	template <typename MatrixType>
	void parseMatrix(const char *str, MatrixType &matrix);
};

