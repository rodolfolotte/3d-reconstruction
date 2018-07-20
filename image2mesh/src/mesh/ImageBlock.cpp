#include "ImageBlock.h"

void ImageBlock::readAllImages(const std::string &path)
{	
	for (auto &c : cameras) {
		c.second.readImageData(path);
	}
}