#include "PhotoscanXMLReader.h"

using namespace std;

void PhotoscanXMLReader::readSensor(Sensor &sensor, pugi::xml_node node)
{
	sensor.id = node.attribute("id").as_string();
	sensor.label = node.attribute("label").as_string();

	auto resolution_node = node.child("resolution");
	sensor.resolution_width = resolution_node.attribute("width").as_int();
	sensor.resolution_height = resolution_node.attribute("height").as_int();

	for (auto &property_ : node.children("property")) {
		std::string name = property_.attribute("name").as_string();
		if (name == "pixel_width")
			sensor.pixel_width = property_.attribute("value").as_double();
		else if (name == "pixel_height")
			sensor.pixel_height = property_.attribute("value").as_double();
		if (name == "focal_length")
			sensor.focal_length = property_.attribute("value").as_double();
	}

	auto calibration_node = node.child("calibration");
	auto calibration_resolution_node = calibration_node.child("resolution");
	sensor.calibration_res_width = calibration_resolution_node.attribute("width").as_int();
	sensor.calibration_res_height = calibration_resolution_node.attribute("height").as_int();

	sensor.f = calibration_node.child("f").text().as_double();
	sensor.fx = calibration_node.child("fx").text().as_double();
	sensor.fy = calibration_node.child("fy").text().as_double();
	sensor.cx = calibration_node.child("cx").text().as_double();
	sensor.cy = calibration_node.child("cy").text().as_double();
	sensor.skew = calibration_node.child("skew").text().as_double();
	sensor.k1 = calibration_node.child("k1").text().as_double();
	sensor.k2 = calibration_node.child("k2").text().as_double();
	sensor.k3 = calibration_node.child("k3").text().as_double();
	sensor.k4 = calibration_node.child("k4").text().as_double();
	sensor.p1 = calibration_node.child("p1").text().as_double();
	sensor.p2 = calibration_node.child("p2").text().as_double();
}

void PhotoscanXMLReader::readCameras(ImageBlock &block, pugi::xml_node node)
{
	for (auto &camera_node : node.children("camera")) {
		std::string id = camera_node.attribute("id").as_string();

		Camera &c = block.cameras[id];
		c.id = id;
		c.file_name = camera_node.attribute("label").as_string();
		c.sensor = &block.sensor; //camera_node.attribute("sensor_id").as_string();
		parseMatrix(camera_node.child_value("transform"), c.transformation);
	}
}

template <typename MatrixType>
void PhotoscanXMLReader::parseMatrix(const char *str, MatrixType &matrix)
{
	std::istringstream ss(str);
	for (int i = 0; i < matrix.rows() && ss; ++i) {
		for (int j = 0; j < matrix.cols() && ss; ++j) {
			double tmp;
			ss >> tmp;
			if (ss.fail())
				std::cerr << "Failed to parse following matrix" << std::endl;

			matrix(i, j) = tmp;
		}
	}
}

void PhotoscanXMLReader::readTransform(Transform &t, pugi::xml_node node)
{
	parseMatrix(node.child_value("rotation"), t.rotation);
	parseMatrix(node.child_value("translation"), t.translation);
	t.scale = node.child("scale").text().as_double();
}

void PhotoscanXMLReader::readImageBlock(const std::string &file, ImageBlock &block)
{
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(file.c_str());
	if (!result)
		std::cerr << result.description() << std::endl;

	pugi::xml_node chunk_node = doc.select_node("/document/chunk").node();

	readSensor(block.sensor, chunk_node.child("sensors").child("sensor"));
	readCameras(block, chunk_node.child("cameras"));

	auto transform_node = chunk_node.child("transform");
	if (transform_node) {
		readTransform(block.to_world_transform, transform_node);
		block.is_world_transform = true;
	}
	else {
		block.is_world_transform = false;
	}

	block.reference_wkt = chunk_node.child("reference").text().as_string();
}