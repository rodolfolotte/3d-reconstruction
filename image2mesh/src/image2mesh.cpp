#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkSmartPointer.h>

#include <embree2/rtcore.h>

#include <iostream>
#include <deque>
#include <string>
#include <set>
#include <fstream>
#include <chrono>

#include "ImageBlock.h"
#include "PhotoscanXMLReader.h"
#include "ProjectionMeshLabelling.h"
#include "MeshUtils.h"

using namespace std;

// EXAMPLE: ./image2mesh -camera /home/rodolfo/Dropbox/phd/data/facades-benchmark/ruemonge2014/PhotoScan/camera.xml
// 						  -image_folder /home/rodolfo/Dropbox/phd/results/facades-benchmark/ruemonge2014/facadeSeg/segmentation/segmentation-100000
// 						  -mesh /home/rodolfo/Dropbox/phd/data/facades-benchmark/ruemonge2014/mesh/ruemonge2014-mesh.ply
//						  -out /home/rodolfo/Dropbox/phd/results/facades-benchmark/ruemonge2014/facadeSeg/mesh/label-10000-mesh-projection.ply

// ./image2mesh -camera /home/rodolfo/Dropbox/phd/data/facades-benchmark/ruemonge2014/PhotoScan/camera.xml -image_folder /home/rodolfo/Dropbox/phd/results/facades-benchmark/ruemonge2014/facadeSeg/segmentation/segmentation-100000 -mesh /home/rodolfo/Dropbox/phd/data/facades-benchmark/ruemonge2014/mesh/ruemonge2014-mesh.ply -out /home/rodolfo/Dropbox/phd/results/facades-benchmark/ruemonge2014/facadeSeg/mesh/label-10000-mesh-projection.ply
int main(int argc, char **argv)
{
	const char *camerasFile = NULL;
	const char *imagesDirectory = NULL;
	const char *inputMeshFile = NULL;
	const char *outputMeshFile = NULL;

	bool help = false;
	bool askhelp = false;

	for (int i = 1; i < argc; i++)
	{
		if (argv[i][0] == '-')
		{
			if (strcmp(argv[i], "-help") == 0)
			{
				askhelp = true;
				break;
			}
			if (strcmp(argv[i], "-camera") == 0)
			{
				camerasFile = argv[i + 1];
				i++;
				continue;
			}
			if (strcmp(argv[i], "-image_folder") == 0)
			{
				imagesDirectory = argv[i + 1];
				i++;
				continue;
			}
			if (strcmp(argv[i], "-mesh") == 0)
			{
				inputMeshFile = argv[i + 1];
				i++;
				continue;
			}
			if (strcmp(argv[i], "-out") == 0)
			{
				outputMeshFile = argv[i + 1];
				i++;
				continue;
			}

			cout << "Invalid " << argv[i] << " option.\n" << endl;
			help = true;
		}
	}

	if (!askhelp)
	{
		if (!camerasFile)
		{
			cout << "\nERROR: No camera projection file specified." << endl;
			help = true;
		}
		if (!imagesDirectory)
		{
			cout << "\nERROR: No image folder specified." << endl;
			help = true;
		}
		if (!inputMeshFile)
		{
			cout << "\nERROR: No mesh file specified." << endl;
			help = true;
		}
		if (!outputMeshFile)
		{
			cout << "\nERROR: No output file specified." << endl;
			help = true;
		}
	}

	if (help || askhelp)
	{
		cout << "\nUsage: ./image2mesh -camera XML_FILE -image_folder IMAGES -mesh PLY_FILE -out NEW_PLY_FILE\n"
				"    [-camera ]\n"
				"    [-image_folder ]\n"
				"    [-mesh ]\n"
				"    [-out ]\n"
			 << endl;
		exit(1);
	}

	auto startTime{std::chrono::steady_clock::now()};

	/* create new Embree device */
	RTCDevice rtcDevice = rtcNewDevice(NULL);
	if (rtcDevice == NULL) {
		std::cerr << "Embree rtcNewDevice error: " << rtcDeviceGetError(rtcDevice) << std::endl;
	}

	ColorMap cm;
	cm[vtkColor3ub(0, 0, 0)] = UNKNOWN_CLASS;
	//cm[vtkColor3ub(128, 255, 255)] = 1; // sky
	cm[vtkColor3ub(0, 255, 255)] = 1; // sky
	cm[vtkColor3ub(0, 0, 255)] = 2; // roof
	cm[vtkColor3ub(255, 255, 0)] = 3; //wall
	cm[vtkColor3ub(255, 0, 0)] = 4; // window
	cm[vtkColor3ub(255, 128, 0)] = 5; // door
	cm[vtkColor3ub(0, 255, 0)] = 6; // shop
	//cm[vtkColor3ub(128, 0, 255)] = 7; // balcony
	cm[vtkColor3ub(255, 0, 255)] = 7; // balcony

	try {

		std::cout << "reading image block...";
		ImageBlock block;
		PhotoscanXMLReader photoscanReader;
		photoscanReader.readImageBlock(camerasFile, block);
		std::cout << "OK" << std::endl;

		std::cout << "reading mesh...";
		vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
		reader->SetFileName(inputMeshFile);
		vtkSmartPointer<vtkPolyData> inMesh = reader->GetOutput();
		reader->Update();
		std::cout << "OK" << std::endl;

		std::cout << "reading images..." << std::endl;
		block.readAllImages(imagesDirectory);
		std::cout << "OK" << std::endl;

		ProjectionMeshLabelling labelling;
		std::vector<int> faceClasses;
		labelling.setMesh(inMesh);
		labelling.setImageBlock(block);
		labelling.setColorMap(cm);
		labelling.setRTCDevice(rtcDevice);
		labelling.setResultClassesVector(faceClasses);
		
		std::cout << "starting ray-tracing from images to the mesh...";
		labelling.labelMesh();
		std::cout << "OK" << std::endl;

		std::cout << "building new mesh with colors representing labels...";
		auto outMesh = buildColoredMesh(inMesh, faceClasses, cm);
		std::cout << "OK" << std::endl;

		std::cout << "Writing new mesh to file " << outputMeshFile << "...";

		auto writer{ vtkSmartPointer<vtkPLYWriter>::New() };
		writer->SetFileName(outputMeshFile);
		writer->SetInputData(outMesh);
		writer->SetArrayName("RGB");
		writer->Update();
		std::cout << "OK" << std::endl;
	}
	catch (const std::ios_base::failure &e) {
		std::cerr << e.what() << e.code().message();
	}
	catch (const std::exception &e) {
		std::cerr << e.what();
	}

	auto finishTime{ std::chrono::steady_clock::now() };
	std::chrono::duration<float> elapsedTime = finishTime - startTime;

	std::cout << "Time: " << elapsedTime.count() << std::endl;

	rtcDeleteDevice(rtcDevice);

    return 0;
}

