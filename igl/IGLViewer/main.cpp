#include <igl/readOFF.h>
#include <igl/readMESH.h>
#include <igl/readPLY.h>
#include <igl/readOBJ.h>
#include <igl/viewer/Viewer.h>
#include <igl/jet.h>

#include <string>

Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd C;

using namespace std;

void read(string input_filename){  
  std::cout << log(0) << " Open " << input_filename << " for visualizing..." << std::endl;
  string extension = input_filename.substr(input_filename.find_last_of('.'));

  if (extension == ".off" || extension == ".OFF") {
    igl::readOFF(input_filename.c_str(), V, F);
  } else if (extension == ".mesh" || extension == ".MESH") {
    igl::readMESH(input_filename.c_str(), V, F);
  } else if (extension == ".ply" || extension == ".PLY") {
    igl::readPLY(input_filename.c_str(), V, F);
   } else if (extension == ".obj" || extension == ".OBJ") {
    igl::readOBJ(input_filename.c_str(), V, F);
  }  
} 

int main(int argc, char *argv[]) {
  string input_filename = argv[1];

  read(input_filename);  
  
  // Plot the mesh
  igl::viewer::Viewer viewer;
  viewer.data.set_mesh(V, F);

  // Use the z coordinate as a scalar field over the surface
  //Eigen::VectorXd Z = V.col(2);

  // Compute per-vertex colors
  //igl::jet(Z,true,C);

  // Add per-vertex colors
  //viewer.data.set_colors(C);

  // Launch the viewer
  viewer.launch();
}
