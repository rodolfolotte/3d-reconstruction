ADD_EXECUTABLE(image2mesh
               camera/Camera.cpp               
               mesh/Color.cpp
               mesh/ImageBlock.cpp
               mesh/MeshUtils.cpp               
               mesh/Color.cpp
               mesh/ProjectionMeshLabelling.cpp
               photoscan/PhotoscanXMLReader.cpp                              
               image2mesh.cpp)

TARGET_LINK_LIBRARIES(image2mesh ${VTK_LIBRARIES} ${PUGIXML_LIBRARY} ${TBB_LIBRARY} ${EMBREE_LIBRARY} ${EIGEN_LIBRARY})
