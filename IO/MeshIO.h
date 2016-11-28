#pragma once

#include <string>

// IO manager for TriMeshes

class TriMesh;

class MeshIO
{
public:

    // read a mesh from file
    // Parameter _fileName: the mesh file name
    // Parameter _mesh: the output mesh
    // Returns: true on success and false on failure
    static bool readMesh(std::string _fileName, TriMesh& _mesh);

    // write a mesh to a file
    // Parameter _fileName: the mesh file name
    // Parameter _mesh: the input mesh
    // Returns: true on success and false on failure
    static bool writeMesh(std::string _fileName, TriMesh& _mesh);
};
