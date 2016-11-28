#include "MeshIO.h"
#include <OpenMesh/Core/IO/MeshIO.hh>

#include "Mesh/TriMesh.h"


bool MeshIO::readMesh(std::string _fileName, TriMesh &_mesh)
{
    return OpenMesh::IO::read_mesh(_mesh, _fileName);
}

bool MeshIO::writeMesh(std::string _fileName, TriMesh &_mesh)
{
    return OpenMesh::IO::write_mesh(_mesh, _fileName);
}
