#include "DebugUtils.h"
#include "IO/MeshIO.h"
#include "Mesh/TriMesh.h"

void DebugUtils::writeTrianglesFile(const std::vector<std::array<OpenMesh::Vec3f, 3> > &_trianglesPoints, std::__cxx11::string _fileName)
{
    TriMesh mesh;
    for (const auto& triangle : _trianglesPoints)
    {
        auto vh0 = mesh.add_vertex(triangle[0]);
        auto vh1 = mesh.add_vertex(triangle[1]);
        auto vh2 = mesh.add_vertex(triangle[2]);
        mesh.add_face(vh0, vh1, vh2);
    }
    MeshIO::writeMesh(_fileName, mesh);
}

void DebugUtils::writeTrianglesFile(const std::vector<std::array<OpenMesh::Vec2f, 3>> &_trianglesPoints, std::__cxx11::string _fileName)
{
    TriMesh mesh;
    for (const auto& triangle : _trianglesPoints)
    {
        auto vh0 = mesh.add_vertex(OpenMesh::Vec3f(triangle[0][0], triangle[0][1], 0.0f));
        auto vh1 = mesh.add_vertex(OpenMesh::Vec3f(triangle[1][0], triangle[1][1], 0.0f));
        auto vh2 = mesh.add_vertex(OpenMesh::Vec3f(triangle[2][0], triangle[2][1], 0.0f));
        mesh.add_face(vh0, vh1, vh2);
    }
    MeshIO::writeMesh(_fileName, mesh);
}
