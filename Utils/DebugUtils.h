#pragma once

#include <vector>
#include <string>
#include <array>
#include <OpenMesh/Core/Geometry/VectorT.hh>

namespace DebugUtils
{

void writeTrianglesFile(const std::vector<std::array<OpenMesh::Vec3f, 3>>& _trianglesPoints,
                        std::string _fileName);

void writeTrianglesFile(const std::vector<std::array<OpenMesh::Vec2f, 3>>& _trianglesPoints,
                        std::string _fileName);
}
