#pragma once

#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Core/Mesh/Handles.hh>

#include <array>

class AABB;

namespace Algorithms
{
// check if 2 AABB are overlapping
// Paramater _box1 the first AABB
// Paramater _box2 the second AABB
// Returns true if the boxes are overlapping (including just touching)
bool checkAxisAlligndBoudnigBoxesIntersections(const AABB& _box1, const AABB& _box2);

// project point into line
// Parameter _linePoint1: a point on the line
// Parameter _linePoint2: another point on the line
// Parameter _point: the point to project into the line
// Returns: the point after projection into the line
OpenMesh::Vec3f projectPointIntoLine(const OpenMesh::Vec3f& _linePoint1,
                                     const OpenMesh::Vec3f& _linePoint2,
                                     const OpenMesh::Vec3f& _point);

// Map 3D triangle points to 2D triangle points
// the mapping will keep the distance and the shape of the original triangle
// Parameter __trianglePoints: the 3D point of the input triangle
// Returns: the 2D mapping of the _trianglePoints
std::array<OpenMesh::Vec2f, 3> map3DTraingleTo2DTriangle(const std::array<OpenMesh::Vec3f, 3>& _trianglePoints);

// compute the barycentric coordinates of a point in reference to a triangle points
// Parameter _point1: triangle point 1
// Parameter _point2: triangle point 2
// Parameter _point3: triangle point 3
// Parameter _point: input point
// Returns: the point barycentric coordinates
OpenMesh::Vec3f barycentric(const OpenMesh::Vec3f& _point1, const OpenMesh::Vec3f& _point2, const OpenMesh::Vec3f& _point3, const OpenMesh::Vec3f& _point);

// Parameter _trianglePoints: triangle points
OpenMesh::Vec3f barycentric(const std::array<OpenMesh::Vec3f, 3>& _trianglePoints, const OpenMesh::Vec3f& _point);

// compute the area of a triangle
// Parameter _triangle: the triangle points
// Returns: the triangle area
float computeTriangleArea(const std::array<OpenMesh::Vec3f, 3>& _triangle);

// a functor to compare 3D points based on their coordinates
struct Compare3DPoints
{
    // this implements the operator < for 3D points
    // comparison is based on x then y the z coordinate of the points
    bool operator()(const OpenMesh::Vec3f& _point1, const OpenMesh::Vec3f& _point2);
};

// a functor to compare Handles
struct CompareHandles
{
    // this implements the operator < for Handles
    bool operator()(const OpenMesh::BaseHandle& _handle1, const OpenMesh::BaseHandle& _handle2);
};

struct CompareTriangle
{
    // this implements the operator < for triangles
    bool operator()(const std::array<OpenMesh::Vec3f, 3>& _triangle1,
                    const std::array<OpenMesh::Vec3f, 3>& _triangle2);
};

}
