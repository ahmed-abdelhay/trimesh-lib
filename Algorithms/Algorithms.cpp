#include "Algorithms.h"

#include "Types/AABB.h"
#include <OpenMesh/Core/Geometry/MathDefs.hh>

bool Algorithms::checkAxisAlligndBoudnigBoxesIntersections(const AABB &_box1, const AABB &_box2)
{
    for (int i = 0; i < 3; ++i)
    {
        if (_box1.minPoint[i] > _box2.maxPoint[i])
            return false;
        if (_box2.minPoint[i] > _box1.maxPoint[i])
            return false;
    }
    return true;
}

OpenMesh::Vec3f Algorithms::projectPointIntoLine(const OpenMesh::Vec3f &_linePoint1, const OpenMesh::Vec3f &_linePoint2, const OpenMesh::Vec3f &_point)
{
    auto t = (_linePoint2 - _linePoint1).normalized();
    return _linePoint1 + OpenMesh::dot(_point - _linePoint1, t) * t;
}

std::array<OpenMesh::Vec2f, 3> Algorithms::map3DTraingleTo2DTriangle(const std::array<OpenMesh::Vec3f, 3> &_trianglePoints)
{
    std::array<OpenMesh::Vec2f, 3> outputPoints;

    auto ab = _trianglePoints[1] - _trianglePoints[0];
    auto ac = _trianglePoints[2] - _trianglePoints[0];

    outputPoints[0][0] = 0.0f;
    outputPoints[0][1] = 0.0f;

    outputPoints[1][0] = ab.length();
    outputPoints[1][1] = 0.0f;

    auto angle = acos(OpenMesh::dot(ab.normalized(), ac.normalized()));
    outputPoints[2][0] =  std::cos(angle) * ac.length();
    outputPoints[2][1] =  std::sin(angle) * ac.length();

    return outputPoints;
}

OpenMesh::Vec3f Algorithms::barycentric(const OpenMesh::Vec3f &_point1, const OpenMesh::Vec3f &_point2, const OpenMesh::Vec3f &_point3, const OpenMesh::Vec3f &_point)
{
    auto v0 = _point2 - _point1;
    auto v1 = _point3 - _point1;
    auto v2 = _point - _point1;

    auto d00 = OpenMesh::dot(v0, v0);
    auto d01 = OpenMesh::dot(v0, v1);
    auto d11 = OpenMesh::dot(v1, v1);
    auto d20 = OpenMesh::dot(v2, v0);
    auto d21 = OpenMesh::dot(v2, v1);
    auto denom = d00 * d11 - d01 * d01;

    OpenMesh::Vec3f barycentric;
    barycentric[1] = (d11 * d20 - d01 * d21) / denom;
    barycentric[2] = (d00 * d21 - d01 * d20) / denom;
    barycentric[0] = 1.0f - barycentric[1] - barycentric[2];

    return barycentric;
}

OpenMesh::Vec3f Algorithms::barycentric(const std::array<OpenMesh::Vec3f, 3> &_trianglePoints, const OpenMesh::Vec3f &_point)
{
    return barycentric(_trianglePoints[0], _trianglePoints[1], _trianglePoints[2], _point);
}

float Algorithms::computeTriangleArea(const std::array<OpenMesh::Vec3f, 3> &_triangle)
{
    return 0.5f * OpenMesh::cross(_triangle[1] - _triangle[0], _triangle[2] - _triangle[0]).norm();
}

bool Algorithms::Compare3DPoints::operator()(const OpenMesh::Vec3f &_point1, const OpenMesh::Vec3f &_point2)
{
    const static auto epsilon = 1e-5; // or we can use 1e-8

    if (_point1[0] < _point2[0] - epsilon) return true;
    if (_point1[0] > _point2[0] + epsilon) return false;

    if (_point1[1] < _point2[1] - epsilon) return true;
    if (_point1[1] > _point2[1] + epsilon) return false;

    if (_point1[2] < _point2[2] - epsilon) return true;

    return false;
}

bool Algorithms::CompareHandles::operator()(const OpenMesh::BaseHandle& _handle1, const OpenMesh::BaseHandle& _handle2)
{
    return _handle1.idx() < _handle2.idx();
}

bool Algorithms::CompareTriangle::operator()(const std::array<OpenMesh::Vec3f, 3> &_triangle1, const std::array<OpenMesh::Vec3f, 3> &_triangle2)
{
    return computeTriangleArea(_triangle1) < computeTriangleArea(_triangle2);
}
