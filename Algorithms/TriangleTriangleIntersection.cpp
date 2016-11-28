#include "TriangleTriangleIntersection.h"

#include <OpenMesh/Core/Geometry/MathDefs.hh>

bool TriangleTriangleIntersection::coplanartriangleTriangleIntersection(const OpenMesh::Vec3f& _v0, const OpenMesh::Vec3f& _v1, const OpenMesh::Vec3f& _v2,
                                                                        const OpenMesh::Vec3f& _u0, const OpenMesh::Vec3f& _u1, const OpenMesh::Vec3f& _u2,
                                                                        IntersectionResultsType& _intersectionResults)
{
    return false;
}

void TriangleTriangleIntersection::computeIntersectionPoints(const OpenMesh::Vec3f& _vertex0,
                                                             const OpenMesh::Vec3f& _vertex1,
                                                             const OpenMesh::Vec3f& _vertex2,
                                                             const OpenMesh::Vec3f& _triangleProjectionIntoLine,
                                                             const OpenMesh::Vec3f& _pointsSignedDistance,
                                                             std::array<float, 2>& _intersectionInterval,
                                                             OpenMesh::Vec3f& _intersectioPoint0, OpenMesh::Vec3f& _intersectioPoint1)
{
    auto tmp = _pointsSignedDistance[0] / (_pointsSignedDistance[0] - _pointsSignedDistance[1]);
    _intersectionInterval[0] = _triangleProjectionIntoLine[0] + (_triangleProjectionIntoLine[1] - _triangleProjectionIntoLine[0]) * tmp;
    auto diff = _vertex1 - _vertex0;
    diff *= tmp;
    _intersectioPoint0 = diff + _vertex0;

    tmp = _pointsSignedDistance[0] / (_pointsSignedDistance[0] - _pointsSignedDistance[2]);
    _intersectionInterval[1] = _triangleProjectionIntoLine[0] + (_triangleProjectionIntoLine[2] - _triangleProjectionIntoLine[0]) * tmp;
    diff = _vertex2 - _vertex0;
    diff *= tmp;
    _intersectioPoint1 = _vertex0 + diff;
}

bool TriangleTriangleIntersection::computeIntervalsOnIntersectionline(const OpenMesh::Vec3f& _vertex0, const OpenMesh::Vec3f& _vertex1,
                                                                      const OpenMesh::Vec3f& _vertex2,
                                                                      const OpenMesh::Vec3f& _triangleProjectionIntoLine,
                                                                      const OpenMesh::Vec3f& _pointsToPlaneSignedDistance,
                                                                      std::array<float, 2>& _intersectionInterval,
                                                                      OpenMesh::Vec3f& _intersectionPoint0, OpenMesh::Vec3f& _intersectionPoint1)
{
    auto du0du1 = _pointsToPlaneSignedDistance[0] * _pointsToPlaneSignedDistance[1];
    auto du0du2 = _pointsToPlaneSignedDistance[0] * _pointsToPlaneSignedDistance[2];
    auto du1du2 = _pointsToPlaneSignedDistance[1] * _pointsToPlaneSignedDistance[2];

    if (du0du1 > 0.0f)
    {
        // here we know that du0du2 <= 0.0
        // that is D0, D1 are on the same side, D2 on the other or on the plane
        computeIntersectionPoints(_vertex2,_vertex0,_vertex1,
                                  OpenMesh::Vec3f(_triangleProjectionIntoLine[2],_triangleProjectionIntoLine[0],_triangleProjectionIntoLine[1]),
                OpenMesh::Vec3f(_pointsToPlaneSignedDistance[2],_pointsToPlaneSignedDistance[0],_pointsToPlaneSignedDistance[1]),
                _intersectionInterval,
                _intersectionPoint0,_intersectionPoint1);
    }
    else if (du0du2 > 0.0f)
    {
        // here we know that d0d1 <= 0.0
        computeIntersectionPoints(_vertex1,_vertex0,_vertex2,
                                  OpenMesh::Vec3f(_triangleProjectionIntoLine[1],_triangleProjectionIntoLine[0],_triangleProjectionIntoLine[2]),
                OpenMesh::Vec3f(_pointsToPlaneSignedDistance[1],_pointsToPlaneSignedDistance[0],_pointsToPlaneSignedDistance[2]),
                _intersectionInterval,
                _intersectionPoint0,_intersectionPoint1);
    }
    else if (du1du2 > 0.0f || _pointsToPlaneSignedDistance[0] != 0.0f)
    {
        // here we know that d0d1 <= 0.0 or that D0 != 0.0
        computeIntersectionPoints(_vertex0,_vertex1,_vertex2,
                                  OpenMesh::Vec3f(_triangleProjectionIntoLine[0],_triangleProjectionIntoLine[1],_triangleProjectionIntoLine[2]),
                OpenMesh::Vec3f(_pointsToPlaneSignedDistance[0],_pointsToPlaneSignedDistance[1],_pointsToPlaneSignedDistance[2]),
                _intersectionInterval,
                _intersectionPoint0,_intersectionPoint1);
    }
    else if (_pointsToPlaneSignedDistance[1] != 0.0f)
    {
        computeIntersectionPoints(_vertex1,_vertex0,_vertex2,
                                  OpenMesh::Vec3f(_triangleProjectionIntoLine[1],_triangleProjectionIntoLine[0],_triangleProjectionIntoLine[2]),
                OpenMesh::Vec3f(_pointsToPlaneSignedDistance[1],_pointsToPlaneSignedDistance[0],_pointsToPlaneSignedDistance[2]),
                _intersectionInterval,
                _intersectionPoint0,_intersectionPoint1);
    }
    else if (_pointsToPlaneSignedDistance[2] != 0.0f)
    {
        computeIntersectionPoints(_vertex2,_vertex0,_vertex1,
                                  OpenMesh::Vec3f(_triangleProjectionIntoLine[2],_triangleProjectionIntoLine[0],_triangleProjectionIntoLine[1]),
                OpenMesh::Vec3f(_pointsToPlaneSignedDistance[2],_pointsToPlaneSignedDistance[0],_pointsToPlaneSignedDistance[1]),
                _intersectionInterval,
                _intersectionPoint0,_intersectionPoint1);
    }
    else
    {
        return true;
    }
    return false;
}

bool TriangleTriangleIntersection::triangleTriangleIntersection(const OpenMesh::Vec3f& _v0, const OpenMesh::Vec3f& _v1, const OpenMesh::Vec3f& _v2,
                                                                const OpenMesh::Vec3f& _u0, const OpenMesh::Vec3f& _u1, const OpenMesh::Vec3f& _u2,
                                                                IntersectionResultsType& _intersectionResults)
{
    // compute plane equation of triangle1
    auto triangle1Edge1 = _v1 - _v0;
    auto triangle1Edge2 = _v2 - _v0;
    auto triangle1Normal = OpenMesh::cross(triangle1Edge1, triangle1Edge2);
    auto d1 = -OpenMesh::dot(triangle1Normal, _v0);

    // plane equation 1: triangle1Normal.X + d1 = 0

    // put triangle 2 face points into plane equation 1 to compute signed distances to the plane
    auto triangle2Point0SignedDistanceToTriangle1 = OpenMesh::dot(triangle1Normal, _u0) + d1;
    auto triangle2Point1SignedDistanceToTriangle1 = OpenMesh::dot(triangle1Normal, _u1) + d1;
    auto triangle2Point2SignedDistanceToTriangle1 = OpenMesh::dot(triangle1Normal, _u2) + d1;

    static const auto epsilon = std::numeric_limits<float>::epsilon();

    if (std::fabs(triangle2Point0SignedDistanceToTriangle1) < epsilon) triangle2Point0SignedDistanceToTriangle1 = 0.0f;
    if (std::fabs(triangle2Point1SignedDistanceToTriangle1) < epsilon) triangle2Point1SignedDistanceToTriangle1 = 0.0f;
    if (std::fabs(triangle2Point2SignedDistanceToTriangle1) < epsilon) triangle2Point2SignedDistanceToTriangle1 = 0.0f;

    // fast rejections if the points are on the same side of the plane
    // if the points are on difference sides then
    // their signed distances should contain positive and negative values
    // then the multiplication of the values must be negative
    auto du0du1 = triangle2Point0SignedDistanceToTriangle1 * triangle2Point1SignedDistanceToTriangle1;
    auto du0du2 = triangle2Point0SignedDistanceToTriangle1 * triangle2Point2SignedDistanceToTriangle1;

    // same sign on all of them + not equal 0 ?
    if (du0du1 > 0.0f && du0du2 > 0.0f)
        return false;  // no intersection occurs

    // compute plane of triangle2
    auto triangle2Edge1 = _u1 - _u0;
    auto triangle2Edge2 = _u2 - _u0;
    auto triangle2Normal = OpenMesh::cross(triangle2Edge1, triangle2Edge2);
    auto d2 = -OpenMesh::dot(triangle2Normal, _u0);

    // plane equation 2: triangle2Normal . X + d2 = 0
    // put triangle 1 into plane equation 2
    auto triangle1Point0SignedDistanceToTriangle2 = OpenMesh::dot(triangle2Normal, _v0) + d2;
    auto triangle1Point1SignedDistanceToTriangle2 = OpenMesh::dot(triangle2Normal, _v1) + d2;
    auto triangle1Point2SignedDistanceToTriangle2 = OpenMesh::dot(triangle2Normal, _v2) + d2;

    if (std::fabs(triangle1Point0SignedDistanceToTriangle2) < epsilon) triangle1Point0SignedDistanceToTriangle2 = 0.0f;
    if (std::fabs(triangle1Point1SignedDistanceToTriangle2) < epsilon) triangle1Point1SignedDistanceToTriangle2 = 0.0f;
    if (std::fabs(triangle1Point2SignedDistanceToTriangle2) < epsilon) triangle1Point2SignedDistanceToTriangle2 = 0.0f;

    auto dv0dv1 = triangle1Point0SignedDistanceToTriangle2 * triangle1Point1SignedDistanceToTriangle2;
    auto dv0dv2 = triangle1Point0SignedDistanceToTriangle2 * triangle1Point2SignedDistanceToTriangle2;

    // same sign on all of them + not equal 0 ?
    if (dv0dv1 > 0.0f && dv0dv2 > 0.0f)
        return false;   // no intersection occurs

    // compute direction of intersection line
    auto intersctionLine = OpenMesh::cross(triangle1Normal, triangle2Normal);

    // compute and index to the largest component of intersectionLine
    auto maxValue = std::fabs(intersctionLine[0]);
    auto maxIndex = 0;

    for (int i = 1; i < 3; ++i)
    {
        auto value = std::fabs(intersctionLine[i]);
        if (value > maxValue)
        {
            maxValue = value;
            maxIndex = i;
        }
    }

    // this is the simplified projection onto intersectionLine
    OpenMesh::Vec3f triangle1ProjectionIntoLine(_v0[maxIndex], _v1[maxIndex], _v2[maxIndex]);
    OpenMesh::Vec3f triangle2ProjectionIntoLine(_u0[maxIndex], _u1[maxIndex], _u2[maxIndex]);

    std::array<float, 2> triangle1IntersectionInterval;
    std::array<float, 2> triangle2IntersectionInterval;

    OpenMesh::Vec3f triangle1IntersectionPoint1, triangle1IntersectionPoint2;
    OpenMesh::Vec3f triangle2IntersectionPoint1, triangle2IntersectionPoint2;

    // compute interval for triangle 1
    auto coplanar = computeIntervalsOnIntersectionline(_v0, _v1, _v2, triangle1ProjectionIntoLine,
                                                       OpenMesh::Vec3f(triangle1Point0SignedDistanceToTriangle2,
                                                                       triangle1Point1SignedDistanceToTriangle2,
                                                                       triangle1Point2SignedDistanceToTriangle2),
                                                       triangle1IntersectionInterval,
                                                       triangle1IntersectionPoint1, triangle1IntersectionPoint2);

    // if triangles are coplanar intersect them
    if (coplanar)
        return coplanartriangleTriangleIntersection(_v0, _v1, _v2, _u0, _u1, _u2, _intersectionResults);


    // compute interval for triangle 2
    computeIntervalsOnIntersectionline(_u0, _u1, _u2, triangle2ProjectionIntoLine,
                                       OpenMesh::Vec3f(triangle2Point0SignedDistanceToTriangle1,
                                                       triangle2Point1SignedDistanceToTriangle1,
                                                       triangle2Point2SignedDistanceToTriangle1),
                                       triangle2IntersectionInterval,
                                       triangle2IntersectionPoint1,triangle2IntersectionPoint2);

    auto sort = [](std::array<float, 2>& _array) -> size_t
    {
        if (OpenMesh::is_gt(_array[0], _array[1]))
        {
            std::swap(_array[0], _array[1]);
            return 1;
        }
        else
        {
            return 0;
        }
    };

    auto smallest1 = sort(triangle1IntersectionInterval);
    auto smallest2 = sort(triangle2IntersectionInterval);

    if (triangle1IntersectionInterval[1] < triangle2IntersectionInterval[0] ||
            triangle2IntersectionInterval[1] < triangle1IntersectionInterval[0])
        return false;

    std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f> intersectionSegment;
    // at this point, we know that the triangles intersect
    if (triangle2IntersectionInterval[0] < triangle1IntersectionInterval[0])
    {
        if (smallest1 == 0)
        {
            intersectionSegment.first = triangle1IntersectionPoint1;
        }
        else
        {
            intersectionSegment.first = triangle1IntersectionPoint2;
        }

        if (triangle2IntersectionInterval[1] < triangle1IntersectionInterval[1])
        {
            if (smallest2 == 0)
            {
                intersectionSegment.second = triangle2IntersectionPoint2;
            }
            else
            {
                intersectionSegment.second = triangle2IntersectionPoint1;
            }
        }
        else
        {
            if (smallest1 == 0)
            {
                intersectionSegment.second = triangle1IntersectionPoint2;
            }
            else
            {
                intersectionSegment.second = triangle1IntersectionPoint1;
            }
        }
    }
    else
    {
        if (smallest2 == 0)
        {
            intersectionSegment.first = triangle2IntersectionPoint1;
        }
        else
        {
            intersectionSegment.first = triangle2IntersectionPoint2;
        }

        if (triangle2IntersectionInterval[1] > triangle1IntersectionInterval[1])
        {
            if (smallest1 == 0)
            {
                intersectionSegment.second = triangle1IntersectionPoint2;
            }
            else
            {
                intersectionSegment.second = triangle1IntersectionPoint1;
            }
        }
        else
        {
            if (smallest2 == 0)
            {
                intersectionSegment.second = triangle2IntersectionPoint2;
            }
            else
            {
                intersectionSegment.second = triangle2IntersectionPoint1;
            }
        }
    }

    _intersectionResults.push_back(intersectionSegment);
    return true;
}

bool TriangleTriangleIntersection::triangleTriangleIntersection(const std::array<OpenMesh::Vec3f, 3> &_face1Points,
                                                                const std::array<OpenMesh::Vec3f, 3> &_face2Points,
                                                                TriangleTriangleIntersection::IntersectionResultsType &_intersectionResults)
{
    return triangleTriangleIntersection(_face1Points[0], _face1Points[1], _face1Points[2],
            _face2Points[0], _face2Points[1], _face2Points[2], _intersectionResults);
}
