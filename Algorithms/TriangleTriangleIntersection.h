#pragma once

#include <OpenMesh/Core/Geometry/VectorT.hh>

#include <array>
#include <vector>

// this class contains implmenetation of triangle triangle intersection
// based on A Fast Triangle-Triangle Intersection Test paper by
// Tomas Moller http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
class TriangleTriangleIntersection
{
    using SegmentType = std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f>;
    // a vector of segments
    using IntersectionResultsType = std::vector<SegmentType>;

public:
    // intersect 2 triangles
    // Parameter _v0: triangle 1 point 1
    // Parameter _v1: triangle 1 point 2
    // Parameter _v2: triangle 1 point 3
    // Parameter _u0: triangle 2 point 1
    // Parameter _u1: triangle 2 point 2
    // Parameter _u2: triangle 2 point 3
    // Parameter _intersectionsResults: the intersections segments of the 2 triangles
    // Returns true if the 2 triangles intersect and false otherwise
    static bool triangleTriangleIntersection(const OpenMesh::Vec3f& _v0, const OpenMesh::Vec3f& _v1, const OpenMesh::Vec3f& _v2,
                                             const OpenMesh::Vec3f& _u0, const OpenMesh::Vec3f& _u1, const OpenMesh::Vec3f& _u2,
                                             IntersectionResultsType& _intersectionResults);

    // intersect 2 triangles
    // Parameter _face1Points: triangle 1 points
    // Parameter _face2Points: triangle 2 points
    // Parameter _intersectionsResults: the intersections segments of the 2 triangles
    // Returns true if the 2 triangles intersect and false otherwise
    static bool triangleTriangleIntersection(const std::array<OpenMesh::Vec3f, 3>& _face1Points,
                                             const std::array<OpenMesh::Vec3f, 3>& _face2Points,
                                             IntersectionResultsType& _intersectionResults);

private:
    // intersect 2 coplanar triangles
    // Parameter _v0: triangle 1 point 1
    // Parameter _v1: triangle 1 point 2
    // Parameter _v2: triangle 1 point 3
    // Parameter _u0: triangle 2 point 1
    // Parameter _u1: triangle 2 point 2
    // Parameter _u2: triangle 2 point 3
    // Parameter _intersectionsResults: the intersections segments of the 2 triangles
    // Returns true if the 2 triangles intersect and false otherwise
    static bool coplanartriangleTriangleIntersection(const OpenMesh::Vec3f& _v0, const OpenMesh::Vec3f& _v1, const OpenMesh::Vec3f& _v2,
                                                     const OpenMesh::Vec3f& _u0, const OpenMesh::Vec3f& _u1, const OpenMesh::Vec3f& _u2,
                                                     IntersectionResultsType& _intersectionResults);


    // compute the intersection intervals and points
    // Parameter _vertex0: triangle point 1
    // Parameter _vertex1: triangle point 2
    // Parameter _vertex2: triangle point 3
    // Parameter _triangleProjectionIntoLine: triangle points projection into the intrsection line
    // Parameter _pointsSignedDistance: triangle points signed distance to the other triangle
    // Parameter _intersectionInterval: intersection segment intervals on the intersection line
    // Parameter _intersectionPoint0: the intersections segments point 0
    // Parameter _intersectionPoint1: the intersections segments point 1
    // Returns true if the 2 triangles are coplanar and false otherwise
    static void computeIntersectionPoints(const OpenMesh::Vec3f& _vertex0,
                                          const OpenMesh::Vec3f& _vertex1,
                                          const OpenMesh::Vec3f& _vertex2,
                                          const OpenMesh::Vec3f& _triangleProjectionIntoLine,
                                          const OpenMesh::Vec3f& _pointsSignedDistance,
                                          std::array<float, 2>& _intersectionInterval,
                                          OpenMesh::Vec3f& _intersectioPoint0, OpenMesh::Vec3f& _intersectioPoint1);



    // compute the intersection intervals and points
    // Parameter _vertex0: triangle point 1
    // Parameter _vertex1: triangle point 2
    // Parameter _vertex2: triangle point 3
    // Parameter _triangleProjectionIntoLine: triangle points projection into the intrsection line
    // Parameter _pointsToPlaneSignedDistance: triangle points signed distance to the other triangle
    // Parameter _intersectionInterval: intersection segment intervals on the intersection line
    // Parameter _intersectionPoint0: the intersections segments point 0
    // Parameter _intersectionPoint1: the intersections segments point 1
    // Returns true if the 2 triangles are coplanar and false otherwise
    static bool computeIntervalsOnIntersectionline(const OpenMesh::Vec3f& _vertex0,
                                                   const OpenMesh::Vec3f& _vertex1,
                                                   const OpenMesh::Vec3f& _vertex2,
                                                   const OpenMesh::Vec3f& _triangleProjectionIntoLine,
                                                   const OpenMesh::Vec3f& _pointsToPlaneSignedDistance,
                                                   std::array<float, 2>& _intersectionInterval,
                                                   OpenMesh::Vec3f& _intersectionPoint0, OpenMesh::Vec3f& _intersectionPoint1);
};
