#pragma once

#include <TriMeshDefs.h>

#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Core/Mesh/Handles.hh>

// Axis Alligned Bounding Box
struct TRIMESHLIBSHARED_DEFS AABB
{
    AABB()
        :faceHandle(-1)
    {}

    float calculateVolume() const
    {
        auto diff = maxPoint - minPoint;
        return diff[0] * diff[1] *  diff[2];
    }

    OpenMesh::Vec3f minPoint;
    OpenMesh::Vec3f maxPoint;
    OpenMesh::FaceHandle faceHandle;
};
