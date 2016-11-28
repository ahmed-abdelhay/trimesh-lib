#pragma once

#include <vector>
#include <array>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::DefaultTraits>  TriangleMesh;

class TriMesh : public TriangleMesh
{
public:
    TriMesh();
    virtual ~TriMesh();

    // clear all the selected faces states
    void clearFaceSelection();

    // mark a face with a given selection state
    // Parameter _fh: the face handle
    // Parameter _selectionState: the selection state
    void setFaceSelectionState(const OpenMesh::FaceHandle _fh, bool _selectionState);

    // Returns a list of the selected faces in the mesh
    std::vector<OpenMesh::FaceHandle> getSelectedFaces();

    // get the point of the face with handle _faceHandle
    std::array<Point, 3> getFacePoints(const FaceHandle& _fh) const;

    // calculate face normal
    // Parameter _fh: the face handle
    // Returns: the normal of the face
    OpenMesh::Vec3f calculateFaceNormal(const FaceHandle& _fh) const;

private:
    OpenMesh::FPropHandleT<bool> mFaceSelectionProperty;
};
