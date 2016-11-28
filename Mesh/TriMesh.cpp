#include "TriMesh.h"


TriMesh::TriMesh()
{
    add_property(mFaceSelectionProperty);
    clearFaceSelection();
}

TriMesh::~TriMesh()
{
    remove_property(mFaceSelectionProperty);
}

void TriMesh::clearFaceSelection()
{
    for (const auto& fh : faces())
        property(mFaceSelectionProperty, fh) = false;
}

void TriMesh::setFaceSelectionState(const OpenMesh::FaceHandle _fh, bool _selectionState)
{
    property(mFaceSelectionProperty, _fh) = _selectionState;
}

std::vector<OpenMesh::FaceHandle> TriMesh::getSelectedFaces()
{
    std::vector<OpenMesh::FaceHandle> selectedFaces;
    for (const auto& fh : faces())
       if (property(mFaceSelectionProperty, fh))
           selectedFaces.push_back(fh);
    return selectedFaces;
}

std::array<TriMesh::Point, 3> TriMesh::getFacePoints(const FaceHandle &_fh) const
{
    int counter = 0;
    std::array<Point, 3> facePoints;
    for (const auto& vh : fv_range(_fh))
        facePoints[counter++] = point(vh);
    return facePoints;
}

OpenMesh::Vec3f TriMesh::calculateFaceNormal(const FaceHandle &_fh) const
{
    auto facePoints = getFacePoints(_fh);
    return OpenMesh::cross(facePoints[1] - facePoints[0], facePoints[2] - facePoints[0]).normalized();
}
