#pragma once

#include <vector>

#include "Types/AABB.h"
#include "Mesh/TriMesh.h"
#include "Trees/AABBTreeNode.h"

// Axis alligned bounding box tree class
// this class is used for spatial partitionning og 3D mesh
class AABBTree
{
    using IntersectionPairListType = std::vector<std::pair<OpenMesh::FaceHandle, OpenMesh::FaceHandle>>;
public:
    AABBTree(TriMesh& _mesh);

    // build the AABB
    // this method must be called before any quires
    void build();

    // intersect the tree with another tree _tree
    // it returns intersection candindates pairs
    // each pair is a FaceHandle in the first mesh and a FaceHandle in the second mesh
    IntersectionPairListType intersectWithTree(const AABBTree& _tree);

private:
    // calculate the AABB tree of the a face on the mMesh
    AABB calculateFaceAABB(const OpenMesh::FaceHandle& _faceHandle) const;

    // compuate the AABB that contains all the boxes in _boxes
    AABB calculateBoxesAABB(const std::vector<AABB>& _boxes);

    // Builds a new hierarchy of bounding boxes for _boxes. At each
    // level, the AABBNode->mNodeAABB field contains a bounding box of
    // all the children. The tree is binary and is built by repeatedly
    // cutting in two approximately equal halves the bounding boxes at
    // each level until a leaf node (i.e. a bounding box given in _boxes)
    // is reached. In order to minimize the depth of the tree, the cutting
    // direction is always chosen as perpendicular to the longest
    // dimension of the bounding box.
    // Parameters _boxes: the AABB of the mesh faces
    std::unique_ptr<AABBTreeNode> build(std::vector<AABB>& _boxes);


    void intersectNodeWithNode(const std::unique_ptr<AABBTreeNode>& _node1, const std::unique_ptr<AABBTreeNode>& _node2, IntersectionPairListType &_intersectionList);


    TriMesh& mMesh;
    std::unique_ptr<AABBTreeNode> mRootNode;
};
