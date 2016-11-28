#include "AABBTree.h"

#include "Mesh/TriMesh.h"
#include "Algorithms/Algorithms.h"
#include "Types/AABB.h"

#include <omp.h>

AABBTree::AABBTree(TriMesh& _mesh)
    :mMesh(_mesh)
{}

void AABBTree::build()
{
    int meshFacesCount = mMesh.n_faces();
    std::vector<AABB> meshFacesBBoxes(meshFacesCount);

#pragma omp parallel for
    for (int i = 0; i < meshFacesCount; ++i)
    {
        meshFacesBBoxes[i] = calculateFaceAABB(OpenMesh::FaceHandle(i));
    }

    mRootNode = build(meshFacesBBoxes);
}

AABBTree::IntersectionPairListType AABBTree::intersectWithTree(const AABBTree &_tree)
{
    IntersectionPairListType intersectionPairs;
    intersectNodeWithNode(mRootNode, _tree.mRootNode, intersectionPairs);
    return intersectionPairs;
}

AABB AABBTree::calculateFaceAABB(const OpenMesh::FaceHandle &_faceHandle) const
{
    auto facePoints = mMesh.getFacePoints(_faceHandle);
    AABB faceBoundingBox;
    faceBoundingBox.minPoint = facePoints[0];
    faceBoundingBox.maxPoint = facePoints[0];
    faceBoundingBox.faceHandle = _faceHandle;

    for (int i = 1; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (facePoints[i][j] > faceBoundingBox.maxPoint[j]) faceBoundingBox.maxPoint[j] = facePoints[i][j];
            if (facePoints[i][j] < faceBoundingBox.minPoint[j]) faceBoundingBox.minPoint[j] = facePoints[i][j];
        }
    }

    return faceBoundingBox;
}

AABB AABBTree::calculateBoxesAABB(const std::vector<AABB> &_boxes)
{
    if (_boxes.empty())
        throw std::logic_error("boxes are empty");

    AABB boxesAABB;
    boxesAABB.minPoint = _boxes[0].minPoint;
    boxesAABB.maxPoint = _boxes[0].maxPoint;

    int boxesCount = _boxes.size();
    for (int i = 1; i < boxesCount; ++i)
    {
        auto& box = _boxes[i];
        for (int j = 0; j < 3; ++j)
        {
            if (box.maxPoint[j] > boxesAABB.maxPoint[j]) boxesAABB.maxPoint[j] = box.maxPoint[j];
            if (box.minPoint[j] < boxesAABB.minPoint[j]) boxesAABB.minPoint[j] = box.minPoint[j];
        }
    }

    return boxesAABB;
}

std::unique_ptr<AABBTreeNode> AABBTree::build(std::vector<AABB> &_boxes)
{
    if (_boxes.size() == 1) // leaf node
        return std::make_unique<AABBTreeNode>(_boxes[0]);

    auto boxesAABB = calculateBoxesAABB(_boxes);

    auto node = std::make_unique<AABBTreeNode>(boxesAABB);

    // get the split direction
    auto splitDirection = 0;

    auto diff = boxesAABB.maxPoint - boxesAABB.minPoint;
    auto maxValue = diff[0];

    for (int i = 1; i < 3; ++i)
    {
        if (diff[i] > maxValue)
        {
            maxValue = diff[i];
            splitDirection = i;
        }
    }

    auto cutAxis = (boxesAABB.minPoint[splitDirection] + boxesAABB.maxPoint[splitDirection]) / 2.0f;

    size_t numberOfBoxesOnPositiveSide = 0;
    size_t numberOfBoxesOnNegativSide = 0;

    std::vector<AABB> boxesOnPositiveSide;
    std::vector<AABB> boxesOnNegativeSide;

    for (const auto& bbox : _boxes)
    {
        if ((bbox.minPoint[splitDirection] + bbox.maxPoint[splitDirection]) / 2.0 > cutAxis)
        {
            boxesOnPositiveSide.push_back(bbox);
            numberOfBoxesOnPositiveSide++;
        }
        else
        {
            boxesOnNegativeSide.push_back(bbox);
            numberOfBoxesOnNegativSide++;
        }
    }

    if (numberOfBoxesOnPositiveSide == 0)
    {
        auto middleIterator = boxesOnNegativeSide.begin();
        std::advance(middleIterator, std::ceil((numberOfBoxesOnNegativSide - 1) / 2.0f));
        boxesOnPositiveSide.insert(boxesOnPositiveSide.end(), middleIterator, boxesOnNegativeSide.end());
        auto distance = std::distance(middleIterator, boxesOnNegativeSide.end());
        for (int i = 0; i < distance; ++i)
            boxesOnNegativeSide.pop_back();
    }
    else if (numberOfBoxesOnNegativSide == 0)
    {
        auto middleIterator = boxesOnPositiveSide.begin();
        std::advance(middleIterator, std::ceil((numberOfBoxesOnPositiveSide - 1) / 2.0f));
        boxesOnNegativeSide.insert(boxesOnNegativeSide.end(), middleIterator, boxesOnPositiveSide.end());
        auto distance = std::distance(middleIterator, boxesOnPositiveSide.end());
        for (int i = 0; i < distance; ++i)
            boxesOnPositiveSide.pop_back();
    }

    node->rightChild = build(boxesOnPositiveSide);
    node->leftChild = build(boxesOnNegativeSide);

    return node;
}

void AABBTree::intersectNodeWithNode(const std::unique_ptr<AABBTreeNode> &_node1, const std::unique_ptr<AABBTreeNode> &_node2, IntersectionPairListType& _intersectionList)
{
    if (_node1 == nullptr || _node2 == nullptr)
        return;

    if (!Algorithms::checkAxisAlligndBoudnigBoxesIntersections(_node1->nodeAABB, _node2->nodeAABB))
        return;

    if (_node1->isLeafNode() && _node2->isLeafNode())
    {
        _intersectionList.emplace_back(_node1->nodeAABB.faceHandle, _node2->nodeAABB.faceHandle);
    }
    else if (_node2->isLeafNode() ||
             (!_node1->isLeafNode() &&
              _node1->nodeAABB.calculateVolume() > _node2->nodeAABB.calculateVolume()))
    {
        intersectNodeWithNode (_node1->leftChild, _node2, _intersectionList);
        intersectNodeWithNode (_node1->rightChild, _node2, _intersectionList);
    }
    else
    {
        intersectNodeWithNode (_node1, _node2->leftChild, _intersectionList);
        intersectNodeWithNode (_node1, _node2->rightChild, _intersectionList);
    }
}
