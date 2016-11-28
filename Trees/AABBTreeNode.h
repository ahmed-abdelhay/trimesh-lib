#pragma once

#include <memory>

#include "Types/AABB.h"

struct AABBTreeNode
{
    AABBTreeNode()
        :leftChild(nullptr),
        rightChild(nullptr)
    {}

    AABBTreeNode(const AABB& _box)
        :leftChild(nullptr),
        rightChild(nullptr),
        nodeAABB(_box)
    {}

    inline bool isLeafNode() const
    {
        return (leftChild == nullptr || rightChild == nullptr);
    }

    std::unique_ptr<AABBTreeNode> leftChild;
    std::unique_ptr<AABBTreeNode> rightChild;

    AABB nodeAABB;
};
