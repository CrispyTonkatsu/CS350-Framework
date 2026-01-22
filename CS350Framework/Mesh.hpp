///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <string>
#include <vector>
#include "Shapes.hpp"

class Mesh
{
public:
    Mesh()
    {
        mDynamic = false;
        mType = 0;
    }

    Mesh(const std::string& name, int type)
    {
        mName = name;
        mDynamic = false;
        mType = type;
    }

    size_t TriangleCount() const { return mIndices.size() / 3; }

    Triangle TriangleAt(size_t triangleIndex) const
    {
        Triangle tri;
        tri.mPoints[0] = mVertices[mIndices[triangleIndex * 3 + 0]];
        tri.mPoints[1] = mVertices[mIndices[triangleIndex * 3 + 1]];
        tri.mPoints[2] = mVertices[mIndices[triangleIndex * 3 + 2]];
        return tri;
    }

    using Vertices = std::vector<Vector3>;
    Vertices mVertices;
    using Indices = std::vector<size_t>;
    Indices mIndices;

    std::string mName;
    bool mDynamic;
    int mType;
};
