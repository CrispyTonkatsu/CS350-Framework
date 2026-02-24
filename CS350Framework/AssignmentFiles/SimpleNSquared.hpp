/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: SimpleNSquared.hpp
Purpose: Declaration of the N-Squared partition methods with and without bounding spheres
Language: C++
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_2
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 24-Feb-2026
End Header -------------------------------------------------------*/

#pragma once

#include <unordered_map>
#include "SpatialPartition.hpp"

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
// A very bad, brute force spatial partition that is used for assignment 1
// (before you get to implement something better). Do not implement your spatial
// partitions this way, make sure that update and remove are O(1) (no searches).
class NSquaredSpatialPartition : public SpatialPartition
{
public:
    NSquaredSpatialPartition();

    void InsertData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;
    void UpdateData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;
    void RemoveData(SpatialPartitionKey& key) override;

    void DebugDraw(int level, const Matrix4& transform,
                   const Vector4& color = Vector4(1), int bitMask = 0) override;

    void CastRay(const Ray& ray, CastResults& results) override;
    void CastFrustum(const Frustum& frustum, CastResults& results) override;

    void SelfQuery(QueryResults& results) override;

    void GetDataFromKey(const SpatialPartitionKey& key,
                        SpatialPartitionData& data) const override;
    void
    FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

    std::vector<void*> mData;
};

/******Student:Assignment2******/
// Implement the n-squared sphere spatial partition
//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
class BoundingSphereSpatialPartition : public SpatialPartition
{
public:
    BoundingSphereSpatialPartition();

    void InsertData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;
    void UpdateData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;
    void RemoveData(SpatialPartitionKey& key) override;

    void DebugDraw(int level, const Matrix4& transform,
                   const Vector4& color = Vector4(1), int bitMask = 0) override;

    void CastRay(const Ray& ray, CastResults& results) override;
    void CastFrustum(const Frustum& frustum, CastResults& results) override;

    void SelfQuery(QueryResults& results) override;

    void
    FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

    // Add your implementation here

    std::unordered_map<void*, SpatialPartitionData> partitions;
};
