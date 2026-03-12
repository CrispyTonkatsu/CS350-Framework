/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: Implementation of the N-Squared partition methods with and without bounding spheres
Language: C++
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_2
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 24-Feb-2026
End Header -------------------------------------------------------*/

#include "Geometry.hpp"
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key,
                                          SpatialPartitionData& data)
{
    // Doing this lazily (and bad, but it's n-squared...).
    // Just store as the key what the client data is so we can look it up later.
    key.mVoidKey = data.mClientData;
    mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key,
                                          SpatialPartitionData& data)
{
    // Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    // Find the key data and remove it
    for (size_t i = 0; i < mData.size(); ++i)
    {
        if (mData[i] == key.mVoidKey)
        {
            mData[i] = mData.back();
            mData.pop_back();
            break;
        }
    }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Matrix4& transform,
                                         const Vector4& color, int bitMask)
{
    // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i];
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum,
                                           CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i];
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        for (size_t j = i + 1; j < mData.size(); ++j)
        {
            results.AddResult(QueryResult(mData[i], mData[j]));
        }
    }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key,
                                              SpatialPartitionData& data) const
{
    data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(
std::vector<SpatialPartitionQueryData>& results) const
{
    for (size_t i = 0; i < mData.size(); ++i)
    {
        SpatialPartitionQueryData data;
        data.mClientData = mData[i];
        results.push_back(data);
    }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key,
                                                SpatialPartitionData& data)
{
    key.mVoidKey = data.mClientData;
    partitions.insert_or_assign(key.mVoidKey, data);
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key,
                                                SpatialPartitionData& data)
{
    partitions.at(key.mVoidKey) = data;
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    partitions.erase(key.mVoidKey);
}

void BoundingSphereSpatialPartition::DebugDraw(int level,
                                               const Matrix4& transform,
                                               const Vector4& color,
                                               int bitMask)
{
    for (auto& partition : partitions)
    {
        gDebugDrawer->DrawSphere(partition.second.mBoundingSphere)
                    .SetTransform(transform)
                    .Color(color)
                    .SetMaskBit(bitMask);
    }
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray,
                                             CastResults& results)
{
    for (auto& partition : partitions)
    {
        const Sphere& sphere{partition.second.mBoundingSphere};
        float t{0.f};
        if (RaySphere(ray.mStart, ray.mDirection, sphere.GetCenter(),
                      sphere.GetRadius(), t))
        {
            results.AddResult({partition.second.mClientData, t});
        }
    }
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum,
                                                 CastResults& results)
{
    for (auto& partition : partitions)
    {
        const Sphere& sphere{partition.second.mBoundingSphere};

        size_t last_axis{0};
        const IntersectionType::Type intersection_type{
        FrustumSphere(frustum.GetPlanes(), sphere.GetCenter(),
                      sphere.GetRadius(), last_axis),
        };

        if (intersection_type == IntersectionType::Inside ||
            intersection_type == IntersectionType::Overlaps)
        {
            results.AddResult({partition.second.mClientData, 0.f});
        }
    }
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
    for (auto i{partitions.begin()}; i != partitions.end(); ++i)
    {
        const auto& sphere_a{i->second.mBoundingSphere};
        for (auto j{std::next(i)}; j != partitions.end(); ++j)
        {
            const auto& sphere_b{j->second.mBoundingSphere};
            if (SphereSphere(sphere_a.GetCenter(), sphere_a.GetRadius(),
                             sphere_b.GetCenter(), sphere_b.GetRadius()))
            {
                results.AddResult(
                {i->second.mClientData, j->second.mClientData});
            }
        }
    }
}

void BoundingSphereSpatialPartition::FilloutData(
std::vector<SpatialPartitionQueryData>& results) const
{
    for (auto& partition : partitions)
    {
        SpatialPartitionQueryData data;
        data.mClientData = partition.second.mClientData;
        data.mBoundingSphere = partition.second.mBoundingSphere;

        results.push_back(data);
    }
}
