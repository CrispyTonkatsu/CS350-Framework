///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
  mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // Doing this lazily (and bad, but it's n-squared...).
  // Just store as the key what the client data is so we can look it up later.
  key.mVoidKey = data.mClientData;
  mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  // Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  // Find the key data and remove it
  for(size_t i = 0; i < mData.size(); ++i)
  {
    if(mData[i] == key.mVoidKey)
    {
      mData[i] = mData.back();
      mData.pop_back();
      break;
    }
  }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    CastResult result;
    result.mClientData = mData[i];
    results.AddResult(result);
  }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
  // Add everything
  for(size_t i = 0; i < mData.size(); ++i)
  {
    for(size_t j = i + 1; j < mData.size(); ++j)
    {
      results.AddResult(QueryResult(mData[i], mData[j]));
    }
  }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
  data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  for(size_t i = 0; i < mData.size(); ++i)
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

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
  Warn("Assignment2: Required function un-implemented");
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  Warn("Assignment2: Required function un-implemented");
}
