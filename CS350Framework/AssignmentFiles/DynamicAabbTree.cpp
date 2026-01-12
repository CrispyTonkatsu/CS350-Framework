///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
{
  mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
  Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
  Warn("Assignment3: Required function un-implemented");
}

