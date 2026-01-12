///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  static const float mFatteningFactor;

  // Add your implementation here
};
