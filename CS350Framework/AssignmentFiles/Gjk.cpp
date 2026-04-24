///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Precompiled.hpp"

#include <numeric>

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints,
                                const Matrix4& transform) const
{
    const Vector3 centroid{std::accumulate(
    localPoints.begin(), localPoints.end(),
    Vector3(),
    [](const Vector3& a, const Vector3& b)
    {
        return a + b;
    }) / static_cast<float>(localPoints.size())};

    return Math::TransformPoint(transform, centroid);
}

Vector3 SupportShape::Support(const Vector3& worldDirection,
                              const std::vector<Vector3>& localPoints,
                              const Matrix4& localToWorldTransform) const
{
    const Vector3 local_direction{
    Math::TransformNormal(localToWorldTransform.Inverted(), worldDirection)};

    Vector3 support_point{};
    float max_dot{0.f};
    for (const auto point : localPoints)
    {
        const float dot{point.Dot(local_direction)};
        if (dot > max_dot)
        {
            max_dot = dot;
            support_point = point;
        }
    }

    return Math::TransformPoint(localToWorldTransform, support_point);
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints,
                             const Matrix4& localToWorldTransform,
                             const Vector4& color) const
{
    /******Student:Assignment5******/
    Warn("Assignment5: Required function un-implemented");
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
    return SupportShape::GetCenter(
    mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
    return SupportShape::Support(
    worldDirection, mModel->mMesh->mVertices,
    mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
    SupportShape::DebugDraw(mModel->mMesh->mVertices,
                            mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
    mScale = Vector3(1);
    mRotation = Matrix3::cIdentity;
    mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const { return mSphere.mCenter; }

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
    return GetCenter() + worldDirection.Normalized() * mSphere.GetRadius();
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
    DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
    shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const { return mTranslation; }

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
    // TODO: Ask about the way they expect us to make the AABB here 
    // (just to clarify from the slides)

    /******Student:Assignment5******/
    Warn("Assignment5: Required function un-implemented");
    return Vector3::cZero;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    DebugShape& shape =
    gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
    shape.Color(color);
    shape.SetTransform(transform);
}

//------------------------------------------------------------ Voronoi Region
// Tests
VoronoiRegion::Type
Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, size_t& newSize,
                           int newIndices[4], Vector3& closestPoint,
                           Vector3& searchDirection)
{
    closestPoint = p0;
    searchDirection = q - p0;
    newIndices[0] = 0;
    newSize = 1;
    return VoronoiRegion::Point0;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
const Vector3& q, const Vector3& p0, const Vector3& p1, size_t& newSize,
int newIndices[4], Vector3& closestPoint, Vector3& searchDirection)
{
    float u, v;
    BarycentricCoordinates(q, p0, p1, u, v);

    VoronoiRegion::Type output;

    if (u <= 0.f)
    {
        closestPoint = p1;
        output = VoronoiRegion::Point1;

        newSize = 1;
        newIndices[0] = 1;
    }
    else if (v <= 0.f)
    {
        closestPoint = p0;
        output = VoronoiRegion::Point0;

        newSize = 1;
        newIndices[0] = 0;
    }
    else
    {
        closestPoint = p0 * u + p1 * v;
        output = VoronoiRegion::Edge01;

        newSize = 2;
        newIndices[0] = 0;
        newIndices[1] = 1;
    }

    searchDirection = q - closestPoint;
    return output;
}

VoronoiRegion::Type
Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
                           const Vector3& p1, const Vector3& p2,
                           size_t& newSize, int newIndices[4],
                           Vector3& closestPoint, Vector3& searchDirection)
{
    // Checking if it is inside the triangle
    float u, v, w;
    if (BarycentricCoordinates(q, p0, p1, p2, u, v, w))
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 2;
        newSize = 3;

        closestPoint = p0 * u + p1 * v + p2 * w;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Triangle012;
    }

    // Discarding p0 case
    if (u < 0.f)
    {
        float u_12, v_12;
        if (BarycentricCoordinates(q, p1, p2, u_12, v_12))
        {
            newIndices[0] = 1;
            newIndices[1] = 2;
            newSize = 2;

            closestPoint = p1 * u_12 + p2 * v_12;
            searchDirection = (q - closestPoint).Normalized();

            return VoronoiRegion::Edge12;
        }

        newIndices[0] = u_12 < 0.f ? 2 : 1;
        newSize = 1;

        closestPoint = u_12 < 0.f ? p2 : p1;
        searchDirection = (q - closestPoint).Normalized();

        return u_12 < 0.f ? VoronoiRegion::Point2 : VoronoiRegion::Point1;
    }

    // Discarding p1 case
    if (v < 0.f)
    {
        float u_20, v_20;
        if (BarycentricCoordinates(q, p2, p0, u_20, v_20))
        {
            newIndices[0] = 0;
            newIndices[1] = 2;
            newSize = 2;

            closestPoint = p2 * u_20 + p0 * v_20;
            searchDirection = (q - closestPoint).Normalized();

            return VoronoiRegion::Edge02;
        }

        newIndices[0] = u_20 < 0.f ? 0 : 2;
        newSize = 1;

        closestPoint = u_20 < 0.f ? p0 : p2;
        searchDirection = (q - closestPoint).Normalized();

        return u_20 < 0.f ? VoronoiRegion::Point0 : VoronoiRegion::Point2;
    }

    // Discarding p2 case
    if (w < 0.f)
    {
        float u_01, v_01;
        if (BarycentricCoordinates(q, p0, p1, u_01, v_01))
        {
            newIndices[0] = 0;
            newIndices[1] = 1;
            newSize = 2;

            closestPoint = p0 * u_01 + p1 * v_01;
            searchDirection = (q - closestPoint).Normalized();

            return VoronoiRegion::Edge01;
        }

        newIndices[0] = u_01 < 0.f ? 1 : 0;
        newSize = 1;

        closestPoint = u_01 < 0.f ? p1 : p0;
        searchDirection = (q - closestPoint).Normalized();

        return u_01 < 0.f ? VoronoiRegion::Point1 : VoronoiRegion::Point0;
    }

    return VoronoiRegion::Unknown;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
const Vector3& p3, size_t& newSize, int newIndices[4], Vector3& closestPoint,
Vector3& searchDirection)
{
    /******Student:Assignment5******/
    Warn("Assignment5: Required function un-implemented");
    return VoronoiRegion::Unknown;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB,
                    unsigned int maxIterations, CsoPoint& closestPoint,
                    float epsilon, int debuggingIndex, bool debugDraw)
{
    Warn("Assignment5: Required function un-implemented");
    return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA,
                                  const SupportShape* shapeB,
                                  const Vector3& direction)
{
    /******Student:Assignment5******/
    CsoPoint result = {};
    Warn("Assignment5: Required function un-implemented");

    return result;
}
