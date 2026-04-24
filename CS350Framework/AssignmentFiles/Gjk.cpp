///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Precompiled.hpp"

#include <numeric>

namespace utils
{
    float get_side(const Vector3& q, const Vector3& p0, const Vector3& p1,
                   const Vector3& p2, const Vector3& back)
    {
        Vector3 normal{(p1 - p0).Cross(p2 - p0)};
        if (normal.Dot(back - p0) < 0.f)
        {
            normal *= -1.f;
        }

        return normal.Dot(q - p0);
    }
}

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
    float max_dot{std::numeric_limits<float>::lowest()};
    for (const Vector3& point : localPoints)
    {
        const float dot{point.Dot(local_direction)};
        if (dot >= max_dot)
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
    const Vector3 local_direction{
    Math::Transform(mRotation.Transposed(), worldDirection)};

    const Vector3 unit_cube_point{
    Math::GetSign(local_direction.x),
    Math::GetSign(local_direction.y),
    Math::GetSign(local_direction.z),
    };

    const Vector3 local_obb_point{unit_cube_point * (mScale * 0.5f)};
    return Math::Transform(mRotation, local_obb_point) + mTranslation;
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
    float u_12, v_12;
    BarycentricCoordinates(q, p1, p2, u_12, v_12);

    float u_20, v_20;
    BarycentricCoordinates(q, p2, p0, u_20, v_20);

    float u_01, v_01;
    BarycentricCoordinates(q, p0, p1, u_01, v_01);

    // Catching the single vertex cases
    if (v_01 <= 0.f && u_20 <= 0.f)
    {
        newIndices[0] = 0;
        newSize = 1;

        closestPoint = p0;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point0;
    }

    if (u_01 <= 0.f && v_12 <= 0.f)
    {
        newIndices[0] = 1;
        newSize = 1;

        closestPoint = p1;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point1;
    }

    if (u_12 <= 0.f && v_20 <= 0.f)
    {
        newIndices[0] = 2;
        newSize = 1;

        closestPoint = p2;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point2;
    }

    float u, v, w;
    BarycentricCoordinates(q, p0, p1, p2, u, v, w);

    if (u_01 > 0.f && v_01 > 0.f && w <= 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newSize = 2;

        closestPoint = p0 * u_01 + p1 * v_01;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge01;
    }

    if (u_12 > 0.f && v_12 > 0.f && u <= 0.f)
    {
        newIndices[0] = 1;
        newIndices[1] = 2;
        newSize = 2;

        closestPoint = p1 * u_12 + p2 * v_12;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge12;
    }

    if (u_20 > 0.f && v_20 > 0.f && v <= 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 2;
        newSize = 2;

        closestPoint = p2 * u_20 + p0 * v_20;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge02;
    }

    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    newSize = 3;

    closestPoint = p0 * u + p1 * v + p2 * w;
    searchDirection = q - closestPoint;

    return VoronoiRegion::Triangle012;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
const Vector3& p3, size_t& newSize, int newIndices[4], Vector3& closestPoint,
Vector3& searchDirection)
{
    float u_01, v_01;
    BarycentricCoordinates(q, p0, p1, u_01, v_01);

    float u_20, v_20;
    BarycentricCoordinates(q, p2, p0, u_20, v_20);

    float u_03, v_03;
    BarycentricCoordinates(q, p0, p3, u_03, v_03);

    float u_12, v_12;
    BarycentricCoordinates(q, p1, p2, u_12, v_12);

    float u_13, v_13;
    BarycentricCoordinates(q, p1, p3, u_13, v_13);

    float u_32, v_32;
    BarycentricCoordinates(q, p3, p2, u_32, v_32);

    if (v_01 <= 0.f && u_20 <= 0.f && v_03 <= 0.f)
    {
        newIndices[0] = 0;
        newSize = 1;

        closestPoint = p0;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point0;
    }

    if (u_01 <= 0.f && v_12 <= 0.f && v_13 <= 0.f)
    {
        newIndices[0] = 1;
        newSize = 1;

        closestPoint = p1;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point1;
    }

    if (u_12 <= 0.f && v_20 <= 0.f && u_32 <= 0.f)
    {
        newIndices[0] = 2;
        newSize = 1;

        closestPoint = p2;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point2;
    }

    if (u_13 <= 0.f && v_32 <= 0.f && u_03 <= 0.f)
    {
        newIndices[0] = 3;
        newSize = 1;

        closestPoint = p3;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Point3;
    }

    float u_031, v_031, w_031;
    BarycentricCoordinates(q, p0, p3, p1, u_031, v_031, w_031);

    float u_023, v_023, w_023;
    BarycentricCoordinates(q, p0, p2, p3, u_023, v_023, w_023);

    float u_213, v_213, w_213;
    BarycentricCoordinates(q, p2, p1, p3, u_213, v_213, w_213);

    float u_012, v_012, w_012;
    BarycentricCoordinates(q, p0, p1, p2, u_012, v_012, w_012);

    if (w_012 <= 0.f && v_031 <= 0.f && u_01 > 0.f && v_01 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newSize = 2;

        closestPoint = p0 * u_01 + p1 * v_01;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge01;
    }

    if (u_012 <= 0.f && w_213 <= 0.f && u_12 > 0.f && v_12 > 0.f)
    {
        newIndices[0] = 1;
        newIndices[1] = 2;
        newSize = 2;

        closestPoint = p1 * u_12 + p2 * v_12;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge12;
    }

    if (v_012 <= 0.f && w_023 <= 0.f && u_20 > 0.f && v_20 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 2;
        newSize = 2;

        closestPoint = p2 * u_20 + p0 * v_20;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge02;
    }

    if (v_213 <= 0.f && u_023 <= 0.f && u_32 > 0.f && v_32 > 0.f)
    {
        newIndices[0] = 2;
        newIndices[1] = 3;
        newSize = 2;

        closestPoint = p3 * u_32 + p2 * v_32;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge23;
    }

    if (v_023 <= 0.f && w_031 <= 0.f && u_03 > 0.f && v_03 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 3;
        newSize = 2;

        closestPoint = p0 * u_03 + p3 * v_03;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge03;
    }

    if (u_213 <= 0.f && u_031 <= 0.f && u_13 > 0.f && v_13 > 0.f)
    {
        newIndices[0] = 1;
        newIndices[1] = 3;
        newSize = 2;

        closestPoint = p1 * u_13 + p3 * v_13;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Edge13;
    }

    const float t{utils::get_side(q, p0, p1, p2, p3)};
    if (t < 0.f && u_012 > 0.f && v_012 > 0.f && w_012 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 2;
        newSize = 3;

        closestPoint = p0 * u_012 + p1 * v_012 + p2 * w_012;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Triangle012;
    }

    const float u{utils::get_side(q, p2, p1, p3, p0)};
    if (u < 0.f && u_213 > 0.f && v_213 > 0.f && w_213 > 0.f)
    {
        newIndices[0] = 1;
        newIndices[1] = 2;
        newIndices[2] = 3;
        newSize = 3;

        closestPoint = p2 * u_213 + p1 * v_213 + p3 * w_213;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Triangle123;
    }

    const float v{utils::get_side(q, p0, p2, p3, p1)};
    if (v < 0.f && u_023 > 0.f && v_023 > 0.f && w_023 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 2;
        newIndices[2] = 3;
        newSize = 3;

        closestPoint = p0 * u_023 + p2 * v_023 + p3 * w_023;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Triangle023;
    }

    const float w{utils::get_side(q, p0, p3, p1, p2)};
    if (w < 0.f && u_031 > 0.f && v_031 > 0.f && w_031 > 0.f)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 3;
        newSize = 3;

        closestPoint = p0 * u_031 + p3 * v_031 + p1 * w_031;
        searchDirection = q - closestPoint;

        return VoronoiRegion::Triangle013;
    }

    newIndices[0] = 0;
    newIndices[1] = 1;
    newIndices[2] = 2;
    newIndices[3] = 3;
    newSize = 4;

    // No need to calculate because it's the point inside
    closestPoint = q;
    searchDirection = Vector3::cZero;

    return VoronoiRegion::Tetrahedra0123;
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
