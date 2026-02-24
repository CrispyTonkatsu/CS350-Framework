/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: Geometry.cpp
Purpose: Implementation of the available intersection tests
Language: C++
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_2
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 24-Feb-2026
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal,
                            float planeDistance)
{
    if (normal.LengthSq() == 0.f)
    {
        return Vector3{};
    }

    const Vector3 unit_normal{normal.Normalized()};
    const float difference{planeDistance - point.Dot(unit_normal)};

    return point + (unit_normal * difference);
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a,
                            const Vector3& b, float& u, float& v, float epsilon)
{
    u = {
    Math::Dot(point - b, a - b) / Math::Dot(a - b, a - b),
    };

    v = 1.f - u;

    return (u >= -epsilon) && (u <= 1.f + epsilon);
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a,
                            const Vector3& b, const Vector3& c, float& u,
                            float& v, float& w, float epsilon)
{
    const Vector3 v0{point - c};
    const Vector3 v1{a - c};
    const Vector3 v2{b - c};

    const float eq_a{v1.LengthSq()};
    const float eq_b{v1.Dot(v2)};
    const float eq_c{eq_b};
    const float eq_d{v2.LengthSq()};

    const float eq_e{v0.Dot(v1)};
    const float eq_f{v0.Dot(v2)};

    const auto det{
    [](float a, float b, float c, float d) { return (a * d) - (b * c); },
    };

    const float eq_det{det(eq_a, eq_b, eq_c, eq_d)};

    // Checking for no solutions
    if (eq_det <= -epsilon)
    {
        u = 0;
        v = 0;
        w = 0;

        return false;
    }

    u = {
    det(eq_e, eq_b, eq_f, eq_d) / eq_det,
    };

    v = {
    det(eq_a, eq_e, eq_c, eq_f) / eq_det,
    };

    w = 1 - u - v;

    return (u >= -epsilon && u <= 1.f + epsilon) &&
    (v >= -epsilon && v <= 1.f + epsilon) &&
    (w >= -epsilon && w <= 1.f + epsilon);
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane,
                                  float epsilon)
{
    const Vector4 homogeneous_point(point.x, point.y, point.z, -1.f);

    const float w{plane.Dot(homogeneous_point)};

    if (w > epsilon)
    {
        return IntersectionType::Inside;
    }

    if (w < -epsilon)
    {
        return IntersectionType::Outside;
    }

    return IntersectionType::Coplanar;
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter,
                 float sphereRadius)
{
    return (sphereCenter - point).Length() <= sphereRadius;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin,
               const Vector3& aabbMax)
{
    return ((point.x >= aabbMin.x) && (point.x <= aabbMax.x)) && //
    ((point.y >= aabbMin.y) && (point.y <= aabbMax.y)) && //
    ((point.z >= aabbMin.z) && (point.z <= aabbMax.z));
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float epsilon)
{
    ++Application::mStatistics.mRayPlaneTests;

    const Vector3 normal{plane.x, plane.y, plane.z};
    if (Math::Abs(rayDir.Dot(normal)) < epsilon)
    {
        return false;
    }

    t = (plane.w - normal.Dot(rayStart)) / normal.Dot(rayDir);

    return (t > -epsilon);
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1,
                 const Vector3& triP2, float& t, float triExpansionEpsilon)
{
    ++Application::mStatistics.mRayTriangleTests;

    const Vector3 normal{(triP1 - triP0).Cross(triP2 - triP0)};
    const Vector4 plane{normal.x, normal.y, normal.z, normal.Dot(triP0)};

    if (!RayPlane(rayStart, rayDir, plane, t))
    {
        return false;
    }

    if (t < -triExpansionEpsilon)
    {
        return false;
    }

    float u, v, w;
    return BarycentricCoordinates(rayStart + (t * rayDir), triP0, triP1, triP2,
                                  u, v, w, triExpansionEpsilon);
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius, float& t)
{
    ++Application::mStatistics.mRaySphereTests;

    const Vector3 x{rayStart - sphereCenter};

    const float A{rayDir.LengthSq()};
    const float B{2 * (rayDir.Dot(x))};
    const float C{x.LengthSq() - Math::Sq(sphereRadius)};

    const float discriminant{Math::Sq(B) - (4 * A * C)};

    if (discriminant < 0.f)
    {
        return false;
    }

    if (discriminant > 0.f)
    {
        const float t0{(-B - Math::Sqrt(discriminant)) / (2 * A)};
        const float t1{(-B + Math::Sqrt(discriminant)) / (2 * A)};

        if ((t0 < 0.f && t1 > 0.f) || (t0 > 0.f && t1 < 0.f))
        {
            t = 0.f;
            return true;
        }

        t = Math::Min(t0, t1);
        if (t < 0.f)
        {
            t = Math::Max(t0, t1);

            if (t < 0.f)
            {
                return false;
            }
        }
    }

    if (discriminant == 0.f)
    {
        t = (-B / (2 * A));

        if (t < 0.f)
        {
            return false;
        }
    }

    return true;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
             const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
    ++Application::mStatistics.mRayAabbTests;

    if (PointAabb(rayStart, aabbMin, aabbMax))
    {
        t = 0.f;
        return true;
    }

    float t_min{std::numeric_limits<float>::lowest()};
    float t_max{std::numeric_limits<float>::max()};

    for (unsigned int i{0}; i < 3; i++)
    {
        if (rayDir[i] == 0.f)
        {
            if (Math::InRange(rayStart[i], aabbMin[i], aabbMax[i]))
            {
                continue;
            }

            return false;
        }

        const float t_aabb_min{(aabbMin[i] - rayStart[i]) / rayDir[i]};
        const float t_aabb_max{(aabbMax[i] - rayStart[i]) / rayDir[i]};

        const float ti_min{rayDir[i] > 0.f ? t_aabb_min : t_aabb_max};
        const float ti_max{rayDir[i] > 0.f ? t_aabb_max : t_aabb_min};

        t_min = Math::Max(ti_min, t_min);
        t_max = Math::Min(ti_max, t_max);
    }

    t = t_min;
    if (t < 0.f)
    {
        t = t_max;

        if (t < 0.f)
        {
            return false;
        }
    }

    return t_min <= t_max;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane, const Vector3& triP0,
                                     const Vector3& triP1, const Vector3& triP2,
                                     float epsilon)
{
    ++Application::mStatistics.mPlaneTriangleTests;

    const IntersectionType::Type res_p0{PointPlane(triP0, plane, epsilon)};
    const IntersectionType::Type res_p1{PointPlane(triP1, plane, epsilon)};
    const IntersectionType::Type res_p2{PointPlane(triP2, plane, epsilon)};

    if ((res_p0 == res_p1) && (res_p0 == res_p2))
    {
        return res_p0;
    }

    const IntersectionType::Type results[] = {res_p0, res_p1, res_p2};

    for (size_t i{0}; i < 3; i++)
    {
        const size_t prev{(i == 0 ? 2 : i - 1) % 3};
        const size_t next{(i + 1) % 3};

        if (!((results[i] == results[next]) && (results[i] != results[prev])))
        {
            continue;
        }

        if (((results[i] == IntersectionType::Outside) ||
             (results[i] == IntersectionType::Inside)) &&
            ((results[prev] == IntersectionType::Outside) ||
             (results[prev] == IntersectionType::Inside)))
        {
            return IntersectionType::Overlaps;
        }

        if (results[i] == IntersectionType::Coplanar)
        {
            return results[prev];
        }

        return results[i];
    }

    return IntersectionType::Overlaps;
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter,
                                   float sphereRadius)
{
    ++Application::mStatistics.mPlaneSphereTests;

    const Vector3 normal{
    plane.x,
    plane.y,
    plane.z,
    };

    const Vector3 test_point{
    ProjectPointOnPlane(sphereCenter, normal, plane.w),
    };

    if (PointSphere(test_point, sphereCenter, sphereRadius))
    {
        return IntersectionType::Overlaps;
    }

    const Vector4 h_center{
    sphereCenter.x,
    sphereCenter.y,
    sphereCenter.z,
    -1.f,
    };

    if (plane.Dot(h_center) < 0.f)
    {
        return IntersectionType::Outside;
    }

    return IntersectionType::Inside;
}

IntersectionType::Type PlaneAabb(const Vector4& plane, const Vector3& aabbMin,
                                 const Vector3& aabbMax)
{
    ++Application::mStatistics.mPlaneAabbTests;

    const Vector4 h_min{
    -plane.x < 0.f ? aabbMin.x : aabbMax.x,
    -plane.y < 0.f ? aabbMin.y : aabbMax.y,
    -plane.z < 0.f ? aabbMin.z : aabbMax.z,
    -1.f,
    };

    const Vector4 h_max{
    plane.x < 0.f ? aabbMin.x : aabbMax.x,
    plane.y < 0.f ? aabbMin.y : aabbMax.y,
    plane.z < 0.f ? aabbMin.z : aabbMax.z,
    -1.f,
    };

    const float dot_min{plane.Dot(h_min)};
    const float dot_max{plane.Dot(h_max)};

    if ((dot_min >= 0.f && dot_max <= 0.f) ||
        (dot_min <= 0.f && dot_max >= 0.f))
    {
        return IntersectionType::Overlaps;
    }

    if (dot_min < 0.f)
    {
        return IntersectionType::Outside;
    }

    return IntersectionType::Inside;
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0,
                                       const Vector3& triP1,
                                       const Vector3& triP2, float epsilon)
{
    ++Application::mStatistics.mFrustumTriangleTests;

    size_t inside_count{0};

    for (size_t i{0}; i < 6; i++)
    {
        const IntersectionType::Type result{
        PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon)};

        if (result == IntersectionType::Outside)
        {
            return IntersectionType::Outside;
        }

        if (result == IntersectionType::Inside)
        {
            inside_count++;
        }
    }

    if (inside_count == 6)
    {
        return IntersectionType::Inside;
    }

    return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter,
                                     float sphereRadius, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumSphereTests;

    size_t inside_count{0};

    for (size_t i{0}; i < 6; i++)
    {
        const IntersectionType::Type result{
        PlaneSphere(planes[i], sphereCenter, sphereRadius)};

        if (result == IntersectionType::Outside)
        {
            return IntersectionType::Outside;
        }

        if (result == IntersectionType::Inside)
        {
            inside_count++;
        }
    }

    if (inside_count == 6)
    {
        return IntersectionType::Inside;
    }

    return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin,
                                   const Vector3& aabbMax, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumAabbTests;

    size_t inside_count{0};

    for (size_t i{0}; i < 6; i++)
    {
        const IntersectionType::Type result{
        PlaneAabb(planes[i], aabbMin, aabbMax)};

        if (result == IntersectionType::Outside)
        {
            return IntersectionType::Outside;
        }

        if (result == IntersectionType::Inside)
        {
            inside_count++;
        }
    }

    if (inside_count == 6)
    {
        return IntersectionType::Inside;
    }

    return IntersectionType::Overlaps;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1)
{
    ++Application::mStatistics.mSphereSphereTests;
    return PointSphere(sphereCenter0, sphereCenter1,
                       sphereRadius0 + sphereRadius1);
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1)
{
    ++Application::mStatistics.mAabbAabbTests;

    // To get the expression used here, invert the formula from the slides
    return ((aabbMin0.x <= aabbMax1.x) && (aabbMin1.x <= aabbMax0.x)) && //
    ((aabbMin0.y <= aabbMax1.y) && (aabbMin1.y <= aabbMax0.y)) && //
    ((aabbMin0.z <= aabbMax1.z) && (aabbMin1.z <= aabbMax0.z));
}
