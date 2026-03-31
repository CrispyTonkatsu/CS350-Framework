/* Start Header ------------------------------------------------------
Copyright (C) 2026 DigiPen Institute of Technology.
File Name: BspTree.cpp
Purpose: Implementation of the available intersection tests
Language: C++
Platform: Windows MSVC version: 18.0.5.56406
Project: e.donosomansilla_CS350_2
Author: Edgar Jose Donoso Mansilla, e.donosomansilla, id: 0066578
Creation date: 30-March-2026
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

#include "BspTree.hpp"
#include "Geometry.hpp"

#include <iostream>
#include <vector>

BspTreeQueryData::BspTreeQueryData() { mDepth = 0; }

void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri,
                            TriangleList& coplanarFront,
                            TriangleList& coplanarBack, TriangleList& front,
                            TriangleList& back, float epsilon)
{
    Warn("Assignment4: Required function un-implemented");

    IntersectionType::Type intersection_type{PlaneTriangle(
    plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon)};

    switch (intersection_type) // NOLINT (The other case is handled below)
    {
    case IntersectionType::Outside:
        back.push_back(tri);
        return;
    case IntersectionType::Inside:
        front.push_back(tri);
        return;

    case IntersectionType::Coplanar:
        const Vector3 normal{
        Math::Cross(tri.mPoints[1] - tri.mPoints[0],
                    tri.mPoints[2] - tri.mPoints[0]),
        };

        (normal.Dot(plane.GetNormal()) > 0 ? coplanarFront : coplanarBack)
        .push_back(tri);
        return;
    }

    std::vector<Vector3> inside{};
    inside.reserve(4);

    std::vector<Vector3> outside{};
    outside.reserve(4);

    for (size_t i{0}; i < 3; i++)
    {
        const Vector3& current_point{tri.mPoints[i]};
        const Vector3& next_point{tri.mPoints[(i + 1) % 3]};

        // NOTE: This ray is the one to change to fix the ordering
        const Ray ray{current_point, next_point - current_point};

        // NOTE: Implement the clipping table
        float t;
        RayPlane(ray.mStart, ray.mDirection, plane.mData, t, epsilon);
        //
        // if (t < 0 || t > 1)
        // {
        //     continue;
        // }

        const Vector3 intersection{ray.GetPoint(t)};

        // NOTE: This is just one case, find a way to handle them
        // mathematically

        IntersectionType::Type current_side{
        PointPlane(current_point, plane.mData, epsilon),
        };

        IntersectionType::Type next_side{
        PointPlane(next_point, plane.mData, epsilon),
        };

        // NOTE: This is a 1:1 implementation of the table in the slides

        if (next_side == IntersectionType::Inside)
        {
            switch (current_side) // NOLINT
            {
            case IntersectionType::Inside:
            case IntersectionType::Coplanar:
                inside.push_back(next_point);
                break;

            case IntersectionType::Outside:
                inside.push_back(intersection);
                inside.push_back(next_point);

                outside.push_back(intersection);
                break;
            }
        }

        if (next_side == IntersectionType::Coplanar)
        {
            switch (current_side) // NOLINT
            {
            case IntersectionType::Inside:
            case IntersectionType::Coplanar:
                inside.push_back(next_point);
                break;

            case IntersectionType::Outside:
                inside.push_back(next_point);

                outside.push_back(next_point);
                break;
            }
        }

        if (next_side == IntersectionType::Outside)
        {
            switch (current_side) // NOLINT
            {
            case IntersectionType::Inside:
                inside.push_back(intersection);

                outside.push_back(intersection);
                outside.push_back(next_point);
                break;

            case IntersectionType::Coplanar:
                outside.push_back(current_point);
                outside.push_back(next_point);
                break;

            case IntersectionType::Outside:
                outside.push_back(next_point);
                break;
            }
        }
    }

    const auto fill_list{
    [](TriangleList& list, const std::vector<Vector3>& points)
    {
        list.emplace_back(points[0], points[1], points[2]);

        if (points.size() > 3)
        {
            list.emplace_back(points[0], points[2], points[3]);
        }
    },
    };

    fill_list(front, inside);
    fill_list(back, outside);
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex,
                              float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
    return Math::PositiveMax();
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k,
                               float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
    return 0;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon,
                      float triExpansionEpsilon, int debuggingIndex)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
    t = Math::PositiveMax();
    return false;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::Invert()
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");
}
