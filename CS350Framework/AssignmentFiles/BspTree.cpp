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

#include <stack>

BspTreeQueryData::BspTreeQueryData()
{
    mDepth = 0;
}

enum class Side: uint8_t
{
    Front = IntersectionType::Inside,
    Back = IntersectionType::Outside,
    Coplanar = IntersectionType::Coplanar,
    Count = 3
};

namespace utils
{
    static inline IntersectionType::Type plane_triangle(
    const Plane& plane, const Triangle& triangle, const float epsilon)
    {
        return PlaneTriangle(
        plane.mData,
        triangle.mPoints[0],
        triangle.mPoints[1],
        triangle.mPoints[2],
        epsilon);
    }

    static inline bool plane_ray(
    const Plane& plane, const Ray& ray, float& t_out, float epsilon = 0.0001f)
    {
        return RayPlane(
        ray.mStart,
        ray.mDirection,
        plane.mData,
        t_out,
        epsilon);
    }

    static inline Vector3 triangle_normal(const Triangle& triangle)
    {
        return (triangle.mPoints[1] - triangle.mPoints[0]).Cross(
        triangle.mPoints[2] - triangle.mPoints[0]).Normalized();
    }

    static inline Plane triangle_to_plane(const Triangle& triangle)
    {
        const Vector3 centroid{
        (triangle.mPoints[0] + triangle.mPoints[1] + triangle.mPoints[2]) /
        3.f};
        return {triangle_normal(triangle), centroid};
    }

    constexpr uint8_t side_pair(
    const IntersectionType::Type current,
    const IntersectionType::Type next)
    {
        uint8_t mask{static_cast<uint8_t>(current)};
        mask |= (static_cast<uint8_t>(next) << 2);
        return mask;
    }

    constexpr uint8_t side_pair(
    const Side current,
    const Side next)
    {
        uint8_t mask{static_cast<uint8_t>(current)};
        mask |= (static_cast<uint8_t>(next) << 2);
        return mask;
    }

    static inline Vector3 intersection(const Vector3& a, const Vector3& b,
                                       const Plane& plane)
    {
        float t_out;
        const Ray ray(a, b - a);
        plane_ray(plane, ray, t_out);

        return ray.GetPoint(t_out);
    }

}

void BspTree::SplitTriangle(
const Plane& plane, const Triangle& tri,
TriangleList& coplanarFront,
TriangleList& coplanarBack,
TriangleList& front,
TriangleList& back,
float epsilon)
{
    switch (utils::plane_triangle(plane, tri, epsilon))
    {
    case IntersectionType::Coplanar:
    {
        if (utils::triangle_normal(tri).Dot(plane.GetNormal()) > 0)
        {
            coplanarFront.push_back(tri);
        }
        else
        {
            coplanarBack.push_back(tri);
        }
    }
        return;
    case IntersectionType::Outside:
        back.push_back(tri);
        return;
    case IntersectionType::Inside:
        front.push_back(tri);
        return;
    case IntersectionType::Overlaps:
    case IntersectionType::NotImplemented:
        break;
    }

    std::vector<Vector3> front_verts;
    std::vector<Vector3> back_verts;

    for (int i = 0; i < 3; i++)
    {
        const Vector3& current{tri.mPoints[i]};
        const Vector3& next{tri.mPoints[(i + 1) % 3]};

        const IntersectionType::Type a_inter{PointPlane(
        current, plane.mData, epsilon)};

        const IntersectionType::Type b_inter{PointPlane(
        next, plane.mData, epsilon)};

        const uint8_t mask = utils::side_pair(a_inter, b_inter);

        switch (mask)
        {
        case utils::side_pair(Side::Front, Side::Front):
        case utils::side_pair(Side::Coplanar, Side::Front):
            front_verts.push_back(next);
            break;

        case utils::side_pair(Side::Back, Side::Front):
        {
            const Vector3 intersection{
            utils::intersection(current, next, plane)};
            front_verts.push_back(intersection);
            front_verts.push_back(next);

            back_verts.push_back(intersection);
        }
        break;

        case utils::side_pair(Side::Front, Side::Coplanar):
        case utils::side_pair(Side::Coplanar, Side::Coplanar):
            front_verts.push_back(next);
            break;

        case utils::side_pair(Side::Back, Side::Coplanar):
            front_verts.push_back(next);
            back_verts.push_back(next);
            break;

        case utils::side_pair(Side::Front, Side::Back):
        {
            const Vector3 intersection{
            utils::intersection(current, next, plane)};
            front_verts.push_back(intersection);

            back_verts.push_back(intersection);
            back_verts.push_back(next);
        }
        break;

        case utils::side_pair(Side::Coplanar, Side::Back):
            back_verts.push_back(current);
            back_verts.push_back(next);
            break;

        case utils::side_pair(Side::Back, Side::Back):
            back_verts.push_back(next);
            break;

        default: ;
        }
    }

    const auto fill_side{
    [](TriangleList& list, const std::vector<Vector3>& points)
    {
        if (points.empty())
        {
            return;
        }

        list.emplace_back(points[0], points[1], points[2]);
        
        if (points.size() > 3)
        {
            list.emplace_back(points[0], points[2], points[3]);
        }
    }};

    fill_side(front, front_verts);
    fill_side(back, back_verts);
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex,
                              float k, float epsilon)
{
    const Triangle& input_triangle{triangles[testIndex]};
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            if ((input_triangle.mPoints[i] - input_triangle.mPoints[j]).
                LengthSq() <
                0.0001f)
                return Math::PositiveMax();
        }
    }

    const Plane plane{utils::triangle_to_plane(input_triangle)};

    float front{0};
    float coplanar{0};
    float back{0};

    for (const Triangle& triangle : triangles)
    {
        switch (utils::plane_triangle(plane, triangle, epsilon)) // NOLINT
        {
        case IntersectionType::Outside:
            back++;
            break;
        case IntersectionType::Inside:
            front++;
            break;
        case IntersectionType::Overlaps:
            coplanar++;
            break;
        }
    }
    return k * coplanar + (1 - k) * Math::Abs(front - back);
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k,
                               float epsilon)
{
    float min_score{Math::PositiveMax()};
    size_t index = 0;
    for (size_t i = 0; i < triangles.size(); i++)
    {
        const float score{CalculateScore(triangles, i, k, epsilon)};
        if (score < min_score)
        {
            index = i;
            min_score = score;
        }
    }
    return index;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
    tree.clear();
    tree.root = tree.construct(nullptr, triangles, k, epsilon, 0);
}

// bool BspTree::RayCastRecursive(NodeID id, const Ray& ray, float& t, float tmin,
//                                float tmax, float planeEpsilon,
//                                float triExpansionEpsilon, int debuggingIndex)
// {
//     /*
//     cases:
//     1. ray start is coplaplar, visit both sides and all triangles
//     2. ray is parallel to the plane. determined by rayplane returning false.
//     - only recurse down the near side
//     3. ray clipped by plane should start/end at the start/end of thickplane
//     3.5 ray-> plane intersection should check thick plane
//
//     */
//
//     const BSPTreeNode& node = GetNode(id);
//     const Vector3 Ray_Min = ray.mDirection * tmin + ray.mStart;
//     //const Vector3 Ray_Max = ray.mDirection * tmax + ray.mStart;
//
//     const IntersectionType::Type point_plane = PointPlane(
//     Ray_Min, node.plane.mData, planeEpsilon);
//     const bool Ray_start_side = IntersectionType::Outside != point_plane;
//     //edge case 1. test both sides + coplanar geometry. C
//     const bool ray_start_coplanar = point_plane == IntersectionType::Coplanar;
//     float ray_plane_t = NAN;
//     float ray_plane_t_thick_diff = NAN;
//     bool ray_plane_hit = false;
//     if (!ray_start_coplanar)
//         ray_plane_hit = RayPlane(Ray_Min, ray.mDirection, node.plane.mData,
//                                  ray_plane_t);
//     if (ray_plane_hit)
//     {
//         if (ray_plane_t > tmax)
//             ray_plane_hit = false;
//         else
//             ray_plane_t_thick_diff = std::abs(
//             planeEpsilon / cosf(atan2f(ray.mDirection.y, ray.mDirection.x)));
//     }
//
//     if (!ray_start_coplanar && !ray_plane_hit && node.GetChild(Ray_start_side)
//         != -1)
//     {
//         float recur_t;
//         bool hit = RayCastRecursive(node.GetChild(Ray_start_side), ray, recur_t,
//                                     tmin, tmax, planeEpsilon,
//                                     triExpansionEpsilon, debuggingIndex);
//         if (hit)
//         {
//             t = recur_t;
//             return true;
//         }
//     }
//     if (ray_start_coplanar)
//     {
//         const float trans_TMIN = tmin, trans_TMAX = tmax;
//         float recur_t_a = INFINITY, recur_t_b = INFINITY, recur_t_c = INFINITY;
//         bool hit = node.GetChild(Ray_start_side) != -1 && RayCastRecursive(
//         node.GetChild(Ray_start_side), ray, recur_t_a, tmin, trans_TMAX,
//         planeEpsilon, triExpansionEpsilon,
//         debuggingIndex);
//
//
//         hit |= node.GetChild(!Ray_start_side) != -1 && RayCastRecursive(
//         node.GetChild(!Ray_start_side), ray, recur_t_b,
//         trans_TMIN, tmax, planeEpsilon,
//         triExpansionEpsilon, debuggingIndex);
//
//         bool tri_hit = node.RayTriangles(ray, recur_t_c, triExpansionEpsilon);
//         if (tri_hit && recur_t_c <= trans_TMAX && recur_t_c >= trans_TMIN)
//         {
//             hit = true;
//         }
//         if (hit)
//         {
//             t = std::min(recur_t_a, std::min(recur_t_b, recur_t_c));
//             return true;
//         }
//     }
//     if (ray_plane_hit)
//     {
//         const float trans_TMIN = tmin, trans_TMAX = tmax;
//         if (ray_plane_hit)
//         {
//             //trans_TMAX = ray_plane_t;
//             //trans_TMIN = ray_plane_t;
//             //trans_TMIN += ray_plane_t_thick_diff;
//             //trans_TMAX -= ray_plane_t_thick_diff;
//         }
//         float recur_t;
//         bool hit = node.GetChild(Ray_start_side) != -1 && RayCastRecursive(
//         node.GetChild(Ray_start_side), ray, recur_t, tmin, trans_TMAX,
//         planeEpsilon, triExpansionEpsilon,
//         debuggingIndex);
//         if (hit)
//         {
//             t = recur_t;
//             return true;
//         }
//
//         hit = node.GetChild(!Ray_start_side) != -1 && RayCastRecursive(
//         node.GetChild(!Ray_start_side), ray, recur_t,
//         trans_TMIN, tmax, planeEpsilon,
//         triExpansionEpsilon, debuggingIndex);
//         if (hit)
//         {
//             t = recur_t;
//             return true;
//         }
//         hit = node.RayTriangles(ray, recur_t, triExpansionEpsilon);
//         if (hit && recur_t <= trans_TMAX && recur_t >= trans_TMIN)
//         {
//             t = recur_t;
//             return true;
//         }
//     }
//
//     return false;
// }

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon,
                      float triExpansionEpsilon, int debuggingIndex)
{
    /******Student:Assignment4******/
    // return RayCastRecursive(root, ray, t, 0, INFINITY, planeEpsilon,
    //                         triExpansionEpsilon, debuggingIndex);


    return false;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
    // /******Student:Assignment4******/
    // NodeLambdaDown(root, [&](NodeID id)
    // {
    //     const BSPTreeNode& node = GetNode(id);
    //     triangles.insert(triangles.end(), node.coplanar_front.begin(),
    //                      node.coplanar_front.end());
    //     triangles.insert(triangles.end(), node.coplanar_back.begin(),
    //                      node.coplanar_back.end());
    // });
}

void BspTree::Invert()
{
    // /******Student:Assignment4******/
    // NodeLambdaDown(root, [&](NodeID id)
    // {
    //     BSPTreeNode& node = GetNode(id);
    //     //std::swap(node.coplanar_back, node.coplanar_front);
    //     //std::reverse(node.coplanar_back.begin(), node.coplanar_back.end());
    //     //std::reverse(node.coplanar_front.begin(), node.coplanar_front.end());
    //     std::swap(node.left, node.right);
    //     node.plane.mData *= -1;
    //     std::reverse(node.coplanar_front.begin(), node.coplanar_front.end());
    //     std::reverse(node.coplanar_back.begin(), node.coplanar_back.end());
    //
    //     for (auto& tri : node.coplanar_back)
    //     {
    //         std::swap(tri.mPoints[0], tri.mPoints[1]);
    //     }
    //     for (auto& tri : node.coplanar_front)
    //     {
    //         std::swap(tri.mPoints[0], tri.mPoints[1]);
    //     }
    // });
}

// void BspTree::ClipPolygonToBSP(BspTree& tree_a, const BspTree& tree_b,
//                                const Triangle& tri, const NodeID B_id,
//                                std::vector<Triangle>& result, float epsilon)
// {
//     const BSPTreeNode& B = tree_b.GetNode(B_id);
//     //BreakOnTriangle(Triangle(Vector3(1,2,-2), Vector3(0,0,1), Vector3(-1,2,-2)), tri);
//     if (B.IsLeaf())
//     {
//         if (B.isTriangleInside(tri, epsilon))
//         {
//             result.push_back(tri); // Keep if inside
//             return;
//         }
//     }
//
//     // Classify triangle against splitting plane
//     const IntersectionType::Type side = PlaneTriangle(
//     B.plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2],
//     epsilon);
//
//     if (side == IntersectionType::Outside)
//     {
//         if (B.right != INVALID_NODE)
//             ClipPolygonToBSP(tree_a, tree_b, tri, B.right, result, epsilon);
//         else
//             result.push_back(tri);
//     }
//     else if (side == IntersectionType::Inside)
//     {
//         if (B.left != INVALID_NODE)
//             ClipPolygonToBSP(tree_a, tree_b, tri, B.left, result, epsilon);
//         //else
//         //	result.push_back(tri);
//     }
//     else if (side == IntersectionType::Overlaps)
//     {
//         // Split the triangle into two parts
//         std::vector<Triangle> frontPart, backPart, coplanars;
//         SplitTriangle(B.plane, tri, coplanars, coplanars, frontPart, backPart,
//                       epsilon);
//         for (const auto& coplanar : coplanars)
//         {
//             if (TriangleNormal(tri).Dot(B.plane.GetNormal()) > 0)
//             {
//                 frontPart.push_back(coplanar);
//             }
//             else
//                 backPart.push_back(coplanar);
//         }
//         if (!B.IsLeaf())
//         {
//             if (B.right != INVALID_NODE)
//             {
//                 for (const auto& tri : frontPart)
//                 {
//                     ClipPolygonToBSP(tree_a, tree_b, tri, B.right, result,
//                                      epsilon);
//                 }
//             }
//             else
//             {
//                 for (const auto& tri : frontPart)
//                 {
//                     result.push_back(tri);
//                 }
//             }
//             if (B.left != INVALID_NODE)
//             {
//                 for (const auto& tri : backPart)
//                 {
//                     ClipPolygonToBSP(tree_a, tree_b, tri, B.left, result,
//                                      epsilon);
//                 }
//             }
//             else
//             {
//                 //for (const auto& tri : backPart)
//                 //	result.push_back(tri);
//             }
//         }
//         else
//         {
//             for (const auto& tri : frontPart)
//                 result.push_back(tri);
//         }
//     }
//     else if (side == IntersectionType::Coplanar)
//     {
//         if (TriangleNormal(tri).Dot(B.plane.GetNormal()) > 0)
//         {
//             if (B.right != INVALID_NODE)
//                 ClipPolygonToBSP(tree_a, tree_b, tri, B.right, result, epsilon);
//             if (B.left != INVALID_NODE)
//                 ClipPolygonToBSP(tree_a, tree_b, tri, B.left, result, epsilon);
//         }
//     }
// }

void ClipPolygonsToBSP(BspTree& tree_a, const BspTree& tree_b,
                       std::vector<Triangle>& polygons,
                       const size_t& B_id, float epsilon)
{
    std::vector<Triangle> result;
    for (Triangle& tri : polygons)
    {
        //BreakOnTriangle(Triangle(Vector3(1, 2, -2), Vector3(1, -2, -2), Vector3(-1, -2, -2)), tri);
        // ClipPolygonToBSP(tree_a, tree_b, tri, B_id, result, epsilon);
    }
    polygons = std::move(result);
}

void ClipToRecursive(BspTree& tree_a, const BspTree& tree_b,
                     const size_t A_id, const size_t B_id,
                     float epsilon)
{
    // BSPTreeNode& A = tree_a.GetNode(A_id);
    // const BSPTreeNode& B = tree_b.GetNode(B_id);
    //
    //
    // ClipPolygonsToBSP(tree_a, tree_b, A.coplanar_front, B_id, epsilon);
    // ClipPolygonsToBSP(tree_a, tree_b, A.coplanar_back, B_id, epsilon);
    //
    // if (A.left != INVALID_NODE)
    //     ClipToRecursive(tree_a, tree_b, A.left, B_id, epsilon);
    // if (A.right != INVALID_NODE)
    //     ClipToRecursive(tree_a, tree_b, A.right, B_id, epsilon);
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    /******Student:Assignment4******/

    // ClipToRecursive(*this, *tree, root, tree->root, epsilon);
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    ClipTo(tree, epsilon);

    tree->ClipTo(this, epsilon);

    tree->Invert();
    tree->ClipTo(this, epsilon);

    tree->Invert();
    std::vector<Triangle> A_triangles, b_triangles;
    AllTriangles(A_triangles);
    tree->AllTriangles(b_triangles);
    std::vector<Triangle> combined = A_triangles;
    combined.insert(combined.end(), b_triangles.begin(), b_triangles.end());
    Construct(combined, k, epsilon);
    tree->Construct(combined, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    Invert();
    tree->Invert();
    Union(tree, k, epsilon);
    Invert();
    tree->Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    tree->Invert();
    Intersection(tree, k, epsilon);
    tree->Invert();
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
    std::stack<Node*> to_visit{};
    to_visit.push(tree.root);

    while (!to_visit.empty())
    {
        Node* current_node{to_visit.top()};
        to_visit.pop();

        if (current_node == nullptr)
        {
            continue;
        }

        BspTreeQueryData data;
        data.mDepth = current_node->depth;
        data.mTriangles = current_node->coplanar_front;
        data.mTriangles.insert(
        data.mTriangles.end(),
        current_node->coplanar_back.begin(),
        current_node->coplanar_back.end());

        results.push_back(data);

        to_visit.push(current_node->back);
        to_visit.push(current_node->front);
    }
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
    /******Student:Assignment4******/

    //gDebugDrawer->DrawTriangle()
    // NodeLambdaDown(root, [&](NodeID id)
    // {
    //     BSPTreeNode& node = GetNode(id);
    //     if (level != -1 && node.depth != level)
    //         return;
    //     for (const auto& tri : node.coplanar_front)
    //         gDebugDrawer->DrawTriangle(tri).Color(color).SetMaskBit(bitMask);
    //     for (const auto& tri : node.coplanar_back)
    //         gDebugDrawer->DrawTriangle(tri).Color(color).SetMaskBit(bitMask);
    // });
}

BspTree::Node& BspTree::Tree::create_node()
{
    auto node{std::make_unique<Node>()};
    node->tree = this;

    const auto insert_location{nodes.emplace(node.get(), std::move(node))};
    return *insert_location.first->second;
}

BspTree::Node* BspTree::Tree::construct(
const Node* parent,
const TriangleList& triangles, float k,
float epsilon, int depth)
{
    if (triangles.empty())
    {
        return nullptr;
    }

    Node& node{create_node()};
    if (parent == nullptr)
    {
        root = &node;
    }

    const size_t plane_index{owner->PickSplitPlane(triangles, k, epsilon)};
    node.plane = utils::triangle_to_plane(triangles[plane_index]);
    node.depth = depth;

    TriangleList front_triangles;
    TriangleList back_triangles;

    for (const auto& triangle : triangles)
    {
        SplitTriangle(node.plane, triangle, node.coplanar_front,
                      node.coplanar_back, front_triangles, back_triangles,
                      epsilon);
    }

    node.front = construct(&node, front_triangles, k, epsilon, depth + 1);
    node.back = construct(&node, back_triangles, k, epsilon, depth + 1);

    return &node;
}

void BspTree::Tree::clear()
{
    root = nullptr;
    nodes.clear();
}

// bool BSPTreeNode::RayTriangles(const Ray& ray, float& t, float epsilon) const
// {
//     size_t hit_index = -1;
//     float min_t = INFINITY;
//
//     for (size_t i = 0; i < coplanar_front.size(); i++)
//     {
//         const Triangle& tri = coplanar_front[i];
//         float t;
//         bool hit = RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0],
//                                tri.mPoints[1], tri.mPoints[2], t, epsilon);
//         if (hit && min_t > t)
//         {
//             min_t = t;
//             hit_index = i;
//         }
//     }
//     for (size_t i = 0; i < coplanar_back.size(); i++)
//     {
//         const Triangle& tri = coplanar_back[i];
//         float t;
//         bool hit = RayTriangle(ray.mStart, ray.mDirection, tri.mPoints[0],
//                                tri.mPoints[1], tri.mPoints[2], t, epsilon);
//         if (hit && min_t > t)
//         {
//             min_t = t;
//             hit_index = i;
//         }
//     }
//
//     t = min_t;
//
//     return hit_index != -1;
// }
//
// bool BSPTreeNode::isTriangleInside(const Triangle& tri, float epsilon) const
// {
//     const auto side = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1],
//                                     tri.mPoints[2], epsilon);
//     if (side == IntersectionType::Outside)
//         return true;
//     if (side == IntersectionType::Coplanar)
//     {
//         if (TriangleNormal(tri).Dot(plane.GetNormal()) > 0)
//             return true;
//     }
//     return false;
// }
