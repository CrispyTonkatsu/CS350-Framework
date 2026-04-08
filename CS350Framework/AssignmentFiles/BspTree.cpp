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

#include <array>
#include <iostream>
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
    return S_PickSplitPlane(triangles, k, epsilon);
}

size_t BspTree::S_PickSplitPlane(const TriangleList& triangles, float k,
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

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon,
                      float triExpansionEpsilon, int debuggingIndex)
{
    if (tree.root == nullptr)
    {
        return false;
    }

    t = Math::PositiveMax();

    return tree.root->ray_cast(
    ray,
    0, Math::PositiveMax(), t,
    planeEpsilon, triExpansionEpsilon,
    debuggingIndex);
}

void BspTree::AllTriangles(TriangleList& triangles) const
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

        std::copy(current_node->coplanar_back.begin(),
                  current_node->coplanar_back.end(),
                  std::back_inserter(triangles));

        std::copy(current_node->coplanar_front.begin(),
                  current_node->coplanar_front.end(),
                  std::back_inserter(triangles));

        to_visit.push(current_node->back);
        to_visit.push(current_node->front);
    }
}

void BspTree::Invert()
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

        for (Triangle& triangle : current_node->coplanar_front)
        {
            std::swap(triangle.mPoints[0], triangle.mPoints[1]);
        }

        for (Triangle& triangle : current_node->coplanar_back)
        {
            std::swap(triangle.mPoints[0], triangle.mPoints[1]);
        }

        std::swap(current_node->front, current_node->back);

        current_node->plane.mData *= -1.f;

        to_visit.push(current_node->back);
        to_visit.push(current_node->front);
    }
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    if (this->tree.root == nullptr)
    {
        return;
    }

    this->tree.root->clip_to(tree->tree.root, epsilon);
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
    if (this->tree.root == nullptr)
    {
        return;
    }

    BspTree b{tree->Clone()};

    ClipTo(&b, epsilon);
    b.ClipTo(this, epsilon);

    b.Invert();
    b.ClipTo(this, epsilon);
    b.Invert();

    TriangleList result;
    AllTriangles(result);
    b.AllTriangles(result);

    Construct(result, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
    if (this->tree.root == nullptr)
    {
        return;
    }

    BspTree b{tree->Clone()};

    Invert();
    b.Invert();

    Union(&b, k, epsilon);
    Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
    if (this->tree.root == nullptr)
    {
        return;
    }

    BspTree b{tree->Clone()};
    b.Invert();

    Intersection(&b, k, epsilon);
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

        const bool should_draw{
        level == -1 ? true : current_node->depth == level
        };

        if (should_draw)
        {
            for (const auto& triangle : current_node->coplanar_front)
            {
                triangle.DebugDraw().Color(color).SetMaskBit(bitMask);
            }

            for (const auto& triangle : current_node->coplanar_back)
            {
                triangle.DebugDraw().Color(color).SetMaskBit(bitMask);
            }

            current_node->plane.DebugDraw(10.f).Color(
            Vector4{1.f, 0.f, 0.f, 0.f});
        }

        to_visit.push(current_node->back);
        to_visit.push(current_node->front);
    }
}

BspTree BspTree::Clone() const
{
    if (tree.root == nullptr)
    {
        return {};
    }

    BspTree output;
    output.tree.root = tree.root->clone(output.tree);

    return output;
}

BspTree::Node& BspTree::Tree::create_node()
{
    auto node{std::make_unique<Node>()};

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

    const size_t plane_index{S_PickSplitPlane(triangles, k, epsilon)};
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

TriangleList BspTree::Node::get_triangles() const
{
    TriangleList output{coplanar_front};
    output.insert(output.end(), coplanar_back.begin(), coplanar_back.end());
    return output;
}

BspTree::Node* BspTree::Node::clone(Tree& other_tree) const
{
    Node& output{other_tree.create_node()};

    output.plane = plane;
    output.coplanar_front = coplanar_front;
    output.coplanar_back = coplanar_back;

    if (front)
    {
        output.front = front->clone(other_tree);
    }

    if (back)
    {
        output.back = back->clone(other_tree);
    }

    return &output;
}

bool BspTree::Node::ray_cast(
const Ray& ray, const float t_min, const float t_max, float& t_out,
const float plane_epsilon, const float triangle_epsilon,
const int debugging_index) const
{
    const auto check_all{[&](const float min, const float max) -> bool
    {
        bool hit{false};
        const TriangleList triangles{get_triangles()};

        for (const Triangle& triangle : triangles)
        {
            float t_hit{0.f};
            const bool hit_triangle{
            RayTriangle(ray.mStart, ray.mDirection,
                        triangle.mPoints[0],
                        triangle.mPoints[1],
                        triangle.mPoints[2],
                        t_hit,
                        triangle_epsilon
            )};

            if (hit_triangle && Math::InRange(t_hit, min, max) && t_hit < t_out)
            {
                hit = true;
                t_out = t_hit;
            }
        }

        return hit;
    }};

    const auto visit_side{
    [&](const Node* side, const float min, const float max) -> bool
    {
        if (side == nullptr)
        {
            return {};
        }
        return side->ray_cast(ray, min, max, t_out, plane_epsilon,
                              triangle_epsilon,
                              debugging_index);
    }};

    const IntersectionType::Type start_side{
    PointPlane(ray.mStart, plane.mData, plane_epsilon)};

    if (start_side == IntersectionType::Coplanar)
    {
        bool hit{false};
        hit |= check_all(t_min, t_max);
        hit |= visit_side(front, t_min, t_max);
        hit |= visit_side(back, t_min, t_max);
        return hit;
    }

    const Node* near_side
    {start_side == IntersectionType::Inside ? front : back};
    const Node* far_side{near_side == front ? back : front};

    float t_plane{0.f};
    const bool hit_plane{
    utils::plane_ray(plane, ray, t_plane, triangle_epsilon)};

    if (!hit_plane)
    {
        return visit_side(near_side, t_min, t_max);
    }

    if (t_plane < t_min)
    {
        return visit_side(far_side, t_min, t_max);
    }

    if (t_plane > t_max)
    {
        return visit_side(near_side, t_min, t_max);
    }

    const float padding{
    Math::Abs(plane_epsilon / ray.mDirection.Dot(plane.GetNormal()))};

    bool hit_final{false};
    hit_final |= visit_side(near_side, t_min, t_plane + padding);
    hit_final |= check_all(t_plane - padding, t_plane + padding);
    hit_final |= visit_side(far_side, t_plane - padding, t_max);

    return hit_final;
}

void BspTree::Node::clip_to(const Node* other, float epsilon)
{
    if (other == nullptr)
    {
        return;
    }

    coplanar_front = other->clip_triangles(coplanar_front, epsilon);
    coplanar_back = other->clip_triangles(coplanar_back, epsilon);

    if (front)
    {
        front->clip_to(other, epsilon);
    }

    if (back)
    {
        back->clip_to(other, epsilon);
    }
}

TriangleList BspTree::Node::clip_triangles(const TriangleList& triangles,
                                           const float epsilon) const
{
    if (triangles.empty())
    {
        return {};
    }

    TriangleList front_triangles;
    TriangleList back_triangles;

    for (const Triangle& triangle : triangles)
    {
        SplitTriangle(
        plane, triangle,
        front_triangles, back_triangles,
        front_triangles, back_triangles, epsilon);
    }

    if (front)
    {
        front_triangles = front->clip_triangles(front_triangles, epsilon);
    }

    if (back)
    {
        back_triangles = back->clip_triangles(back_triangles, epsilon);
    }
    else
    {
        back_triangles.clear();
    }

    front_triangles.insert(
    front_triangles.end(),
    back_triangles.begin(),
    back_triangles.end());
    return front_triangles;

}
