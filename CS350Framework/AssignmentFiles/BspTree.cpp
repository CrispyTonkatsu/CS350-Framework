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

BspTreeQueryData::BspTreeQueryData() { mDepth = 0; }

void BspTree::SplitTriangle(
const Plane& plane, const Triangle& tri,
TriangleList& coplanarFront,
TriangleList& coplanarBack, TriangleList& front,
TriangleList& back, float epsilon)
{
    const IntersectionType::Type intersection_type{PlaneTriangle(
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
                float t;
                RayPlane(ray.mStart, ray.mDirection, plane.mData, t, epsilon);
                const Vector3 intersection{ray.GetPoint(t)};

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
                float t;
                RayPlane(ray.mStart, ray.mDirection, plane.mData, t, epsilon);
                const Vector3 intersection{ray.GetPoint(t)};

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
    const auto is_degenerate{
    [](const Triangle& triangle, const float epsilon)
    {
        return Math::Cross(triangle.mPoints[1] - triangle.mPoints[0],
                           triangle.mPoints[2] - triangle.mPoints[0])
        .Length() < epsilon;
    }};

    const Triangle& triangle_to_test{triangles.at(testIndex)};
    if (is_degenerate(triangle_to_test, epsilon))
    {
        return Math::PositiveMax();
    }

    Plane plane;
    plane.Set(triangle_to_test.mPoints[0], triangle_to_test.mPoints[1],
              triangle_to_test.mPoints[2]);

    float front_count{0};
    float overlap_count{0};
    float back_count{0};

    for (const Triangle& triangle : triangles)
    {
        const IntersectionType::Type intersection{
        PlaneTriangle(plane.mData, triangle.mPoints[0], triangle.mPoints[1],
                      triangle.mPoints[2], epsilon)};

        switch (intersection) // NOLINT
        {
        case IntersectionType::Inside:
            front_count++;
            break;
        case IntersectionType::Overlaps:
            overlap_count++;
            break;
        case IntersectionType::Outside:
            back_count++;
            break;
        }
    }

    return k * overlap_count + (1 - k) * Math::Abs(front_count - back_count);
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k,
                               float epsilon)
{
    size_t min_i{0};
    float min_cost{Math::PositiveMax()};
    for (size_t i{0}; i < triangles.size(); i++)
    {
        const float cost{CalculateScore(triangles, i, k, epsilon)};
        if (cost < min_cost)
        {
            min_i = i;
            min_cost = cost;
        }
    }

    return min_i;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
    // TODO: write a note explaining that the results provided in the master 
    // results file is just weird and have compared with team mates to 
    // try and find a difference

    tree.nodes.clear();
    tree.set_root(nullptr);
    tree.construct(nullptr, triangles, k, epsilon);
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon,
                      float triExpansionEpsilon, int debuggingIndex)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");

    return tree.get_root()->ray_cast(
    ray,
    0, Math::PositiveMax(),
    t,
    planeEpsilon,
    triExpansionEpsilon);
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
    const Node* root{tree.get_root()};
    if (root == nullptr)
    {
        return;
    }

    std::queue<const Node*> to_visit{};
    to_visit.push(root);

    while (!to_visit.empty())
    {
        const Node* node{to_visit.front()};
        to_visit.pop();

        triangles.insert(triangles.end(), node->coplanar_front.begin(),
                         node->coplanar_front.end());
        triangles.insert(triangles.end(), node->coplanar_back.begin(),
                         node->coplanar_back.end());

        if (node->back)
        {
            to_visit.push(node->back);
        }

        if (node->front)
        {
            to_visit.push(node->front);
        }
    }
}

void BspTree::Invert()
{
    Node* root{tree.get_root()};
    if (root == nullptr)
    {
        return;
    }

    std::queue<Node*> to_visit{};
    to_visit.push(root);

    while (!to_visit.empty())
    {
        Node* node{to_visit.front()};
        to_visit.pop();

        std::swap(node->front, node->back);
        node->plane.mData *= -1.f;

        std::reverse(node->coplanar_front.begin(), node->coplanar_front.end());
        std::reverse(node->coplanar_back.begin(), node->coplanar_back.end());

        for (Triangle& triangle : node->coplanar_front)
        {
            std::swap(triangle.mPoints[0], triangle.mPoints[1]);
        }

        for (Triangle& triangle : node->coplanar_back)
        {
            std::swap(triangle.mPoints[0], triangle.mPoints[1]);
        }

        if (node->back)
        {
            to_visit.push(node->back);
        }

        if (node->front)
        {
            to_visit.push(node->front);
        }
    }
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    this->tree.clip_against(tree->tree.get_root(), epsilon);
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");

    ClipTo(tree, epsilon);
    tree->ClipTo(this, epsilon);

    tree->Invert();
    tree->ClipTo(this, epsilon);
    tree->Invert();

    TriangleList this_triangles{};
    AllTriangles(this_triangles);

    TriangleList other_triangles{};
    tree->AllTriangles(other_triangles);

    this_triangles.insert(this_triangles.end(), other_triangles.begin(),
                          other_triangles.end());

    Construct(this_triangles, k, epsilon);
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
    const Node* root{tree.get_root()};

    if (root == nullptr)
    {
        return;
    }

    std::vector<const Node*> to_visit{};
    to_visit.emplace_back(root);

    while (!to_visit.empty())
    {
        const Node* current_node{to_visit.back()};
        to_visit.pop_back();

        BspTreeQueryData data;
        data.mTriangles = current_node->get_triangles();
        data.mDepth = current_node->get_depth();

        results.emplace_back(data);

        if (current_node->back)
        {
            to_visit.emplace_back(current_node->back);
        }

        if (current_node->front)
        {
            to_visit.emplace_back(current_node->front);
        }
    }
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
    const Node* root{tree.get_root()};
    if (root == nullptr)
    {
        return;
    }

    std::queue<const Node*> to_visit{};
    to_visit.push(root);

    while (!to_visit.empty())
    {
        const Node* node{to_visit.front()};
        to_visit.pop();

        const int node_level{node->get_depth()};
        const bool should_draw{level == -1 ? true : node_level == level};

        if (should_draw)
        {
            for (auto& triangle : node->get_triangles())
            {
                triangle.DebugDraw().Color(color).SetMaskBit(bitMask);
            }
        }

        if (node_level == level)
        {
            continue;
        }

        if (node->back)
        {
            to_visit.push(node->back);
        }

        if (node->front)
        {
            to_visit.push(node->front);
        }
    }
}

void BspTree::Tree::set_owner(BspTree* new_owner) { owner = new_owner; }

BspTree* BspTree::Tree::get_owner() const { return owner; }

void BspTree::Tree::set_root(Node* new_root) { root = new_root; }

BspTree::Node* BspTree::Tree::get_root() const { return root; }

BspTree::Node& BspTree::Tree::create_node()
{
    auto node{std::make_unique<Node>()};
    node->tree = this;

    const auto insert_location{nodes.insert({node.get(), std::move(node)})};

    return *insert_location.first->second;
}

BspTree::Node* BspTree::Tree::construct(
Node* parent, const TriangleList& triangles, float k,
float epsilon)
{
    if (triangles.empty())
    {
        return nullptr;
    }

    Node& node{create_node()};
    const size_t split_index{owner->PickSplitPlane(triangles, k, epsilon)};
    const Triangle& split_triangle{triangles[split_index]};

    if (parent == nullptr)
    {
        root = &node;
    }
    else
    {
        node.parent = parent;
    }

    node.plane.Set(
    split_triangle.mPoints[0],
    split_triangle.mPoints[1],
    split_triangle.mPoints[2]);

    TriangleList front;
    TriangleList back;

    for (const Triangle& triangle : triangles)
    {
        SplitTriangle(node.plane, triangle, node.coplanar_front,
                      node.coplanar_back, front, back, epsilon);
    }

    node.front = construct(&node, front, k, epsilon);
    node.back = construct(&node, back, k, epsilon);

    return &node;
}

void BspTree::Tree::clip_against(Node* node, float epsilon)
{
    if (node == nullptr)
    {
        return;
    }

    node->coplanar_back = root->clip_triangles(node->coplanar_back, epsilon);
    node->coplanar_front = root->clip_triangles(node->coplanar_front, epsilon);

    clip_against(node->front, epsilon);
    clip_against(node->back, epsilon);
}

bool BspTree::Node::is_root() const { return tree->get_root() == this; }

TriangleList BspTree::Node::get_triangles() const
{
    // TODO: Make this faster so that the function isn't lagging so bad 
    // when we need it later.

    TriangleList output;
    output.reserve(coplanar_back.size() + coplanar_front.size());

    for (const Triangle& triangle : coplanar_front)
    {
        output.push_back(triangle);
    }

    for (const Triangle& triangle : coplanar_back)
    {
        output.push_back(triangle);
    }

    return output;
}

int BspTree::Node::get_depth() const
{
    int depth{0};

    auto current_node{this};

    while (!current_node->is_root())
    {
        depth++;
        current_node = current_node->parent;
    }

    return depth;
}

bool BspTree::Node::ray_cast(
const Ray& ray,
float t_min, float t_max, float& t_out,
float plane_epsilon, float triangle_epsilon) const
{
    const float point_side_thin{
    Vector4(ray.mStart.x, ray.mStart.y, ray.mStart.z, 1.f).Dot(plane.mData)};
    
    const IntersectionType::Type point_side{
    PointPlane(ray.mStart, plane.mData, plane_epsilon)};

    const Node* near_side{point_side_thin >= 0 ? back : front};
    const Node* far_side{near_side == front ? back : front};

    float t_plane;
    const bool hit_plane{
    RayPlane(ray.mStart, ray.mDirection, plane.mData, t_plane, plane_epsilon)};

    if (point_side == IntersectionType::Coplanar)
    {
        // TODO: Get the smallest return value out of this the triangle checks

        float t_near;
        const bool hit_near{near_side &&
        near_side->ray_cast(ray, t_min, t_max, t_near, plane_epsilon,
                            triangle_epsilon)};

        if (hit_near)
        {
            t_out = t_near;
            return true;
        }

        float t_far;
        const bool hit_far{far_side &&
        far_side->ray_cast(ray, t_min, t_max, t_far, plane_epsilon,
                           triangle_epsilon)};

        if (hit_far)
        {
            t_out = t_far;
            return true;
        }

        float t_triangles;
        const bool hit_triangles{
        ray_triangles(ray, t_triangles, triangle_epsilon)};

        if (hit_triangles)
        {
            t_out = t_triangles;
            return true;
        }
    }

    if (!hit_plane)
    {
        float t_near;
        const bool hit_near{near_side &&
        near_side->ray_cast(ray, t_min, t_max, t_near, plane_epsilon,
                            triangle_epsilon)};

        if (hit_near)
        {
            t_out = t_near;
            return true;
        }
    }

    if (hit_plane)
    {
        float t_near;
        const bool hit_near{near_side &&
        near_side->ray_cast(ray, t_min, t_plane, t_near, plane_epsilon,
                            triangle_epsilon)};

        if (hit_near)
        {
            t_out = t_near;
            return true;
        }

        float t_far;
        const bool hit_far{far_side &&
        far_side->ray_cast(ray, t_plane, t_max, t_far, plane_epsilon,
                           triangle_epsilon)};

        if (hit_far)
        {
            t_out = t_far;
            return true;
        }

        float t_triangles;
        const bool hit_triangles{
        ray_triangles(ray, t_triangles, triangle_epsilon)};

        if (hit_triangles)
        {
            t_out = t_triangles;
            return true;
        }
    }

    if (t_plane < 0 || t_plane > t_max)
    {
        float t_near;
        const bool hit_near{near_side &&
        near_side->ray_cast(ray, t_min, t_max, t_near, plane_epsilon,
                            triangle_epsilon)};

        if (hit_near)
        {
            t_out = t_near;
            return true;
        }
    }

    if (t_plane > 0 && t_plane < t_min)
    {
        float t_far;
        const bool hit_far{far_side &&
        far_side->ray_cast(ray, t_min, t_max, t_far, plane_epsilon,
                           triangle_epsilon)};

        if (hit_far)
        {
            t_out = t_far;
            return true;
        }
    }

    // TODO: Handle edge-case 2
    // TODO: Handle edge-case 3

    return false;
}

bool BspTree::Node::ray_triangles(const Ray& ray, float& t_out,
                                  float triangle_epsilon) const
{
    bool hit{false};
    float t_min{Math::PositiveMax()};

    for (const Triangle& triangle : coplanar_front)
    {
        float t_triangle;
        const bool hit_triangle{
        RayTriangle(
        ray.mStart, ray.mDirection,
        triangle.mPoints[0], triangle.mPoints[1], triangle.mPoints[2],
        t_triangle, triangle_epsilon)};

        if (hit_triangle && t_triangle < t_min)
        {
            t_min = t_triangle;
            hit = true;
        }
    }

    for (const Triangle& triangle : coplanar_back)
    {
        float t_triangle;
        const bool hit_triangle{
        RayTriangle(
        ray.mStart, ray.mDirection,
        triangle.mPoints[0], triangle.mPoints[1], triangle.mPoints[2],
        t_triangle, triangle_epsilon)};

        if (hit_triangle && t_triangle < t_min)
        {
            t_min = t_triangle;
            hit = true;
        }
    }

    t_out = t_min;
    return hit;
}

TriangleList BspTree::Node::clip_triangles(const TriangleList& triangles,
                                           float epsilon)
{
    std::vector<Triangle> front_triangles;
    std::vector<Triangle> back_triangles;

    for (const Triangle& triangle : triangles)
    {
        SplitTriangle(plane, triangle, front_triangles, back_triangles,
                      front_triangles, back_triangles, epsilon);
    }

    if (front)
    {
        front->clip_triangles(front_triangles, epsilon);
    }

    if (back)
    {
        back->clip_triangles(back_triangles, epsilon);
    }
    else
    {
        back_triangles.clear();
    }

    front_triangles.insert(front_triangles.end(), back_triangles.begin(),
                           back_triangles.end());
    return front_triangles;
}
