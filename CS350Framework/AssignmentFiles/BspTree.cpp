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
    tree.construct(nullptr, triangles, k, epsilon);
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

        if (node->right)
        {
            to_visit.push(node->right);
        }

        if (node->left)
        {
            to_visit.push(node->left);
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

        std::swap(node->left, node->right);
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

        if (node->right)
        {
            to_visit.push(node->right);
        }

        if (node->left)
        {
            to_visit.push(node->left);
        }
    }
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

        if (current_node->right)
        {
            to_visit.emplace_back(current_node->right);
        }

        if (current_node->left)
        {
            to_visit.emplace_back(current_node->left);
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

        if (node->right)
        {
            to_visit.push(node->right);
        }

        if (node->left)
        {
            to_visit.push(node->left);
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

    node.left = construct(&node, front, k, epsilon);
    node.right = construct(&node, back, k, epsilon);

    return &node;
}

bool BspTree::Node::is_root() const { return tree->get_root() == this; }

TriangleList BspTree::Node::get_triangles() const
{
    // TODO: Make this faster so that the function isn't lagging so bad

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
