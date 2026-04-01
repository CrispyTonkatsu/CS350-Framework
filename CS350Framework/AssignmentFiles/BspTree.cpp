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

        // TODO: Factor this into the ones that need it
        float t;
        RayPlane(ray.mStart, ray.mDirection, plane.mData, t, epsilon);
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
    const auto is_degenerate{[epsilon](const Triangle& triangle)
    {
        return (triangle.mPoints[1] - triangle.mPoints[0]).Cross(
        triangle.mPoints[2] - triangle.mPoints[0]).LengthSq() < epsilon;
    }};

    const Triangle& triangle_to_test{triangles.at(testIndex)};
    if (is_degenerate(triangle_to_test))
    {
        return Math::PositiveMax();
    }

    Plane plane;
    plane.Set(triangle_to_test.mPoints[0], triangle_to_test.mPoints[1],
              triangle_to_test.mPoints[2]);

    if (plane.GetNormal().LengthSq() < epsilon * epsilon)
    {
        return Math::PositiveMax();
    }

    float front_count{0};
    float overlap_count{0};
    float back_count{0};
    for (const Triangle& triangle : triangles)
    {
        if (&triangle == &triangle_to_test)
        {
            continue;
        }

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
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");

    tree.k = k;
    tree.epsilon = epsilon;
    tree.set_owner(this);

    Node& node{tree.create_node(TriangleList(triangles))};
    tree.set_root(&node);

    std::vector<Node*> to_visit{};
    to_visit.emplace_back(tree.get_root());

    while (!to_visit.empty())
    {
        Node* current_node{to_visit.back()};
        to_visit.pop_back();

        if (current_node->get_triangles().size() == 1)
        {
            continue;
        }

        std::pair<Node*, Node*> children{current_node->split()};

        if (children.second != nullptr)
        {
            to_visit.push_back(children.second);
        }

        if (children.first != nullptr)
        {
            to_visit.push_back(children.first);
        }
    }
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

        if (current_node->get_right())
        {
            to_visit.emplace_back(current_node->get_right());
        }

        if (current_node->get_left())
        {
            to_visit.emplace_back(current_node->get_left());
        }
    }
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
    /******Student:Assignment4******/
    Warn("Assignment4: Required function un-implemented");

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
                triangle
                .DebugDraw()
                .Color(color)
                .SetMaskBit(bitMask);
            }

            // TODO: Ask how they want the debug draw
            node->get_plane()
                .DebugDraw(30.f)
                .Color(color)
                .SetMaskBit(bitMask);
        }

        if (node->is_leaf())
        {
            continue;
        }

        if (node_level == level)
        {
            continue;
        }

        if (node->get_right())
        {
            to_visit.push(node->get_right());
        }

        if (node->get_left())
        {
            to_visit.push(node->get_left());
        }
    }
}

void BspTree::Tree::set_owner(BspTree* new_owner)
{
    owner = new_owner;
}

BspTree* BspTree::Tree::get_owner() const
{
    return owner;
}

void BspTree::Tree::set_root(Node* new_root)
{
    root = new_root;
}

BspTree::Node* BspTree::Tree::get_root() const
{
    return root;
}

BspTree::Node& BspTree::Tree::create_node(TriangleList&& triangles)
{
    size_t plane_index{owner->PickSplitPlane(triangles, k, epsilon)};
    Triangle& triangle{triangles[plane_index]};

    Plane plane;
    plane.Set(triangle.mPoints[0], triangle.mPoints[1], triangle.mPoints[2]);

    auto node{std::make_unique<Node>()};
    node->set_tree(this);
    node->set_triangles(std::move(triangles));
    node->set_plane(plane);

    const auto insert_location
    {nodes.insert({node.get(), std::move(node)})};

    return *insert_location.first->second;
}

void BspTree::Node::set_tree(Tree* new_tree)
{
    tree = new_tree;
}

void BspTree::Node::set_parent(Node* new_parent)
{
    parent = new_parent;
}

const TriangleList& BspTree::Node::get_triangles() const
{
    return triangles;
}

void BspTree::Node::set_triangles(TriangleList&& new_triangles)
{
    triangles = std::move(new_triangles);
}

const Plane& BspTree::Node::get_plane() const
{
    return plane;
}

void BspTree::Node::set_plane(const Plane& new_plane)
{
    plane = new_plane;
}

bool BspTree::Node::is_root() const
{
    return tree->get_root() == this;
}

bool BspTree::Node::is_leaf() const
{
    return left == nullptr && right == nullptr;
}

std::pair<BspTree::Node*, BspTree::Node*> BspTree::Node::split()
{
    TriangleList negative;
    TriangleList coplanar;
    TriangleList positive;

    for (Triangle triangle : triangles)
    {
        IntersectionType::Type intersection_type{
        PlaneTriangle(plane.mData, triangle.mPoints[0],
                      triangle.mPoints[1],
                      triangle.mPoints[2],
                      tree->epsilon)};

        switch (intersection_type) // NOLINT
        {
        case IntersectionType::Coplanar:
            coplanar.push_back(triangle);
            break;
        case IntersectionType::Outside:
            negative.push_back(triangle);
            break;
        case IntersectionType::Inside:
            positive.push_back(triangle);
            break;
        case IntersectionType::Overlaps:
            // TODO: Left off here, I think the coplanar arrays
            // here are wrong or something
            SplitTriangle(plane, triangle, coplanar, coplanar, positive,
                          negative, tree->epsilon);
            break;
        }
    }

    const auto try_construct{[this](TriangleList&& triangles_to_use) -> Node*
    {
        if (triangles_to_use.empty())
        {
            return nullptr;
        }

        Node& new_node{tree->create_node(std::move(triangles_to_use))};
        new_node.set_parent(this);

        return &new_node;
    }};

    left = try_construct(std::move(positive));
    right = try_construct(std::move(negative));
    
    set_triangles(std::move(coplanar));

    return {left, right};
}

int BspTree::Node::get_depth() const
{
    int depth{0};

    const Node* current_node{this};

    while (!current_node->is_root())
    {
        depth++;
        current_node = current_node->get_parent();
    }

    return depth;
}

BspTree::Node* BspTree::Node::get_parent() const
{
    return parent;
}

BspTree::Node* BspTree::Node::get_left() const
{
    return left;
}

BspTree::Node* BspTree::Node::get_right() const
{
    return right;
}
