///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Precompiled.hpp"

#include <algorithm>
#include <array>
#include "DynamicAabbTree.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree() { mType = SpatialPartitionTypes::AabbTree; }

DynamicAabbTree::~DynamicAabbTree() {}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key,
                                 SpatialPartitionData& data)
{
    Warn("Assignment3: Required function un-implemented");

    key.mVoidKey = data.mClientData;

    const Vector3 center{data.mAabb.GetCenter()};
    const Vector3 padded_extents{data.mAabb.GetHalfSize() * mFatteningFactor};

    Node& node{
    tree.create_node(
    key.mVoidKey, data.mClientData,
    Aabb::BuildFromCenterAndHalfExtents(center, padded_extents)),
    };

    tree.insert_node(node);
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key,
                                 SpatialPartitionData& data)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::DebugDraw(int level, const Matrix4& transform,
                                const Vector4& color, int bitMask)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
    Warn("Assignment3: Required function un-implemented");
}

void DynamicAabbTree::FilloutData(
std::vector<SpatialPartitionQueryData>& results) const
{
    Warn("Assignment3: Required function un-implemented");
}

DynamicAabbTree::Node& DynamicAabbTree::Tree::create_node(void* key, void* data,
                                                          Aabb bounds)
{
    Node to_insert{};
    to_insert.set_data(data);
    to_insert.set_tree(this);

    auto insert_location{nodes.insert({key, std::move(to_insert)})};
    return insert_location.first->second;
}

void DynamicAabbTree::Tree::insert_node(Node& node)
{
    if (root == nullptr)
    {
        root = &node;
        return;
    }

    Node* current_node{root};
    while (!current_node->is_leaf())
    {
        current_node =
        node.select_path(current_node->get_left(), current_node->get_right());
    }

    current_node->split(node);

    update_nodes(node);
}

void DynamicAabbTree::Tree::update_nodes(Node& node)
{
    Node* current_node{&node};

    while (!current_node->is_root())
    {
        // TODO: check whether it is ok to refit as we go up
        current_node->refit_bounds();

        DynamicAabbTree::Node::RotationType rotation_type{
        current_node->should_rotate(),
        };

        // TODO: Do the rotation
        // TODO: Consider refitting the stuff again if a rotation does occurr
        switch (rotation_type)
        {
        case Node::RotationType::LEFT_TO_RIGHTRIGHT:
            break;

        case Node::RotationType::LEFT_TO_RIGHTLEFT:
            break;

        case Node::RotationType::RIGHT_TO_LEFTLEFT:
            break;

        case Node::RotationType::RIGHT_TO_LEFTRIGHT:
            break;

        case Node::RotationType::NO_ROTATION:
            break;
        }

        current_node = current_node->get_parent();
    }
}

void DynamicAabbTree::Node::set_data(void* new_data) { data = new_data; }

void DynamicAabbTree::Node::set_tree(Tree* new_tree) { tree = new_tree; }

DynamicAabbTree::Node* DynamicAabbTree::Node::get_sibling() const
{
    if (this == parent->left)
    {
        return right;
    }

    if (this == parent->right)
    {
        return left;
    }

    return nullptr;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::replace_child(Node& child,
                                                            Node& replacement)
{
    Node* output{replacement.parent};

    replacement.parent = this;

    if (left == &child)
    {
        left = &replacement;
    }

    if (right == &child)
    {
        right = &replacement;
    }

    return output;
}

bool DynamicAabbTree::Node::is_root() const { return parent == nullptr; }

bool DynamicAabbTree::Node::is_leaf() const { return !left && !right; }

DynamicAabbTree::Node& DynamicAabbTree::Node::split(Node& other)
{
    Node& new_node{
    tree->create_node(nullptr, nullptr, Aabb::Combine(bounds, other.bounds)),
    };

    new_node.parent = parent;
    parent = &new_node;

    new_node.left = this;
    new_node.right = &other;

    return new_node;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::select_path(Node* left,
                                                          Node* right) const
{
    // TODO: Add the cases where the left or right are null

    const float diff_left{get_possible_cost(*left)};
    const float diff_right{get_possible_cost(*right)};

    if (diff_left < diff_right)
    {
        return left;
    }

    return right;
}

float DynamicAabbTree::Node::get_current_cost() const
{
    return bounds.GetSurfaceArea();
}

float DynamicAabbTree::Node::get_possible_cost(const Node& other) const
{
    const Aabb new_bounds{Aabb::Combine(bounds, other.bounds)};
    return new_bounds.GetSurfaceArea() - other.bounds.GetSurfaceArea();
}

DynamicAabbTree::Node* DynamicAabbTree::Node::get_parent() const
{
    return parent;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::get_left() const { return left; }

DynamicAabbTree::Node* DynamicAabbTree::Node::get_right() const
{
    return right;
}

void DynamicAabbTree::Node::refit_bounds()
{
    // TODO: Add the cases where the left or right are null
    bounds = Aabb::Combine(left->bounds, right->bounds);
}

DynamicAabbTree::Node::RotationType DynamicAabbTree::Node::should_rotate() const
{
    using CostSolution = std::pair<float, RotationType>;

    std::array<CostSolution, 5> possible_rotations{
    {
    // Possible rotations with their associated costs

    {rotation_cost_delta(*left->left, *left->right, *right),
     DynamicAabbTree::Node::RotationType::RIGHT_TO_LEFTLEFT},

    {rotation_cost_delta(*left->right, *left->left, *right),
     DynamicAabbTree::Node::RotationType::RIGHT_TO_LEFTRIGHT},

    {rotation_cost_delta(*right->left, *right->right, *left),
     RotationType::LEFT_TO_RIGHTLEFT},

    {rotation_cost_delta(*right->right, *right->left, *left),
     RotationType::LEFT_TO_RIGHTRIGHT},

    // The element for no good rotations the change in cost has to be negative
    // to make it a better state for the tree

    {0.f, RotationType::NO_ROTATION},
    },
    };

    const CostSolution& output{
    *std::min_element( //
    possible_rotations.begin(), possible_rotations.end(),
    [](const CostSolution& a, const CostSolution& b)
    { return a.first < b.first; }),
    };

    return output.second;
}

float DynamicAabbTree::Node::rotation_cost_delta(Node& to_rotate, Node& to_stay,
                                                 Node& sibling) const
{
    return (to_rotate.get_current_cost() + to_stay.get_possible_cost(sibling)) -
    get_current_cost();
}

void DynamicAabbTree::Node::rotate(Node& grandchild, Node& child)
{
    // TODO: Add cases where the sibling is nullptr (this should never happen at
    // least to simple intuition but we'll see)

    child.parent->replace_child(child, grandchild);
    grandchild.parent->replace_child(grandchild, child);
}
