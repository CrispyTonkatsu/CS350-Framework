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

DynamicAabbTree::~DynamicAabbTree()
{
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key,
                                 SpatialPartitionData& data)
{
    Warn("Assignment3: Required function un-implemented");

    const Vector3 center{data.mAabb.GetCenter()};
    const Vector3 padded_extents{data.mAabb.GetHalfSize() * mFatteningFactor};

    Node& node{
    tree.create_node(
    data.mClientData,
    Aabb::BuildFromCenterAndHalfExtents(center, padded_extents)),
    };

    key.mVoidKey = &node;

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

        results.emplace_back(
        SpatialPartitionQueryData({current_node->get_data(),
                                   current_node->get_bounds()}));

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

void DynamicAabbTree::Tree::set_root(Node* new_root)
{
    root = new_root;
}

DynamicAabbTree::Node* DynamicAabbTree::Tree::get_root() const
{
    return root;
}

DynamicAabbTree::Node& DynamicAabbTree::Tree::create_node(void* data,
    const Aabb& bounds)
{
    auto to_insert{std::make_unique<Node>()};
    to_insert->set_data(data);
    to_insert->set_tree(this);
    to_insert->set_bounds(bounds);

    const auto insert_location
    {nodes.insert(std::move(to_insert))};
    return *insert_location.first->get();
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
        current_node->select_path(current_node->get_left(),
                                  current_node->get_right());
    }

    current_node->split(node);

    update_nodes(node);
}

void DynamicAabbTree::Tree::update_nodes(Node& node)
{

    for (Node* current_node{&node}; !current_node->is_root();
         current_node = current_node->get_parent())
    {
        current_node->refit_bounds();

        Node::RotationData rotation{
        current_node->should_rotate(),
        };

        // TODO: Do the rotation
        if (!rotation.is_valid)
        {
            continue;
        }

        current_node->rotate(*rotation.small_child, *rotation.pivot);

        // TODO: Consider refitting the stuff again if a rotation does occur
        current_node->refit_bounds();
    }
}

void* DynamicAabbTree::Node::get_data() const
{
    return data;
}

Aabb DynamicAabbTree::Node::get_bounds() const
{
    return bounds;
}

void DynamicAabbTree::Node::set_data(void* new_data) { data = new_data; }

void DynamicAabbTree::Node::set_tree(Tree* new_tree) { tree = new_tree; }

void DynamicAabbTree::Node::set_bounds(const Aabb& new_bounds)
{
    bounds = new_bounds;
}

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

DynamicAabbTree::Node* DynamicAabbTree::Node::replace_child(Node& to_replace,
    Node* replacement)
{
    Node* output{replacement ? replacement->parent : nullptr};

    to_replace.parent = nullptr;

    if (replacement)
    {
        if (replacement == tree->get_root())
        {
            tree->set_root(this);
        }
        replacement->parent = this;
    }

    if (left == &to_replace)
    {
        left = replacement;
    }

    if (right == &to_replace)
    {
        right = replacement;
    }

    return output;
}

bool DynamicAabbTree::Node::is_root() const { return parent == nullptr; }

bool DynamicAabbTree::Node::is_leaf() const { return !left && !right; }

DynamicAabbTree::Node& DynamicAabbTree::Node::split(Node& other)
{
    Node& new_node{
    tree->create_node(nullptr, Aabb::Combine(bounds, other.bounds)),
    };

    if (parent != nullptr)
    {
        parent->replace_child(*this, &new_node);
    }

    parent = &new_node;
    other.parent = &new_node;

    new_node.left = this;
    new_node.right = &other;

    if (tree->get_root() == this)
    {
        tree->set_root(&new_node);
    }

    return new_node;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::select_path(const Node* left_path,
    const Node* right_path) const
{
    // TODO: Add the cases where the left or right are null

    const float diff_left{get_possible_cost(*left_path)};
    const float diff_right{get_possible_cost(*right_path)};

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
    if (is_leaf())
    {
        return;
    }

    // TODO: Add the cases where the left or right are null
    bounds = Aabb::Combine(left->bounds, right->bounds);
}

DynamicAabbTree::Node::RotationData DynamicAabbTree::Node::should_rotate() const
{
    if (is_leaf())
    {
        return {};
    }

    std::array<RotationData, 5> possible_rotations{
    {
    // Possible rotations with their associated costs

    {rotation_cost_delta(left->left, left->right, right),
     left->right, left, true},

    {rotation_cost_delta(left->right, left->left, right),
     left->left, left, true},

    {rotation_cost_delta(right->left, right->right, left),
     right->right, right, true},

    {rotation_cost_delta(right->right, right->left, left),
     right->left, right, true},

    // The element for no good rotations the change in cost has to be negative
    // to make it a better state for the tree

    {},
    },
    };

    return {
    *std::min_element( //
    possible_rotations.begin(), possible_rotations.end(),
    [](const RotationData& a, const RotationData& b)
    {
        return a.cost_delta < b.cost_delta;
    }),
    };
}

float DynamicAabbTree::Node::rotation_cost_delta(
const Node* big_child, const Node* small_child,
const Node* sibling) const
{
    if (!big_child || !small_child || !sibling)
    {
        // Any number larger than 0.f should do
        return 1.f;
    }

    return (big_child->get_current_cost() + small_child->
        get_possible_cost(*sibling)) -
    get_current_cost();
}

// NOTE: This is from the perspective of the old parent
void DynamicAabbTree::Node::rotate(Node& small_child, Node& pivot)
{
    // TODO: Left off here:
    // fixing the rotations so they exactly match the slides
    // it seems to be an issue we have in the way we are getting 
    // the points of rotation in the should_rotate() function

    // TODO: Add cases where the child is nullptr (this should never happen at
    // least to simple intuition but we'll see)

    parent->replace_child(*this, &pivot);
    pivot.replace_child(small_child, this);
    replace_child(pivot, &small_child);
}
