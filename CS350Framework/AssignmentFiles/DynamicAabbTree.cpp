///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Precompiled.hpp"

#include <algorithm>
#include "DynamicAabbTree.hpp"

#include <queue>

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree() { mType = SpatialPartitionTypes::AabbTree; }

DynamicAabbTree::~DynamicAabbTree()
{
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key,
                                 SpatialPartitionData& data)
{
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
    const auto search{tree.nodes.find(static_cast<Node*>(key.mVoidKey))};
    if (search == tree.nodes.end())
    {
        return;
    }

    Node& node{*search->second};

    const Aabb& tree_aabb{node.get_bounds()};
    const Aabb& contained_aabb{data.mAabb};

    if (tree_aabb.Contains(contained_aabb))
    {
        return;
    }

    tree.remove_node(node);

    const Vector3 center{data.mAabb.GetCenter()};
    const Vector3 padded_extents{data.mAabb.GetHalfSize() * mFatteningFactor};

    node.set_bounds(
    Aabb::BuildFromCenterAndHalfExtents(center, padded_extents));

    tree.insert_node(node);
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
    const auto search{tree.nodes.find(static_cast<Node*>(key.mVoidKey))};
    if (search == tree.nodes.end())
    {
        return;
    }

    Node& node{*search->second};

    tree.remove_node(node);
    tree.delete_node(node);
}

void DynamicAabbTree::DebugDraw(int level, const Matrix4& transform,
                                const Vector4& color, int bitMask)
{
    if (tree.get_root() == nullptr)
    {
        return;
    }

    std::queue<const Node*> to_visit{};
    to_visit.push(tree.get_root());

    while (!to_visit.empty())
    {
        const Node* node{to_visit.front()};
        to_visit.pop();

        const int node_level{node->get_depth()};
        const bool should_draw{level == -1 ? true : node_level == level};

        if (should_draw)
        {
            node->get_bounds()
                .DebugDraw()
                .Color(color)
                .SetTransform(transform).
                SetMaskBit(bitMask);
        }

        if (node->is_leaf())
        {
            continue;
        }

        if (node_level == level)
        {
            continue;
        }

        to_visit.push(node->get_left());
        to_visit.push(node->get_right());
    }
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
    std::vector<const Node*> to_visit{};
    to_visit.emplace_back(tree.get_root());

    while (!to_visit.empty())
    {
        const Node* current_node{to_visit.back()};
        to_visit.pop_back();

        const Aabb aabb{current_node->get_bounds()};

        float t{0.f};
        if (!RayAabb(ray.mStart, ray.mDirection, aabb.mMin, aabb.mMax, t))
        {
            continue;
        }

        if (current_node->is_leaf())
        {
            results.AddResult(CastResult{current_node->get_data(), t});
            continue;
        }

        to_visit.push_back(current_node->get_left());
        to_visit.push_back(current_node->get_right());
    }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum,
                                  CastResults& results)
{
    std::vector<Node*> to_visit{};
    to_visit.emplace_back(tree.get_root());

    while (!to_visit.empty())
    {
        Node* current_node{to_visit.back()};
        to_visit.pop_back();

        const Aabb aabb{current_node->get_bounds()};

        IntersectionType::Type cast_result{FrustumAabb(
        frustum.GetPlanes(), aabb.mMin, aabb.mMax,
        current_node->get_last_axis())};

        if (cast_result == IntersectionType::Outside)
        {
            // There is no need to keep checking for outliers
            continue;
        }

        if (cast_result == IntersectionType::Inside)
        {
            // We know that everything inside of this is inside so it is reported
            const std::vector<Node*> leaves{current_node->get_leaves()};
            for (const Node* node : leaves)
            {
                results.AddResult(CastResult{node->get_data(), 0.f});
            }

            continue;
        }

        if (current_node->is_leaf())
        {
            results.AddResult(CastResult{current_node->get_data(), 0.f});
            continue;
        }

        to_visit.push_back(current_node->get_left());
        to_visit.push_back(current_node->get_right());
    }
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
    std::vector<std::pair<Node*, Node*>> to_visit{};
    const Node* root{tree.get_root()};
    to_visit.emplace_back(root->get_left(), root->get_right());

    while (!to_visit.empty())
    {
        const std::pair<Node*, Node*> pair{to_visit.back()};
        to_visit.pop_back();

        Node* a{pair.first};
        Node* b{pair.second};

        if (!a->is_leaf())
        {
            to_visit.emplace_back(a->get_left(), a->get_right());
        }

        if (!b->is_leaf())
        {
            to_visit.emplace_back(b->get_left(), b->get_right());
        }

        Aabb a_bounds{a->get_bounds()};
        Aabb b_bounds{b->get_bounds()};

        const bool are_overlapping{AabbAabb(a_bounds.mMin, a_bounds.mMax,
                                            b_bounds.mMin, b_bounds.mMax)};

        if (!are_overlapping)
        {
            continue;
        }

        if (a->is_leaf() && b->is_leaf())
        {
            results.AddResult(QueryResult{a->get_data(), b->get_data()});
            continue;
        }

        if (!a->is_leaf() && !b->is_leaf())
        {

            if (a_bounds.GetVolume() < b_bounds.GetVolume())
            {
                to_visit.emplace_back(a, b->get_left());
                to_visit.emplace_back(a, b->get_right());
            }
            else
            {
                to_visit.emplace_back(b, a->get_left());
                to_visit.emplace_back(b, a->get_right());
            }

            continue;
        }

        Node* leaf{a->is_leaf() ? a : b};
        const Node* internal{a->is_leaf() ? b : a};

        to_visit.emplace_back(leaf, internal->get_left());
        to_visit.emplace_back(leaf, internal->get_right());
    }
}

void DynamicAabbTree::FilloutData(
std::vector<SpatialPartitionQueryData>& results) const
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

        void* data_to_print{current_node->get_data()};

        SpatialPartitionQueryData data(
        {data_to_print, current_node->get_bounds()});
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
    {nodes.insert({to_insert.get(), std::move(to_insert)})};

    return *insert_location.first->second;
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
        node.select_path(current_node->get_left(),
                         current_node->get_right());
    }

    current_node->split(node);

    update_nodes(node);
}

void DynamicAabbTree::Tree::remove_node(Node& node)
{
    if (node.is_root())
    {
        root = nullptr;
        return;
    }

    Node* parent{node.get_parent()};
    Node* sibling{node.get_sibling()};

    if (parent->is_root())
    {
        set_root(sibling);
        sibling->set_parent(nullptr);
    }
    else
    {
        parent->get_parent()->replace_child(*parent, node.get_sibling());
    }

    delete_node(*parent);

    update_nodes(*sibling);
}

void DynamicAabbTree::Tree::delete_node(Node& node)
{
    nodes.erase(&node);
}

void DynamicAabbTree::Tree::update_nodes(Node& node)
{
    for (Node* current_node{node.get_parent()};
         current_node != nullptr;
         current_node = current_node->get_parent())
    {
        current_node->refit_bounds();
        current_node->update_height();

        const Node::RotationData rotation{current_node->should_rotate()};

        if (!rotation.is_valid)
        {
            continue;
        }

        current_node->rotate(*rotation.big_child, *rotation.pivot);
    }
}

std::vector<DynamicAabbTree::Node*> DynamicAabbTree::Node::get_leaves()
{
    std::vector<Node*> output;

    std::vector<Node*> to_visit{};
    to_visit.emplace_back(this);

    while (!to_visit.empty())
    {
        Node* current_node{to_visit.back()};
        to_visit.pop_back();

        if (current_node->is_leaf())
        {
            output.push_back(current_node);
            continue;
        }

        to_visit.push_back(current_node->left);
        to_visit.push_back(current_node->right);
    }

    return output;
}

void DynamicAabbTree::Node::set_last_axis(size_t new_last_axis)
{
    last_axis = new_last_axis;
}

size_t& DynamicAabbTree::Node::get_last_axis()
{
    return last_axis;
}

void* DynamicAabbTree::Node::get_data() const
{
    return data;
}

Aabb DynamicAabbTree::Node::get_bounds() const
{
    return bounds;
}

void DynamicAabbTree::Node::set_data(void* new_data)
{
    data = new_data;
}

void DynamicAabbTree::Node::set_tree(Tree* new_tree)
{
    tree = new_tree;
}

void DynamicAabbTree::Node::set_parent(Node* new_parent)
{
    parent = new_parent;
}

void DynamicAabbTree::Node::set_bounds(const Aabb& new_bounds)
{
    bounds = new_bounds;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::get_sibling() const
{
    if (this == parent->left)
    {
        return parent->right;
    }

    if (this == parent->right)
    {
        return parent->left;
    }

    return nullptr;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::replace_child(
Node& to_replace,
Node* replacement)
{
    Node* output{replacement ? replacement->parent : nullptr};

    if (replacement)
    {
        if (replacement->is_root())
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

bool DynamicAabbTree::Node::is_root() const
{
    return this == tree->get_root();
}

bool DynamicAabbTree::Node::is_leaf() const
{
    return !left && !right;
}

DynamicAabbTree::Node& DynamicAabbTree::Node::split(Node& other)
{
    Node& new_node{
    tree->create_node(nullptr, Aabb::Combine(bounds, other.bounds)),
    };

    if (is_root())
    {
        tree->set_root(&new_node);
    }
    else
    {
        parent->replace_child(*this, &new_node);
    }

    parent = &new_node;
    other.parent = &new_node;

    new_node.left = this;
    new_node.right = &other;

    return new_node;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::select_path(Node* left_path,
    Node* right_path) const
{
    const float diff_left{get_possible_cost_delta(*left_path)};
    const float diff_right{get_possible_cost_delta(*right_path)};

    if (diff_left < diff_right)
    {
        return left_path;
    }

    return right_path;
}

float DynamicAabbTree::Node::get_current_cost() const
{
    return bounds.GetSurfaceArea();
}

float DynamicAabbTree::Node::get_possible_cost(const Node& other) const
{
    const Aabb new_bounds{Aabb::Combine(bounds, other.bounds)};
    return new_bounds.GetSurfaceArea();
}

float DynamicAabbTree::Node::get_possible_cost_delta(
const Node& other) const
{
    return get_possible_cost(other) - other.bounds.GetSurfaceArea();
}

DynamicAabbTree::Node* DynamicAabbTree::Node::get_parent() const
{
    return parent;
}

DynamicAabbTree::Node* DynamicAabbTree::Node::get_left() const
{
    return left;
}

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

    bounds = Aabb::Combine(left->bounds, right->bounds);
}

void DynamicAabbTree::Node::update_height()
{
    if (is_leaf())
    {
        height = 0;
        return;
    }

    const int height_left{left->height};
    const int height_right{right->height};

    height = Math::Max(height_left, height_right) + 1;
}

int DynamicAabbTree::Node::get_depth() const
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

DynamicAabbTree::Node::RotationData
DynamicAabbTree::Node::should_rotate() const
{
    if (is_leaf())
    {
        return {};
    }

    const int balance{left->height - right->height};
    if (balance <= 1 && balance >= -1)
    {
        return {};
    }

    if (balance > 0)
    {
        return left->generate_rotation();
    }

    return right->generate_rotation();
}

DynamicAabbTree::Node::RotationData DynamicAabbTree::Node::
generate_rotation()
{
    const bool left_is_smaller{left->height < right->height};

    return {
    left_is_smaller ? left : right,
    this,
    true,
    };
}

void DynamicAabbTree::Node::rotate(Node& small_child, Node& pivot)
{
    // TODO: Left off here trying to fix the rotation:
    // which has something wrong somewhere

    if (is_root())
    {
        tree->set_root(&pivot);
        pivot.parent = nullptr;
    }
    else
    {
        parent->replace_child(*this, &pivot);
    }

    pivot.replace_child(small_child, this);
    replace_child(pivot, &small_child);

    update_height();
    refit_bounds();

    pivot.update_height();
    pivot.refit_bounds();
}
