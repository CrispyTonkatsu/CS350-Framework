///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include "Shapes.hpp"
#include "SpatialPartition.hpp"

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
    DynamicAabbTree();
    ~DynamicAabbTree() override;

    // Spatial Partition Interface
    void InsertData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;

    void UpdateData(SpatialPartitionKey& key,
                    SpatialPartitionData& data) override;

    void RemoveData(SpatialPartitionKey& key) override;

    void DebugDraw(int level, const Matrix4& transform,
                   const Vector4& color = Vector4(1), int bitMask = 0) override;

    void CastRay(const Ray& ray, CastResults& results) override;
    void CastFrustum(const Frustum& frustum, CastResults& results) override;

    void SelfQuery(QueryResults& results) override;

    void
    FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

    static const float mFatteningFactor;

    // Assignment3: Add your implementation here

    class Node;

    class Tree
    {
        Node* root{nullptr};

    public:
        std::map<Node*, std::unique_ptr<Node>> nodes{};

        void set_root(Node* new_root);
        Node* get_root() const;

        Node& create_node(void* data, const Aabb& bounds);

        void insert_node(Node& node);

        void remove_node(Node& node);

        void delete_node(Node& node);

        void update_nodes(Node& node);
    };

    Tree tree{};

    class Node
    {
        Tree* tree{nullptr};

        Aabb bounds{};
        void* data{nullptr};

        Node* parent{nullptr};
        Node* left{nullptr};
        Node* right{nullptr};

    public:
        int height{0};

        void* get_data() const;
        Aabb get_bounds() const;

        void set_data(void* new_data);
        void set_tree(Tree* new_tree);
        void set_parent(Node* new_parent);
        void set_bounds(const Aabb& new_bounds);

        Node* get_sibling() const;
        Node* replace_child(Node& to_replace, Node* replacement);

        bool is_root() const;
        bool is_leaf() const;
        Node& split(Node& other);

        Node* select_path(Node* left_path, Node* right_path) const;
        float get_current_cost() const;
        float get_possible_cost(const Node& other) const;
        float get_possible_cost_delta(const Node& other) const;

        void refit_bounds();
        void update_height();

        int get_depth() const;

        Node* get_parent() const;
        Node* get_left() const;
        Node* get_right() const;

        // TODO: Replace rotation type with a struct that can be used to just execute the rotation instead

        struct RotationData
        {
            float cost_delta{0.f};
            Node* big_child{nullptr};
            Node* pivot{nullptr};
            bool is_valid{false};
        };

        RotationData should_rotate() const;
        float rotation_cost_delta(const Node* big_child,
                                  const Node* small_child,
                                  const Node* pivot,
                                  const Node* sibling) const;
        void rotate(Node& big_child, Node& pivot);
    };
};
