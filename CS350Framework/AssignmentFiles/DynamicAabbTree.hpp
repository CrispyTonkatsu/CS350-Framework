///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

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
        std::unordered_map<void*, Node> nodes{};

    public:
        Node& create_node(void* key, void* data, Aabb bounds);

        void insert_node(Node& node);

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

        size_t height{0};

    public:
        void set_data(void* new_data);
        void set_tree(Tree* new_tree);

        Node* get_sibling() const;
        Node* replace_child(Node& child, Node& replacement);

        bool is_root() const;
        bool is_leaf() const;
        Node& split(Node& other);

        Node* select_path(Node* left, Node* right) const;
        float get_current_cost() const;
        float get_possible_cost(const Node& other) const;
        void refit_bounds();

        Node* get_parent() const;
        Node* get_left() const;
        Node* get_right() const;

        enum class RotationType
        {
            LEFT_TO_RIGHTRIGHT,
            LEFT_TO_RIGHTLEFT,
            RIGHT_TO_LEFTLEFT,
            RIGHT_TO_LEFTRIGHT,
            NO_ROTATION,
        };

        RotationType should_rotate() const;
        float rotation_cost_delta(Node& to_rotate, Node& to_stay,
                                  Node& sibling) const;
        void rotate(Node& grandchild, Node& subtree);
    };
};
