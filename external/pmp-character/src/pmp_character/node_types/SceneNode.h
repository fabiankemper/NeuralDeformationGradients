//
// Created by peasman on 28.06.24.
//

#ifndef NODE_H
#define NODE_H
#include <pmp_character/Types.h>
#include <pmp/types.h>
#include <vector>
#include <string>

#include <iostream>
#include <vector>
#include <memory> // For std::shared_ptr

inline int currentID=0;

class SceneNode {
public:
    // Constructor
    SceneNode(const std::string& name);
    SceneNode(const std::string& name, SceneNode* parent);
    virtual ~SceneNode() = default;

    bool operator==(const SceneNode &) const;

    void rotateNode(V3 axis, pmp::Scalar angle);

    void transformNodetransform(M4x4 transformation);

    // Method to add a child node
    void addChild(std::shared_ptr<SceneNode> child);

    // Method to remove a child node
    bool removeChild(std::shared_ptr<SceneNode> child);

    // Method to get the node's name
    std::string getName() const;

    // Method to get the children
    const std::vector<std::shared_ptr<SceneNode>>& getChildren() const;

    // Method to print the node and its children (for debugging)
    void print(int depth = 0) const;

    virtual void update() {}


    SceneNode* parent =nullptr;
    std::string name;
    int id;
    std::vector<std::shared_ptr<SceneNode>> children;
    M4x4 transform = pmp::mat4::identity();
    NodeType nodeType = NodeType::Node;
};

// Specialize std::hash for SceneNode
namespace std {
    template <>
    struct hash<SceneNode> {
        std::size_t operator()(const SceneNode& node) const {
            return std::hash<int>()(node.id);
        }
    };
}

#endif //NODE_H
