//
// Created by peasman on 28.06.24.
//


#include "SceneNode.h"

SceneNode::SceneNode(const std::string &name): name(name), transform(pmp::mat4::identity())
{
    id = currentID;
    currentID++;
}

SceneNode::SceneNode(const std::string &name, SceneNode *parent) : name(name), transform(pmp::mat4::identity()), parent(parent)
{
    id = currentID;
    currentID++;
}

bool SceneNode::operator==(const SceneNode &other) const
{
    if(id == other.id)
    {
        return true;
    }
    return false;
}

void SceneNode::rotateNode(V3 axis, pmp::Scalar angle)
{
    transform = transform * pmp::rotation_matrix(axis, angle);
}

void SceneNode::transformNodetransform(M4x4 transformation)
{
    transform = transform * transformation;
}

void SceneNode::addChild(std::shared_ptr<SceneNode> child)
{
    children.push_back(child);
    child->parent = this;
}

bool SceneNode::removeChild(std::shared_ptr<SceneNode> child)
{
    auto it = std::find(children.begin(), children.end(), child);
    if (it != children.end()) {
        children.erase(it);
        return true;
    }
    return false;
}

std::string SceneNode::getName() const
{
    return name;
}

const std::vector<std::shared_ptr<SceneNode>> & SceneNode::getChildren() const
{
    return children;
}

void SceneNode::print(int depth) const
{
    for (int i = 0; i < depth; ++i) {
        std::cout << "  ";
    }
    std::cout << name << std::endl;
    for (const auto& child : children) {
        child->print(depth + 1);
    }
}

