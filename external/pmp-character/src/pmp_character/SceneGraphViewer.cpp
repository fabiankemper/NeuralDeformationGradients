//
// Created by peasman on 28.06.24.
//

#include "SceneGraphViewer.h"
#include <imgui.h>
#include "renderers/CharacterRenderer.h"
#include "renderers/NodeRenderer.h"
#include "renderers/PointSetRenderer.h"


namespace pmp_character
{
    SceneGraphViewer::SceneGraphViewer(const char *_title, int _width, int _height, bool _showgui) :
    pmp::TrackballViewer(_title, _width, _height, _showgui)
    {
        manipulate_ = false;
        draw_skeleton_ = true;
        renderers[*root_node] = std::make_shared<NodeRenderer>(root_node);
        add_draw_mode("Texture");
        add_draw_mode("Smooth Shading");
        set_draw_mode("Texture");

        clear_help_items();
        add_help_item("Up/Down", "Select next/prev joint");
        add_help_item("Backspace", "Reset posture and effectors");
        add_help_item("Left mouse", "Rotate character");
        add_help_item("Right mouse", "Zoom character");
        add_help_item("Mouse wheel", "Zoom character");
        add_help_item("Ctrl + Left mouse", "Select/drag joint");
        add_help_item("G", "Toggle GUI dialog");
        add_help_item("PageUp/Down", "Scale GUI dialogs");
#ifndef __EMSCRIPTEN__
        add_help_item("Esc/Q", "Quit application");
#endif
    }
    void SceneGraphViewer::renderByNodeType(const M4x4 &modelView, const std::shared_ptr<SceneNode> &node)
    {
        auto renderer = renderers[*node];
        // std::cout << modelView << std::endl;
        switch(node->nodeType)
        {
        case NodeType::Mesh:
            // std::dynamic_pointer_cast<MeshN>(renderers[*node]);
            break;
        case NodeType::SkinnedMesh:
        {
            auto actualRenderer = std::dynamic_pointer_cast<SkinnedMeshNodeRenderer>(renderer);
            actualRenderer->draw(modelView, projection_matrix_ * modelView, linear_part(modelview_matrix_), draw_mode_names_[draw_mode_]);
            break;
        }
        case NodeType::Character:
        {
            auto actualRenderer = std::dynamic_pointer_cast<CharacterRenderer>(renderer);
            actualRenderer->draw(modelView, projection_matrix_ * modelView, linear_part(modelview_matrix_), draw_mode_names_[draw_mode_]);
            break;
        }
        case NodeType::Node:
        {
            auto actualRenderer = std::dynamic_pointer_cast<NodeRenderer>(renderer);
            actualRenderer->draw(modelView, projection_matrix_ * modelView, linear_part(modelview_matrix_), draw_mode_names_[draw_mode_]);
            break;
        }
        case NodeType::PointSet:
        {
            auto actualRenderer = std::dynamic_pointer_cast<PointSetRenderer>(renderer);
            actualRenderer->draw(modelView, projection_matrix_ * modelView, linear_part(modelview_matrix_), draw_mode_names_[draw_mode_]);
            break;
        }
        default: ;
        }
    }
    std::shared_ptr<SceneNode> getSceneNodeByNameRecursive(const std::shared_ptr<SceneNode>& node, const std::string &name)
    {
        if(node->name == name)
        {
            return node;
        }
        for(auto& child : node->children)
        {
            auto foundChild = getSceneNodeByNameRecursive(child, name);
            if(foundChild != nullptr)
            {
                return foundChild;
            }
        }

        std::cerr << "Could not find child node with name " << name << ".\n";
        return nullptr;
    }

    std::shared_ptr<SceneNode> SceneGraphViewer::getSceneNodeByName(const std::string &name)
    {
        auto ref = getSceneNodeByNameRecursive(root_node, name);
        if(ref!= nullptr)
        {
            return ref;
        }
        else
        {
            std::cerr << "Could not find Scene node with name " << name << std::endl;
            return nullptr;
        }
    }

    void SceneGraphViewer::draw_scene_recursive(M4x4 modelView, const std::shared_ptr<SceneNode> &node)
    {
        // Get the current Renderer
        renderByNodeType(modelView, node);
        if(node->nodeType == NodeType::Character)
        {
            return;
        }
        // renderers[*node]->draw(modelView, projection_matrix_,linear_part(modelview_matrix_), draw_mode_names_[draw_mode_] );

        // Transform the modelview with the current nodes transform
        M4x4 newModelView = modelView * node->transform;
        for(auto child : node->children)
        {
            draw_scene_recursive(newModelView, child);
        }
    }
    void SceneGraphViewer::createRenderer(const std::shared_ptr<SceneNode> &node)
    {
        std::cout << "Create Renderer " << node->name << " with type " << toString(node->nodeType) << std::endl;
        switch(node->nodeType)
        {
            case NodeType::Mesh:
            {
                break;
            }
            case NodeType::SkinnedMesh:
            {
                auto renderer = std::make_shared<SkinnedMeshNodeRenderer>(SkinnedMeshNodeRenderer(std::dynamic_pointer_cast<MeshNode>(node)));
                renderers[*node] = renderer;
                break;
            }
            case NodeType::Character:
            {
                renderers[*node] = std::make_shared<CharacterRenderer>(
                    CharacterRenderer(std::dynamic_pointer_cast<Character>(node)));
                break;
            }

            case NodeType::Node:
                {
                    renderers[*node] = std::make_shared<NodeRenderer>(NodeRenderer(std::dynamic_pointer_cast<SceneNode>(node)));
                    break;
                }
        case NodeType::PointSet:
            {
                auto pointset =std::dynamic_pointer_cast<PointSet>(node);
                renderers[*node] = std::make_shared<PointSetRenderer>(pointset);
                break;
            }
            default: ;
        }
    }
    // Function to recursively display tree nodes
    void SceneGraphViewer::ShowTreeNode(const std::shared_ptr<SceneNode>& node) {
        if (ImGui::TreeNodeEx(node->name.c_str())) {
            switch(node->nodeType) {
            case NodeType::Character:
            {
                auto renderer = std::dynamic_pointer_cast<CharacterRenderer>(renderers[*node]);
                ImGui::Checkbox("Render skeleton", &renderer->renderSkeleton);
                break;
            }
            case NodeType::SkinnedMesh:
            {
                auto renderer = std::dynamic_pointer_cast<SkinnedMeshNodeRenderer>(renderers[*node]);
                ImGui::Checkbox("Visible", &renderer->show);
                break;
            }
            case NodeType::PointSet:
            {
                auto renderer = std::dynamic_pointer_cast<PointSetRenderer>(renderers[*node]);
                ImGui::SliderFloat("PointSize: ", &renderer->point_size, 0,25);
                break;
            }
            default:
            {
                break;
            }
            }
            for (const auto& child : node->children) {
                ShowTreeNode(child);
            }
            ImGui::TreePop();
        }
    }

    // Main function to display the tree
    void SceneGraphViewer::ShowTree(const std::shared_ptr<SceneNode>& root) {
        // ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.f, 5.f));
        ShowTreeNode(root);
    }
    void SceneGraphViewer::add_scene_node_to_root_node(const std::shared_ptr<SceneNode> &node)
    {
        std::cout << "Added node " << node->name << " with type " << toString(node->nodeType) << std::endl;
        if(renderers.contains(*node))
        {
            return;
        }
        createRenderer(node);
        root_node->addChild(node);
        if(std::dynamic_pointer_cast<Character>(node) != nullptr)
        {
            auto characterRenderer = std::dynamic_pointer_cast<CharacterRenderer>(renderers[*node]);
            for(int i =0; i < node->children.size(); i++)
            {
                auto child = node->children[i];
                renderers[*child] = characterRenderer->mesh_node_renderers[i];
            }
            auto character = std::dynamic_pointer_cast<Character>(node);
            set_scene(character->skeleton_.center_, character->skeleton_.size_);
        } else
        {
            for(const auto& child : node->children)
            {
                createRenderer(child);
            }
        }

    }

    void SceneGraphViewer::draw(const std::string &drawMode)
    {
        // modelview_matrix_ = modelview_matrix_ * character_.character->transform;
        pmp::mat4 currentModelView = modelview_matrix_;
        // compute required matrices
        pmp::mat3 normalMatrix = inverse(transpose(linear_part(currentModelView)));
        pmp::mat4 modelviewprojectionMatrix = projection_matrix_ * currentModelView;


        // draw scene
        draw_scene_recursive(currentModelView, root_node);
    }


    void SceneGraphViewer::mouse(int _button, int _action, int _mods)
    {
        // CTRL not pressed -> rotate model
        if (!ctrl_pressed())
        {
            TrackballViewer::mouse(_button, _action, _mods);
            return;
        }
    }
    void SceneGraphViewer::motion(double _x, double _y)
    {
        // CTRL not pressed -> rotate model
        if (!ctrl_pressed())
        {
            TrackballViewer::motion(_x, _y);
            return;
        }
    }

    void SceneGraphViewer::process_imgui()
    {
        ImGui::SetNextItemOpen(true);
        if(ImGui::CollapsingHeader("TreeStructure"))
        {
            ShowTree(root_node);
        }
    }
    void SceneGraphViewer::keyboard(int key, int code, int action, int mods)
    {
        if (action != GLFW_PRESS && action != GLFW_REPEAT)
            return;

        switch (key)
        {
            // toggle animation
        case GLFW_KEY_SPACE:
        {
            // TODO switch drawmode
            if (draw_mode_names_[draw_mode_] == "Texture")
            {
                set_draw_mode("Smooth Shading");
                break;
            }
            if (draw_mode_names_[draw_mode_] == "Smooth Shading")
            {
                set_draw_mode("Texture");
                break;
            }
            // if (!animate_ && manipulate_)
            // {
            //     character_.reset();
            //     manipulate_ = false;
            // }
            // animate_ = !animate_;
            break;
        }

            // reset posture
        case GLFW_KEY_BACKSPACE:
        {
            manipulate_ = false;
            view_all();
            break;
        }

        default:
        {
            TrackballViewer::keyboard(key, code, action, mods);
            break;
        }
        }
    }
}
