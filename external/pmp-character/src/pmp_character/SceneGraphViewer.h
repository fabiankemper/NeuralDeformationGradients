//
// Created by peasman on 28.06.24.
//

#ifndef SCENEGRAPHVIEWER_H
#define SCENEGRAPHVIEWER_H

#include <pmp/visualization/trackball_viewer.h>

#include "node_types/SceneNode.h"
#include "renderers/Renderer.h"
#include <unordered_map>
namespace pmp_character
{
    class SceneGraphViewer :  public pmp::TrackballViewer {
    public:
        SceneGraphViewer(const char *_title, int _width, int _height, bool _showgui = true);

        std::shared_ptr<SceneNode> root_node = std::make_shared<SceneNode>("root");

        std::unordered_map<SceneNode, std::shared_ptr<Renderer>> renderers;

        void draw_scene_recursive(M4x4 modelView, const std::shared_ptr<SceneNode> &node);
        void createRenderer(const std::shared_ptr<SceneNode> &node);
        void ShowTreeNode(const std::shared_ptr<SceneNode>& node);
        void ShowTree(const std::shared_ptr<SceneNode>& root);
        void renderByNodeType(const M4x4 &modelView, const std::shared_ptr<SceneNode> &node);
        std::shared_ptr<SceneNode> getSceneNodeByName(const std::string& name);

        void add_scene_node_to_root_node(const std::shared_ptr<SceneNode> &node);

        virtual void draw(const std::string &drawMode) override;

        /// handle keyboard events
        virtual void keyboard(int key, int code, int action, int mods) override;

        /// this function handles mouse button events
        virtual void mouse(int button, int action, int mods) override;

        /// this function handles mouse motion (passive/active position)
        virtual void motion(double x, double y) override;

        /// render/handle GUI
        virtual void process_imgui() override;

    private: // simulation data and settings
        /// the loaded character to visualize




        /// are we doing ik or animation?
        bool manipulate_ = false;

        /// draw the skeleton?
        bool draw_skeleton_ = true;
    };
}


#endif //SCENEGRAPHVIEWER_H
