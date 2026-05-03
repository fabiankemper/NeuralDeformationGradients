//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================
#pragma once
//=============================================================================

#include <pmp/visualization/trackball_viewer.h>
#include "InverseKinematics.h"
#include "node_types/Character.h"
#include "renderers/CharacterRenderer.h"

namespace pmp_character
{
    //== CLASS DEFINITION =========================================================

    /** 3D-Viewer for skeleton-based animation.
     The viewer class has a Character,
     which is controlled by manually selecting and manipulating joint angles,
     or through a pre-loaded animation, or through inverse kinematics
     \sa Character
     \sa Inverse_kinematics
    **/

    class Viewer : public pmp::TrackballViewer
    {
    public:
        /// constructor
        Viewer(const char *_title, int _width, int _height, bool _showgui = true);
        Viewer(const char *_title, int _width, int _height, const std::shared_ptr<Character> &character, bool _showgui = true);
        CharacterRenderer *character_;
        /// load character from file
        bool load_character(Character &character);
        /// inverse kinematics
        Inverse_kinematics ik_;
        void update();

    private: // GUI functions
        /// render the scene
        virtual void draw(const std::string &drawMode) override;

        /// handle keyboard events
        virtual void keyboard(int key, int code, int action, int mods) override;

        /// this function handles mouse button events
        virtual void mouse(int button, int action, int mods) override;

        /// this function handles mouse motion (passive/active position)
        virtual void motion(double x, double y) override;

        /// this function triggers the time_integration() function
        virtual void do_processing() override;

        /// render/handle GUI
        virtual void process_imgui() override;

    private: // simulation data and settings
        /// the loaded character to visualize




        /// are we doing ik or animation?
        bool manipulate_;

        /// draw the skeleton?
        bool draw_skeleton_;
    };
} // namespace pmp_character
//=============================================================================
