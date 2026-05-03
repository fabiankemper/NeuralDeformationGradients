//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================

//== INCLUDES =================================================================

#include "Viewer.h"

#include <string>

#include <imgui.h>
#ifdef __APPLE__
    #include <pmp/visualization/gl.h>
#else
    #include <GL/gl.h>
#endif
#include <pmp/stop_watch.h>
#include <pmp/visualization/trackball_viewer.h>

// todo: fake @friedemann: why?
#include "InverseKinematics.h"

namespace pmp_character
{
    //== IMPLEMENTATION ==========================================================

    Viewer::Viewer(const char *_title, int _width, int _height, bool _showgui) :
        pmp::TrackballViewer(_title, _width, _height, _showgui)
    {
        manipulate_ = false;
        draw_skeleton_ = true;
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

    Viewer::Viewer(const char *_title, int _width, int _height, const std::shared_ptr<Character> &character, bool _showgui) :
        pmp::TrackballViewer(_title, _width, _height, _showgui)
    {
        character->skinning();
        character_ = new CharacterRenderer(character);
        ik_.set_skeleton(&character_->character->skeleton_);

        #ifdef __EMSCRIPTEN__
            ik_.load_dof("default_vitras_template.dof");
        #else
            ik_.load_dof("../data/default_vitras_template.dof");
        #endif
        std::vector<double> prev_params = ik_.get_current_parameters();
        ik_.collect_dofs();
        ik_.set_parameters(prev_params);
        ik_.set_clamping(true);
        manipulate_ = false;
        draw_skeleton_ = true;
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
        set_scene(pmp::vec3(character_->character->skeleton_.get_center()), 0.5f * character_->character->skeleton_.get_size());
    }


    bool Viewer::load_character(Character &character)
    {
        character_ = new CharacterRenderer(std::make_shared<Character>(character));
        for (auto &child : character_->mesh_node_renderers)
        {
            child->create_buffers();
        }
        character_->skeletonRenderer->update();
        // ik_ = Inverse_kinematics(character, Inverse_kinematics::PSEUDO_INVERSE);
        set_scene(pmp::vec3(character_->character->skeleton_.get_center()), 0.5f * character_->character->skeleton_.get_size());
        return true;
    }
    //-----------------------------------------------------------------------------

    void Viewer::keyboard(int key, int code, int action, int mods)
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
                character_->character->skeleton_.reset();
                manipulate_ = false;
                character_->character->update();
                character_->updateSkinnedVerticesAndBlendshapes();
                view_all();
                break;
            }

        case GLFW_KEY_UP:
            {
                character_->select_joint(character_->character->skeleton_.selected_joint() + 1);
                break;
            }

        case GLFW_KEY_DOWN:
            {
                character_->select_joint(character_->character->skeleton_.selected_joint() - 1);
                break;
            }

        default:
            {
                TrackballViewer::keyboard(key, code, action, mods);
                break;
            }
        }
    }

    //-----------------------------------------------------------------------------

    void Viewer::do_processing()
    {
        //static double elapsed;
        //static pmp::StopWatch timer;
    }

    //-----------------------------------------------------------------------------

    void Viewer::process_imgui()
    {
        if (ImGui::CollapsingHeader("Select/Animate Joint", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (character_->character->skeleton_.selected_joint() == -1)
                character_->character->skeleton_.select_joint(1);

            int j = character_->character->skeleton_.selected_joint();
            ImGui::Text("Selected Joint: %s", character_->character->skeleton_.get_joint(j)->name_.c_str());

            if (ImGui::Button("Previous Joint"))
            {
                character_->select_joint(j - 1);
            }
            ImGui::SameLine();
            if (ImGui::Button("Next Joint"))
            {
                character_->select_joint(j + 1);
            }

            ImGui::Spacing();

            if (ImGui::Button("X+"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(1, 0, 0), 10.0f);
                manipulate_ = true;
                update();

            }
            ImGui::SameLine();
            if (ImGui::Button("X-"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(1, 0, 0), -10.0f);
                manipulate_ = true;
                update();

            }
            ImGui::SameLine();
            if (ImGui::Button("Y+"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(0, 1, 0), 10.0f);
                manipulate_ = true;
                update();

            }
            ImGui::SameLine();
            if (ImGui::Button("Y-"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(0, 1, 0), -10.0f);
                manipulate_ = true;
                update();

            }
            ImGui::SameLine();
            if (ImGui::Button("Z+"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(0, 0, 1), 10.0f);
                manipulate_ = true;
                update();

            }
            ImGui::SameLine();
            if (ImGui::Button("Z-"))
            {
                character_->character->skeleton_.rotate_joint(character_->character->skeleton_.selected_joint(),
                                                             pmp::vec3(0, 0, 1), -10.0f);
                manipulate_ = true;
                update();

            }
        }

        ImGui::Spacing();
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Inverse Kinematics", ImGuiTreeNodeFlags_DefaultOpen))
        {

            // float e = ik_.;
            // ImGui::SliderFloat("Epsilon for PInv", &e, 0.01f, 1.0f, "%.2f");


            // int ik_mode = ik_.;
            // ImGui::RadioButton("Pseudo-Inverse", &ik_mode, 0);
            // ImGui::RadioButton("Damped Least Squares", &ik_mode, 1);
            // ik_.set_ik_mode(ik_mode);
            //
            // if (ik_mode == 0)
            // {
            //     float e = ik_.epsilon();
            //     ImGui::PushItemWidth(70);
            //     ImGui::SliderFloat("Epsilon for PInv", &e, 0.01f, 1.0f, "%.2f");
            //     ImGui::PopItemWidth();
            //     ik_.set_epsilon(e);
            // }
            // else if (ik_mode == 1)
            // {
            //     float d = ik_.damping();
            //     ImGui::PushItemWidth(70);
            //     ImGui::SliderFloat("Damping for DLS", &d, 0.0f, 100.0f, "%.2f");
            //     ImGui::PopItemWidth();
            //     ik_.set_damping(d);
            // }
        }
        if(ImGui::Button("SetBindPose"))
        {
            character_->character->skeleton_.set_current_pose_as_bindpose();
        }
        ImGui::Spacing();
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Skinning", ImGuiTreeNodeFlags_DefaultOpen))
        {
            int skin_mode = character_->character->get_skinning_mode();
            ImGui::RadioButton("LBS", &skin_mode, 0);
            ImGui::RadioButton("DQS", &skin_mode, 1);
            character_->character->set_skinning_mode(skin_mode);
        }

        ImGui::Spacing();
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen))
        {
            std::string draw_mode = character_->get_draw_mode();
            // ImGui::RadioButton("Shaded", &draw_mode, 0);
            // ImGui::RadioButton("Textured", &draw_mode, 1);
            // ImGui::RadioButton("Bone Weights", &draw_mode, 2);
            ImGui::Checkbox("Draw Skeleton", &draw_skeleton_);
            character_->set_draw_mode(draw_mode);
        }
        if (ImGui::CollapsingHeader("Blendshapes", ImGuiTreeNodeFlags_OpenOnArrow))
        {
            if (not character_->character->get_mesh_by_index(0)->blendshape_collection.empty)
            {
                for (size_t i = 0; i < character_->character->blendshapeNames.size(); i++)
                {
                    if (ImGui::SliderFloat(character_->character->blendshapeNames[i].c_str(),
                                           &character_->character->blendshapeEffectors[i], 0, 1))
                    {
                        character_->character->apply_blendshape();
                        update();
                    }
                }
            }
        }
        if(ImGui::Button("Translate"))
        {
            // vec3 translation = vec3(1,0,0);
            // translate(translation);
            character_->character->transform  =character_->character->transform *  pmp::scaling_matrix(pmp::vec3(1.5,1,1));
        }
    }
    void Viewer::update()
    {
        character_->character->update();
        character_->skeletonRenderer->update();

    }

    //-----------------------------------------------------------------------------

    void Viewer::draw(const std::string &drawMode)
    {
        // OpenGL state
        glEnable(GL_DEPTH_TEST);

        // clear screen
        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // modelview_matrix_ = modelview_matrix_ * character_.character->transform;
        pmp::mat4 currentModelView = modelview_matrix_ * character_->character->transform;
        // compute required matrices
        pmp::mat3 normalMatrix = inverse(transpose(linear_part(currentModelView)));
        pmp::mat4 modelviewprojectionMatrix = projection_matrix_ * currentModelView;
        character_->draw(currentModelView, modelviewprojectionMatrix, normalMatrix, drawMode);


        // draw skeleton
        if (draw_skeleton_)
        {
            character_->draw_skeleton(modelviewprojectionMatrix);
        }
        // draw character mesh
    }

    //----------------------------------------------------------------------------

    void Viewer::mouse(int _button, int _action, int _mods)
    {
        // CTRL not pressed -> rotate model
        if (!ctrl_pressed())
        {
            TrackballViewer::mouse(_button, _action, _mods);
            return;
        }

        // CTRL pressed -> drag joints around
        if (_action == GLFW_PRESS)
        {
            // select joint
            if (_button == GLFW_MOUSE_BUTTON_LEFT)
            {
                // get point under mouse cursor
                double x, y;
                cursor_pos(x, y);

                // get viewport paramters
                pmp::vec4 vp(0.0f);
                glGetFloatv(GL_VIEWPORT, &vp[0]);

                // in OpenGL y=0 is at the 'bottom'
                y = vp[3] - y;

                // invert viewport mapping
                pmp::vec2 p2d;
                p2d[0] = ((float)x - (float)vp[0]) / ((float)vp[2]) * 2.0f - 1.0f;
                p2d[1] = ((float)y - (float)vp[1]) / ((float)vp[3]) * 2.0f - 1.0f;

                // find closest joint
                auto mvp = projection_matrix_ * modelview_matrix_;
                int j = character_->character->skeleton_.pick_joint(pmp::vec2(p2d[0], p2d[1]), mvp);

                // select this joint

                character_->select_joint(j);

                // set it as IK effector
                manipulate_ = true;
            }
        }
    }

    //-----------------------------------------------------------------------------

    void Viewer::motion(double _x, double _y)
    {
        // CTRL not pressed -> rotate model
        if (!ctrl_pressed())
        {
            TrackballViewer::motion(_x, _y);
            return;
        }

        if (left_mouse_pressed())
        {
            if (character_->character->skeleton_.selected_joint() != -1)
            {
                int joint = character_->character->skeleton_.selected_joint();
                pmp::vec3 p = character_->character->skeleton_.get_joint(joint)->get_global_translation();

                // project joint position to 2D
                pmp::mat4 mvp = projection_matrix_ * modelview_matrix_;
                pmp::vec4 pp = mvp * pmp::vec4(p[0], p[1], p[2], 1.0f);
                pp /= pp[3];

                // get viewport data
                GLint viewport[4];
                glGetIntegerv(GL_VIEWPORT, viewport);
                int x = (int)_x;
                int y = (int)_y;

                // in OpenGL y=0 is at the 'bottom'
                y = viewport[3] - y;

                // unproject mouse position to 3D
                pmp::vec4 pn;
                pn[0] = ((float)x - (float)viewport[0]) / ((float)viewport[2]) * 2.0f - 1.0f;
                pn[1] = ((float)y - (float)viewport[1]) / ((float)viewport[3]) * 2.0f - 1.0f;
                // use z of projected joint position for correct depth
                pn[2] = pp[2];
                pn[3] = 1.0f;

                // unproject
                pn = inverse(mvp) * pn;
                pn /= pn[3];

                // set target position for effector and solve IK
                pmp::Point target(pn[0], pn[1], pn[2]);
                // ik_.set_effector(joint, target);
                std::vector<int> joints;
                joints.emplace_back(joint);
                std::vector<pmp::Point> targets;
                targets.emplace_back(target);

                ik_.set_targets_and_positions_by_index(targets, joints);
                ik_.solve_IK();
                update();
            }
        }
    }
} // namespace pmp_character
//=============================================================================
