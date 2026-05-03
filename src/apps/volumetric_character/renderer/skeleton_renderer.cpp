// =====================================================================================================================

#include "skeleton_renderer.h"

#include <pmp/algorithms/utilities.h>
#include <pmp_character/renderers/shaders.h>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------

SkeletonRenderer::SkeletonRenderer(VolumetricCharacter* character)
{
    _volumetric_character = character;
    _shader.source(skeleton_vshader, skeleton_fshader);

    // supress warning
    (void) character_vshader;
    (void) character_fshader;
}

SkeletonRenderer::~SkeletonRenderer()
{
    if (_vertex_buffer)
        glDeleteBuffers(1, &_vertex_buffer);
    if (_color_buffer)
        glDeleteBuffers(1, &_color_buffer);
    if (_line_vertex_array_object)
        glDeleteVertexArrays(1, &_line_vertex_array_object);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkeletonRenderer::draw(const pmp::mat4& model_view_projection) -> void
{
    if (not _volumetric_character || _volumetric_character->state()->rig.joints_.empty()) {
        return;
    }

    if (!_shader.is_valid())
    {
        _shader.source(skeleton_vshader, skeleton_fshader);
    }

    if(not _line_vertex_array_object)
    {
        update_buffers();
    }

    pmp::mat4 mvp = model_view_projection;

    glBindVertexArray(_line_vertex_array_object);

    _shader.use();
    _shader.set_uniform("modelview_projection_matrix", mvp);

    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_LINES, 0, _vertex_buffer_size);

    glEnable(GL_DEPTH_TEST);
    _shader.disable();

    // TODO: ENABLE THIS
    // draw_selected_joint();
    glBindVertexArray(0);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkeletonRenderer::update_buffers() -> void
{
    if (not _volumetric_character) {
        return;
    }

    if (not _line_vertex_array_object) {
        glGenVertexArrays(1, &_line_vertex_array_object);
        glBindVertexArray(_line_vertex_array_object);
        glGenBuffers(1, &_color_buffer);
        glGenBuffers(1, &_skeleton_cs_color_buffer);
        glGenVertexArrays(1, &_joint_vertex_array_object);
        glGenBuffers(1, &_joint_buffer);
        glGenBuffers(1, &_joint_color_buffer);
    }

    std::vector<volumetric_base::Joint *>& joints = _volumetric_character->state()->rig.joints_;
    size_t num_joints = joints.size();

    std::vector<pmp::vec3> position_array;
    std::vector<pmp::vec3> color_array;

    _vertex_buffer_size = static_cast<int>(6 * num_joints);

    for (volumetric_base::Joint * j: joints) {
        if (j->parent_) {
            _vertex_buffer_size += 2;
        }
    }

    position_array.reserve(_vertex_buffer_size);
    color_array.reserve(_vertex_buffer_size);

    pmp::BoundingBox bb;
    bb += pmp::bounds(_volumetric_character->skinnable_layer(Layer::SKEL).mesh());
    bb += pmp::bounds(_volumetric_character->skinnable_layer(Layer::MUSC).mesh());
    bb += pmp::bounds(_volumetric_character->skinnable_layer(Layer::SKIN).mesh());

    float cs_size = bb.size() * 0.01f;

    const static pmp::vec3 x_axis = pmp::vec3(1.0f, 0.0f, 0.0f);
    const static pmp::vec3 y_axis = pmp::vec3(0.0f, 1.0f, 0.0f);
    const static pmp::vec3 z_axis = pmp::vec3(0.0f, 0.0f, 1.0f);
    const static pmp::vec3 gray = pmp::vec3(0.3f, 0.3f, 0.3f);

    for (size_t joint_index = 0; joint_index < num_joints; ++joint_index) {
      volumetric_base::Joint * j = joints[joint_index];
        pmp::vec3 jp = j->get_global_translation();

        if (j->parent_) {
            position_array.push_back(jp);
            position_array.push_back(j->parent_->get_global_translation());

            color_array.push_back(gray);
            color_array.push_back(gray);
        }

        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, x_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, y_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, z_axis));

        color_array.push_back(x_axis);
        color_array.push_back(x_axis);
        color_array.push_back(y_axis);
        color_array.push_back(y_axis);
        color_array.push_back(z_axis);
        color_array.push_back(z_axis);
    }

    glBindVertexArray(_line_vertex_array_object);

    glBindBuffer(GL_ARRAY_BUFFER, _color_buffer);
    glBufferData(GL_ARRAY_BUFFER, position_array.size() * sizeof(position_array[0]),
                 position_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, _skeleton_cs_color_buffer);
    glBufferData(GL_ARRAY_BUFFER, color_array.size() * sizeof(color_array[0]),
                 color_array.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);

    // clear dirty
    _volumetric_character->set_skeleton_dirty(false);
}

auto SkeletonRenderer::draw_selected_joint() -> void
{
    glBindVertexArray(_joint_vertex_array_object);
    if(not _shader.is_valid())
    {
        _shader.source(skeleton_vshader, skeleton_fshader);
    }
    _shader.use();
    std::vector<pmp::vec3> position_array;
    std::vector<pmp::vec3> color_array;

    const pmp::vec3 x_ax = pmp::vec3(1.0f, 0.0f, 0.0f);
    const pmp::vec3 y_ax = pmp::vec3(0.0f, 1.0f, 0.0f);
    //const pmp::vec3 z_ax = pmp::vec3(0.0f, 0.0f, 1.0f);

    // collect effectors (in red color)
    for (auto j : _volumetric_character->state()->rig.joints_)
    {
        if (j->is_effector_)
        {
            position_array.push_back(j->get_global_translation());
            color_array.push_back(x_ax);
        }
    }

    // add selected joint (in green color)
    if (_volumetric_character->state()->rig.selected_joint_ > 0 &&
        _volumetric_character->state()->rig.selected_joint_ < (static_cast<int> (_volumetric_character->state()->rig.joints_.size())))
    {
        position_array.push_back(
            _volumetric_character->state()->rig.joints_[_volumetric_character->state()->rig.selected_joint_]->get_global_translation());
        color_array.push_back(y_ax);
    }
    // upload buffer and draw as points
    if (not position_array.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, _joint_buffer);
        glBufferData(GL_ARRAY_BUFFER, position_array.size() * sizeof(pmp::vec3), &position_array[0], GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, _joint_color_buffer);
        glBufferData(GL_ARRAY_BUFFER, color_array.size() * sizeof(pmp::vec3), &color_array[0], GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(1);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, position_array.size());
    }
    _shader.disable();
    glEnable(GL_DEPTH_TEST);
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================
