// =====================================================================================================================

#include "volumetric_character/character_utils.h"
#include "volumetric_character/renderer/volumetric_character_renderer.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

[[maybe_unused]]
VolumetricCharacterRenderer::VolumetricCharacterRenderer(
    VolumetricCharacter& character
) : _character(character)
{
    _skel_wrap_renderer = new SkinnableMeshRenderer(_character.skinnable_layer(Layer::SKEL));
    _musc_wrap_renderer = new SkinnableMeshRenderer(_character.skinnable_layer(Layer::MUSC));
    _skin_renderer = new SkinnableMeshRenderer(_character.skinnable_layer(Layer::SKIN));

    _skeleton_renderer = new SkeletonRenderer(&_character);

    // set colors and transparency
    // skeleton
    _skel_wrap_renderer->material().front_color_ = pmp::Color(0.945, 0.859, 0.694);
    _skel_wrap_renderer->material().back_color_ = pmp::Color(0.8, 0.8, 0.8);

    // muscles
    _musc_wrap_renderer->material().alpha_ = 1.0F;
    _musc_wrap_renderer->material().front_color_ = pmp::Color(0.788, 0.386, 0.386);
    _musc_wrap_renderer->material().back_color_ = pmp::Color(0.8, 0.8, 0.8);

    // skin
    _skin_renderer->material().alpha_ = 0.34F;
    _skin_renderer->material().front_color_ = pmp::vec3(0.55, 0.75, 0.95);
    _skin_renderer->material().back_color_ = pmp::vec3(0.55, 0.75, 0.95);

}

// ---------------------------------------------------------------------------------------------------------------------

volumetric_character::VolumetricCharacterRenderer::~VolumetricCharacterRenderer()
{
    delete _skel_wrap_renderer;
    delete _musc_wrap_renderer;
    delete _skin_renderer;
    delete _highres_skel_renderer;
    delete _highres_musc_renderer;
    delete _skeleton_renderer;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacterRenderer::draw(
    const pmp::mat4& model_view,
    const pmp::mat4& model_view_projection,
    const pmp::mat3& normal,
    const std::string& draw_mode
) const -> void
{
    if (_character.highres_skinning) {

        // The deformed cage
        volumetric_base::LayeredMesh deformed_mesh(
            _character.skinnable_layer(Layer::SKEL).bake_mesh(),
            _character.skinnable_layer(Layer::MUSC).bake_mesh(),
            _character.skinnable_layer(Layer::SKIN).bake_mesh()
        );

        volumetric_base::LayeredMesh rest_mesh(
            _character.skinnable_layer(Layer::SKEL).mesh(),
            _character.skinnable_layer(Layer::MUSC).mesh(),
            _character.skinnable_layer(Layer::SKIN).mesh()
        );

        _highres_skel_renderer->draw_warped(model_view, model_view_projection, normal,
            deformed_mesh.toVertexMatrix(), rest_mesh.toVertexMatrix(), draw_mode);

        _highres_musc_renderer->draw_warped(model_view, model_view_projection, normal,
            deformed_mesh.toVertexMatrix(), rest_mesh.toVertexMatrix(), draw_mode);

    } else {
        _skel_wrap_renderer->draw(model_view, model_view_projection, normal, draw_mode);
        _musc_wrap_renderer->draw(model_view, model_view_projection, normal, draw_mode);
    }

    _skin_renderer->draw(model_view, model_view_projection, normal, draw_mode);

    if (show_skeleton && _skeleton_renderer != nullptr) {
        draw_skeleton(model_view_projection);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacterRenderer::set_alpha(float skeleton_alpha, float muscle_alpha, float skin_alpha) const -> void
{
    if (_skeleton_renderer) {
        _skel_wrap_renderer->material().alpha_ = utils::clamp(skeleton_alpha, 0.0f, 1.0f);
        _highres_skel_renderer->material().alpha_ = utils::clamp(skeleton_alpha, 0.0f, 1.0f);
    }
    if (_musc_wrap_renderer) {
        _musc_wrap_renderer->material().alpha_ = utils::clamp(muscle_alpha, 0.0f, 1.0f);
        _highres_musc_renderer->material().alpha_ = utils::clamp(   muscle_alpha, 0.0f, 1.0f);
    }
    if (_skin_renderer) {
        _skin_renderer->material().alpha_ = utils::clamp(skin_alpha, 0.0f, 1.0f);
    }
}

auto VolumetricCharacterRenderer::set_highres_weights(
    const bary_warp::BarycentricWarp &warp) -> void {
    volumetric_base::LayeredMesh const mesh (
        _character.skinnable_layer(Layer::SKEL).bake_mesh(),
        _character.skinnable_layer(Layer::MUSC).bake_mesh(),
        _character.skinnable_layer(Layer::SKIN).bake_mesh()
    );

    _highres_skel_renderer = new WarpedMeshRenderer(_character.highres_layer(Layer::SKEL), warp._barycentric_warper->skeleton_weights);
    _highres_musc_renderer = new WarpedMeshRenderer(_character.highres_layer(Layer::MUSC), warp._barycentric_warper->muscle_weights);

    // same material for highres
    _highres_skel_renderer->material() = _skel_wrap_renderer->material();
    _highres_musc_renderer->material() = _musc_wrap_renderer->material();

    _highres_skel_renderer->set_highres_topology(
        _character.state()->get_highres_mesh(Layer::SKEL)
    );
    _highres_musc_renderer->set_highres_topology(
        _character.state()->get_highres_mesh(Layer::MUSC)
    );

}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacterRenderer::draw_skeleton(const pmp::mat4& model_view_projection) const -> void
{
    if (_skeleton_renderer != nullptr) {
        if (_character.is_skeleton_dirty()) {
            _skeleton_renderer->update_buffers();
        }
        _skeleton_renderer->draw(model_view_projection);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================
