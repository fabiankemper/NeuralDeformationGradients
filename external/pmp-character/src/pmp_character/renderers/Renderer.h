//
// Created by peasman on 28.06.24.
//

#ifndef RENDERER_H
#define RENDERER_H
#include "pmp_character/node_types/SceneNode.h"
#include <pmp/visualization/gl.h>

namespace pmp_character
{

    class Renderer
    {
    public:
        virtual ~Renderer() = default;
        Renderer([[maybe_unused]] std::shared_ptr<SceneNode> _node) {};
        std::shared_ptr<SceneNode> node;

        virtual void draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection,
                                const pmp::mat3 &normal, std::string draw_mode)=0;
        virtual void update() {}
    };


}


#endif //RENDERER_H
