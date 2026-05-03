//
// Created by peasman on 28.06.24.
//

#ifndef MATRIXUTILITY_H
#define MATRIXUTILITY_H
#include "pmp_character/Types.h"

namespace MatrixUtility
{
    static V3 getTranslationPart(const M4x4& mat)
    {
        return V3(mat(0,3), mat(1,3), mat(2,3));
    }

}


#endif //MATRIXUTILITY_H
