#pragma once

#include "osgqt_global.h"       
#include <osg/State>

class OSGQT_EXPORT StateEx :public osg::State
{
public:
    StateEx() :defaultFbo(0) {};
    inline void setDefaultFbo(GLuint fbo)
    {
        defaultFbo = fbo;
    }
    inline GLuint getDefaultFbo() const
    {
        return defaultFbo;
    }

protected:
    GLuint defaultFbo;
};
