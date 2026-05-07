#pragma once
       
#include <osg/State>

class StateEx :public osg::State
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
