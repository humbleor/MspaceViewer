#pragma once

#include <osgUtil/CullVisitor>

class CullVisitorEx :public osgUtil::CullVisitor
{
public:
    META_NodeVisitor(Ex, CullVisitorEx)

    CullVisitorEx() {}
    CullVisitorEx(const CullVisitorEx& cv) : osgUtil::CullVisitor(cv) { }
    CullVisitorEx* clone() const
    {
        return new CullVisitorEx(*this);
    }

    virtual void apply(osg::Camera& camera);
};
