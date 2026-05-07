#pragma once

#include <osgUtil/RenderStage>

class RenderStageEx : public osgUtil::RenderStage
{
public:
    virtual void drawInner(osg::RenderInfo& renderInfo,
        osgUtil::RenderLeaf*& previous, bool& doCopyTexture);
};
