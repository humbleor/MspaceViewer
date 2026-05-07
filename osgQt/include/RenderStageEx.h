#pragma once

#include "osgqt_global.h"
#include <osgUtil/RenderStage>

class OSGQT_EXPORT RenderStageEx : public osgUtil::RenderStage
{
public:
    virtual void drawInner(osg::RenderInfo& renderInfo,
        osgUtil::RenderLeaf*& previous, bool& doCopyTexture);
};
