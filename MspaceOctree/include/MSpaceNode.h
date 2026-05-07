#pragma once

#include "MSpaceOCTree_global.h"
#include <osg/PagedLOD>
#include <osgDB/DatabasePager>

namespace osg {
	class MSPACEOCTREE_LIB_EXPORT MSpaceNode:public osg::PagedLOD
	{
	public:
		MSpaceNode();
		~MSpaceNode();

		virtual void traverse(osg::NodeVisitor& nv);
		virtual bool addChild(Node* child);
		virtual bool removeExpiredChildren(double expiryTime, unsigned int expiryFrame, NodeList& removedChildren);
		void setTimeStamp(unsigned int childNo, double timeStamp) { _perRangeDataList[0]._timeStamp = timeStamp; }
		void setFrameNumber(unsigned int childNo, unsigned int frameNumber) { _perRangeDataList[0]._frameNumber = frameNumber; }
	};

}