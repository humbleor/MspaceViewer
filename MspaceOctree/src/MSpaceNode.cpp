#include "../include/MSpaceNode.h"	

#include <algorithm>
#include <osg/CullStack>
#include <osgDB/Registry>
#include <osg/ProxyNode>


using namespace osg;


MSpaceNode::MSpaceNode()
{
};

MSpaceNode::~MSpaceNode()
{
}

bool MSpaceNode::addChild(Node* child)
{
    if (Group::addChild(child))
    {
        return true;
    }
    return false;
}

bool MSpaceNode::removeExpiredChildren(double expiryTime, unsigned int expiryFrame, NodeList& removedChildren)
{

    if (_children.size() > _numChildrenThatCannotBeExpired)
    {
        unsigned cindex = _children.size() - 1;
        if (!_perRangeDataList[0]._filename.empty() &&
            _perRangeDataList[0]._timeStamp + _perRangeDataList[0]._minExpiryTime < expiryTime &&
            _perRangeDataList[0]._frameNumber + _perRangeDataList[0]._minExpiryFrames < expiryFrame)
        {
            osg::Node* nodeToRemove = _children[cindex].get();
            removedChildren.push_back(nodeToRemove);
            return Group::removeChildren(cindex, 1);
        }
    }

    return false;
}

void MSpaceNode::traverse(osg::NodeVisitor& nv)
{
    // set the frame number of the traversal so that external nodes can find out how active this
    // node is.
    if (nv.getFrameStamp() &&
        nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
    {
        setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
    }

    double timeStamp = nv.getFrameStamp() ? nv.getFrameStamp()->getReferenceTime() : 0.0;
    unsigned int frameNumber = nv.getFrameStamp() ? nv.getFrameStamp()->getFrameNumber() : 0;
    bool updateTimeStamp = nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR;

    switch (nv.getTraversalMode())
    {
    case(NodeVisitor::TRAVERSE_ALL_CHILDREN):
        std::for_each(_children.begin(), _children.end(), NodeAcceptOp(nv));
        break;
    case(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
    {
        float required_range = 0;
        if (_rangeMode == DISTANCE_FROM_EYE_POINT)
        {
            required_range = nv.getDistanceToViewPoint(getCenter(), true);
        }
        else
        {
            osg::CullStack* cullStack = nv.asCullStack();
            if (cullStack && cullStack->getLODScale() > 0.0f)
            {
                required_range = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
            }
            else
            {
                // fallback to selecting the highest res tile by
                // finding out the max range

                required_range = osg::maximum(required_range, _rangeList[0].first);
            }
        }

        bool needToLoadChild = false;
        if (_rangeList[0].first <= required_range && required_range < _rangeList[0].second)
        {
            if (_children.size() == _numChildrenThatCannotBeExpired)
            {
                //for (unsigned int i = 0; i < _children.size(); i++)
                //{
                //    _children[i]->accept(nv);
                //}
                needToLoadChild = true;
            }
            else
            {
                if (updateTimeStamp)
                {
                    _perRangeDataList[0]._timeStamp = timeStamp;
                    _perRangeDataList[0]._frameNumber = frameNumber;
                }
                for (unsigned int i = 0; i < _children.size(); i++)
                {
                    _children[i]->accept(nv);
                }
            }
        }

        if (needToLoadChild)
        {
            // now request the loading of the next unloaded child.
            if (!_disableExternalChildrenPaging &&
                nv.getDatabaseRequestHandler())
            {
                // compute priority from where abouts in the required range the distance falls.
                float priority = (_rangeList[0].second - required_range) / (_rangeList[0].second - _rangeList[0].first);

                // invert priority for PIXEL_SIZE_ON_SCREEN mode
                if (_rangeMode == PIXEL_SIZE_ON_SCREEN)
                {
                    priority = -priority;
                }

                // modify the priority according to the child's priority offset and scale.
                priority = _perRangeDataList[0]._priorityOffset + priority * _perRangeDataList[0]._priorityScale;

                if (_databasePath.empty())
                {
                    nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[0]._filename, nv.getNodePath(), priority, nv.getFrameStamp(), _perRangeDataList[0]._databaseRequest, _databaseOptions.get());
                }
                else
                {
                    // prepend the databasePath to the child's filename.
                    nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath + _perRangeDataList[0]._filename, nv.getNodePath(), priority, nv.getFrameStamp(), _perRangeDataList[0]._databaseRequest, _databaseOptions.get());
                }
            }

        }
        break;
    }
    default:
        break;
    }
}