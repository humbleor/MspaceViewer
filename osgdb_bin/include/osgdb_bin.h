#pragma once
#include "osgdb_bin_global.h"

#include <osg/Notify>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Group>

#include <osgDB/ReaderWriter>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

class OSGB_BIN_EXPORT osgDB_bin:public osgDB::ReaderWriter
{
public:
    osgDB_bin();
    ~osgDB_bin();

    //插件类名称
    const char* className() const;

    //检查插件是否支持扩展名
    bool acceptsExtension(const std::string& extension) const;

    //读取节点
    ReadResult readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options)const;
    WriteResult writeNode(const osg::Node& node, const std::string& fileName, const osgDB::ReaderWriter::Options* options)const;

};
//插件注册，定义全局变量
REGISTER_OSGPLUGIN(bin, osgDB_bin);