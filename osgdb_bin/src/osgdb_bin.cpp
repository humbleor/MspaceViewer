#include "../include/osgdb_bin.h"
#include "../../MSpaceOctree/modules/unsuck/unsuck.hpp"

osgDB_bin::osgDB_bin()
{
}

osgDB_bin::~osgDB_bin()
{
}

const char* osgDB_bin::className() const
{
	return "bin Reader/Writer";
}

bool osgDB_bin::acceptsExtension(const std::string& extension) const
{
	return osgDB::equalCaseInsensitive(extension, "bin");
}

osgDB_bin::ReadResult osgDB_bin::readNode(const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
{
    // byteOffset[0], byteSize[1],  bpp[2],
    //posscale.x[3],posscale.y[4],posscale.z[5],
    //posoffset.x[6],posoffset.y[7],posoffset.z[8]
    //hascolor[9], rgbOffset[10],path[11];
    std::string fileinformation[12];

    std::istringstream out(fileName);
    unsigned int index = 0;
    std::string str;
    while (std::getline(out, str, '|') && index < 12)
    {
        fileinformation[index] = str;
        index++;
    }

    //得到文件后缀
    std::string _ext = osgDB::getLowerCaseFileExtension(fileinformation[11]);
    //判断是否支持该文件
    if (!acceptsExtension(_ext))
        return ReadResult::FILE_NOT_HANDLED;
    //创建一个存储数据的节点
    osg::ref_ptr<osg::Geode> _geode = new osg::Geode();

    osg::ref_ptr<osg::Vec3Array> _coordinates = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> _colors = new osg::Vec4Array();

    int64_t byteOffset = stoull(fileinformation[0]);
    int64_t byteSize = stoull(fileinformation[1]);
    auto buffer = readBinaryFile(fileinformation[11], byteOffset, byteSize);
    int bpp = stoi(fileinformation[2]);
    int numpoints = buffer.size() / bpp;



    for (int64_t i = 0; i < numpoints; i++) {
        int64_t pointoffset = i * bpp;

        int32_t ix = read<int32_t>(buffer, pointoffset + 0);
        int32_t iy = read<int32_t>(buffer, pointoffset + 4);
        int32_t iz = read<int32_t>(buffer, pointoffset + 8);

        double posscalex = stod(fileinformation[3]);
        double posscaley = stod(fileinformation[4]);
        double posscalez = stod(fileinformation[5]);

        double posoffsetx = stod(fileinformation[6]);
        double posoffsety = stod(fileinformation[7]);
        double posoffsetz = stod(fileinformation[8]);

        double x = double(ix) * posscalex + posoffsetx;
        double y = double(iy) * posscaley + posoffsety;
        double z = double(iz) * posscalez + posoffsetz;
        _coordinates->push_back(osg::Vec3(x, y, z));
        float r, g, b;
        bool hascolor = stoi(fileinformation[9]);
        if (hascolor)
        {
            int64_t rgbOffset = stoull(fileinformation[10]);
            r = (float)read<uint16_t>(buffer, pointoffset + rgbOffset + 0) / 255 / 255;
            g = (float)read<uint16_t>(buffer, pointoffset + rgbOffset + 2) / 255 / 255;
            b = (float)read<uint16_t>(buffer, pointoffset + rgbOffset + 4) / 255 / 255;

        }
        else
        {
            r = 0.0f;
            g = 1.0f;
            b = 0.0f;
        }

        _colors->push_back(osg::Vec4(r, g, b, 1.0f));
    }

    osg::ref_ptr<osg::Geometry> _geometry = new osg::Geometry();
    _geometry->setVertexArray(_coordinates);
    _geometry->setColorArray(_colors, osg::Array::BIND_PER_VERTEX);
    _geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, numpoints));
    _geode->addDrawable(_geometry);
    return _geode;
}

osgDB_bin::WriteResult osgDB_bin::writeNode(const osg::Node& node, const std::string& fileName, const osgDB::ReaderWriter::Options* options) const
{
    return WriteResult();
}
