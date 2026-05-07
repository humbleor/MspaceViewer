
#include <string>
#include <osg/PagedLOD>
#include <osgDB/ReadFile>

#include "../libs/laszip/laszip_api.h"

#include "../libs/json/json.hpp"



#include "Attributes.h"
#include "../modules/unsuck/unsuck.hpp"
#include "Vector3.h"
#include "MSpaceNode.h"

using std::string;
using json = nlohmann::json;

namespace potree_to_osg {

struct Point {
    double x;
    double y;
    double z;
    uint16_t r;
    uint16_t g;
    uint16_t b;
};
enum NodeType {
    NORMAL = 0,
    LEAF = 1,
    PROXY = 2,
};
struct Node {

    string name = "";
    Vector3 min;
    Vector3 max;

    vector<shared_ptr<Node>> children = vector<shared_ptr<Node>>(8, nullptr);
    shared_ptr<Node> parent = nullptr;
    int32_t nodeType = -1;
    int64_t byteOffset = 0;
    int64_t byteSize = 0;
    int64_t numPoints = 0;

    int level() {
        return name.size() - 1;
    }

    void traverse(function<void(Node*, int level)> callback, int level = 0) {

        callback(this, level);

        for (auto child : children) {
            if (child != nullptr) {
                child->traverse(callback, level + 1);
            }
        }

    }
};

inline pair<Vector3,Vector3> childAABB(pair<Vector3,Vector3> aabb, int& index) {

    Vector3 min = aabb.first;
    Vector3 max = aabb.second;

    auto size = max-min;

    if ((index & 0b0001) > 0) {
        min.z += size.z / 2;
    } else {
        max.z -= size.z / 2;
    }

    if ((index & 0b0010) > 0) {
        min.y += size.y / 2;
    } else {
        max.y -= size.y / 2;
    }

    if ((index & 0b0100) > 0) {
        min.x += size.x / 2;
    } else {
        max.x -= size.x / 2;
    }

    return { min, max };
}

inline Attributes getAttributes(json& jsMetadata) {

    vector<Attribute> attributeList;
    auto jsAttributes = jsMetadata["attributes"];

    for (auto jsAttribute : jsAttributes) {

        string name = jsAttribute["name"];
        string description = jsAttribute["description"];
        int size = jsAttribute["size"];
        int numElements = jsAttribute["numElements"];
        int elementSize = jsAttribute["elementSize"];
        AttributeType type = typenameToType(jsAttribute["type"]);

        Attribute attribute(name, size, numElements, elementSize, type);

        attributeList.push_back(attribute);
    }

    double scaleX = jsMetadata["scale"][0];
    double scaleY = jsMetadata["scale"][1];
    double scaleZ = jsMetadata["scale"][2];

    double offsetX = jsMetadata["offset"][0];
    double offsetY = jsMetadata["offset"][1];
    double offsetZ = jsMetadata["offset"][2];

    Attributes attributes(attributeList);
    attributes.posScale = { scaleX, scaleY, scaleZ };
    attributes.posOffset = { offsetX, offsetY, offsetZ };

    return attributes;
}
inline void loadHierarchyRecursive(string path, osg::ref_ptr<osg::MSpaceNode> lodStructure, int64_t offset, int64_t size,Vector3 posscale, Vector3 posoffset,bool hascolor, int64_t rgbOffset,int byte)
{
    string hierarchyPath = path + "/hierarchy.bin";
    auto data = readBinaryFile(hierarchyPath, offset, size);

    int64_t bytesPerNode = 22;
    int64_t numNodes = size / bytesPerNode;

    vector<osg::ref_ptr<osg::MSpaceNode>> MSpaceNodes;
    MSpaceNodes.reserve(numNodes);
    MSpaceNodes.push_back(lodStructure);
    //path[0], byteOffset[1], byteSize[2],  bpp[3],
    //posscale.x[4],posscale.y[5],posscale.z[6],
    //posoffset.x[7],posoffset.y[8],posoffset.z[9]
    //hascolor[10], rgbOffset[11];
    for (int i = 0; i < numNodes; i++) {

        auto currentNode = MSpaceNodes[i];

        uint64_t offsetNode = i * bytesPerNode;
        uint8_t type = data[offsetNode + 0];
        int32_t childMask = data[offsetNode + 1];
        int64_t byteOffset = read<int64_t>(data, offsetNode + 6);
        int64_t byteSize = read<int64_t>(data, offsetNode + 14);
        
        string filename = std::to_string(byteOffset) + "|" + std::to_string(byteSize) + "|" + std::to_string(byte)
            + "|" + std::to_string(posscale.x) + "|" + std::to_string(posscale.y) + "|" + std::to_string(posscale.z)
            + "|" + std::to_string(posoffset.x) + "|" + std::to_string(posoffset.y) + "|" + std::to_string(posoffset.z)
            + "|" + std::to_string(hascolor) + "|" + std::to_string(rgbOffset) + "|" + path + "/octree.bin";
        currentNode->setFileName(0, filename);


        if (type == potree_to_osg::NodeType::PROXY) {
            loadHierarchyRecursive(path, currentNode, byteOffset, byteSize,posscale,posoffset,hascolor,rgbOffset,byte);
        }
        else {
            // load child node data for current node
            for (int32_t childIndex = 0; childIndex < 8; childIndex++) {
                bool childExists = ((1 << childIndex) & childMask) != 0;

                if (!childExists) {
                    continue;
                }
                osg::ref_ptr<osg::MSpaceNode> childPagedNode = new osg::MSpaceNode;

                childPagedNode->setRangeMode(osg::LOD::RangeMode::PIXEL_SIZE_ON_SCREEN);
                childPagedNode->setRange(0, 100, FLT_MAX);
                currentNode->addChild(childPagedNode);
                MSpaceNodes.push_back(childPagedNode);

            }
            currentNode->setNumChildrenThatCannotBeExpired(currentNode->getNumChildren());
        }
    }
}
inline void loadHierarchy(string path, json& metadata, osg::ref_ptr<osg::MSpaceNode> lodStructure,Attributes attributes) {


    auto jsHierarchy = metadata["hierarchy"];

    Vector3 min, max;
    {
        min.x = metadata["boundingBox"]["min"][0];
        min.y = metadata["boundingBox"]["min"][1];
        min.z = metadata["boundingBox"]["min"][2];

        max.x = metadata["boundingBox"]["max"][0];
        max.y = metadata["boundingBox"]["max"][1];
        max.z = metadata["boundingBox"]["max"][2];
    }
    //lodStructure->setCenterMode(osg::LOD::USER_DEFINED_CENTER);
    lodStructure->setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
    lodStructure->setRange(0, 100, FLT_MAX);
    lodStructure->setInitialBound(osg::BoundingSphere(osg::Vec3((max - min).x * 0.5, (max - min).y * 0.5, (max - min).z * 0.5), max.distanceTo(min) * 0.5));
    //lodStructure->setCenter(osg::Vec3((max - min).x * 0.5, (max - min).y * 0.5, (max - min).z * 0.5));
    //lodStructure->setRadius(max.distanceTo(min) *0.5);


    int64_t rgbOffset = 0;
    int64_t rgbOffsetFind = 0;
    bool hascolor = false;
    for (Attribute attribute : attributes.list) {
        if (attribute.name == "rgb") {
            rgbOffset = rgbOffsetFind;
            hascolor = true;
            break;
        }

        rgbOffsetFind += attribute.size;
    }
    int64_t offset = 0;
    int64_t firstChunkSize = jsHierarchy["firstChunkSize"];
    loadHierarchyRecursive(path, lodStructure, offset, firstChunkSize,attributes.posScale,attributes.posOffset,hascolor, rgbOffset,attributes.bytes);

}

 inline void save(string target, vector<Point>& points, Vector3 min, Vector3 max) {
    laszip_POINTER laszip_writer;
    laszip_point* laszip_point;
    laszip_header* header;

    laszip_create(&laszip_writer);
    laszip_get_header_pointer(laszip_writer, &header);

    header->version_major = 1;
    header->version_minor = 4;
    header->header_size = 375;
    header->offset_to_point_data = header->header_size;
    header->point_data_format = 2;
    header->point_data_record_length = 26;
    header->number_of_point_records = points.size();
    header->x_scale_factor = 0.001;
    header->y_scale_factor = 0.001;
    header->z_scale_factor = 0.001;
    header->x_offset = 0.0;
    header->y_offset = 0.0;
    header->z_offset = 0.0;
    header->min_x = min.x;
    header->min_y = min.y;
    header->min_z = min.z;
    header->max_x = max.x;
    header->max_y = max.y;
    header->max_z = max.z;

    header->extended_number_of_point_records = points.size();





    laszip_open_writer(laszip_writer, target.c_str(), false);

    laszip_get_point_pointer(laszip_writer, &laszip_point);

    double coordinates[3];

    for (Point point : points) {

        coordinates[0] = point.x;
        coordinates[1] = point.y;
        coordinates[2] = point.z;

        laszip_point->rgb[0] = point.r;
        laszip_point->rgb[1] = point.g;
        laszip_point->rgb[2] = point.b;

        //laszip_set_point(laszip_writer, laszip_point);
        laszip_set_coordinates(laszip_writer, coordinates);
        //laszip_set_point(laszip_writer, laszip_point);
        laszip_write_point(laszip_writer);
    }

    laszip_close_writer(laszip_writer);
    laszip_destroy(laszip_writer);
}

 inline void convert(string path,osg::ref_ptr<osg::MSpaceNode> lodStructure) {

     string metadataPath = path + "/metadata.json";


     string strMetadata = readTextFile(metadataPath);
     json jsMetadata = json::parse(strMetadata);


     auto attributes = getAttributes(jsMetadata);

     loadHierarchy(path, jsMetadata, lodStructure, attributes);

}

}
