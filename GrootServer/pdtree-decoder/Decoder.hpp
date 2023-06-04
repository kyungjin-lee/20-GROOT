#pragma once


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <vector>
#include <thread>

#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <time.h>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>

#include "JpegDecoder.hpp"

using namespace std;
using PointType = pcl::PointXYZRGB; 
#define MAX(a,b) \
    ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MIN(a,b) \
    ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


struct Manifest{
    int key_frame_interval;
    float max_breadth_depth_sidelength;
};


// PDTree encoded header
struct RxFrameHeader{
    char frame_type;
    int num_points;
    int num_breadth_bytes;
    int num_breadth_nodes;
    int num_depth_bytes;
    int num_color_bytes;
    int num_icp_nodes;
    int num_icp_points;
    float root_center[3];
    float root_sidelength;
};


struct RxFrame{
    RxFrameHeader header;
    uint8_t* breadth_bytes;
    uint8_t* depth_bytes;
    uint8_t* breadth_leaf_num;
    uint8_t* color_bytes;
};


struct RenderFrame{

    char frame_type;
    int num_points;

    Eigen::Vector4f* center_list;
    uint8_t* depth_bytes;
    uint8_t* color_bytes;

};

struct DecodeDepthThreadArgs{
    int cnt;
    uint8_t* depth_bytes;
    uint8_t* decoded_bytes;
};

class Decoder
{
    public:
        Decoder(){}
        ~Decoder(){}

        void readManifest(std::string filename, Manifest manifest);
        void decodePDTreeFile(std::string filename);
        void generatePointCloud(std::string filename, Manifest manifest);
        void write_decode(std::string filename);

    private:
        void decodeBreadthBytes(RxFrameHeader header, uint8_t* breadth_bytes, uint8_t* breadth_leaf_num);


        void decodeDepthBytes(int num_points, int num_depth_bytes, uint8_t* depth_bytes);
        void decodeColorBytes(int num_points, int num_color_bytes, uint8_t* color_bytes);

        Eigen::Vector4f getChildCenter(Eigen::Vector4f parentCenter, float sidelen, uint8_t idx);

        void getDepthBytes(int cnt, vector<uint8_t>* current_input, vector<uint8_t>* current_output);    

        void test_sum(int cnt, vector<uint8_t>* current_input);
        RenderFrame render_frame_;
};


