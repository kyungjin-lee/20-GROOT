#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <bitset>
#include <chrono>
#include <math.h>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/common/transforms.h>
#include <zstd.h>


using namespace std;
using PointType = pcl::PointXYZRGB;


struct Color
{
    int index;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    Color():index(0), r(0), g(0), b(0) {}
};


struct FrameHeader
{
    char frame_type;
    int num_points;
    int num_breadth_bytes;
    int num_breadth_nodes; 
    int num_depth_bytes;
    int num_color_bytes;
    int num_icp_nodes;
    int num_icp_points;
    Eigen::Vector3f root_center;
    float root_sidelength;
};

struct FramePayload
{
    vector<uint8_t> breadth_bytes;
    vector<uint8_t> depth_bytes;
    vector<uint8_t> breadth_leaf_indices;
    vector<uint8_t> color_bytes;
};

struct Resources
{
    uint8_t* fBuffer;
    uint8_t* cBuffer;
    size_t fBufferSize;
    size_t cBufferSize;
    ZSTD_CCtx* cctx;
};


class Frame
{
    public:
        Frame();
        ~Frame(){}



        void readPointCloud(int frame_index, std::string filename, float scale, bool isFlip, float x, float y, float z);
        void writeFrame(std::string filename, unsigned int* avgCompSize);
        void generateOctree(float voxelsize);


        void compressPDTree(int is_user_adaptive, bool isShort);

        vector<uint8_t> getColorBytes();
        pcl::PointCloud<PointType>::Ptr getPointCloud();
        //vector<PointType> getOrderedPoints();
        vector<vector<uint8_t>> getBreadthBytes();


        vector<Eigen::Vector4f> get_debug_point_list();
        vector<uint8_t> get_debug_colors();


        // generate payload with or without color
        void generatePayload();
        void generatePayload(vector<uint8_t> compressed_colors);

    protected:

        void initialize();   
        void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr out_cloud, float scale, bool isFlip,  float x, float y , float z);
        
        // functions to generate auxiliary information
        void generateLeafNodeIndices();
        void generateChildNodeIndices();

        // functions for saving compressed files
        void generateHeader(char type);
        void reorderDepthColor();
        void reorderDepthColorShort();

        // functions for efficient octree probing
        Eigen::Vector3f getChildCenter(Eigen::Vector3f parentCenter,float sidelen, int idx);
        void getNodesInOctreeContainer(pcl::octree::OctreeNode* currentNode, std::vector<int>& indices);

        // functions to compress breadth bytes
        void compressBreadthBytes();
        void compressBreadthBytesShort();
        // functions to compress depth bytes
        void compressDepthBytes(int is_user_adaptive);
        void compressDepthBytesZstd(vector<uint8_t> depth_bytes);


        // variables

        int frame_index_ = 0;
        pcl::PointCloud<PointType>::Ptr cloud_;
        pcl::octree::OctreePointCloud<PointType> octree_;
        int octree_depth_ = 0;
        int max_breadth_depth_ = 0;
        int num_points_ = 0;
        int orig_num_points_= 0;

        Eigen::Vector3f root_center_;
        float root_sidelength_;

        vector<vector<uint8_t>> breadth_bytes_;
        vector<uint8_t> depth_list_;
        vector<Color> color_list_;

        vector<int> point_indices_;


        vector<uint8_t> compressed_depth_bytes_;

        FrameHeader header_;  
        FramePayload payload_;




        //For debuggin purpose
        vector<uint8_t> debug_color_list_;
        vector<Eigen::Vector4f> debug_point_list_;


        vector<vector<int>> leaf_indices_;
        vector<vector<int>> child_indices_;

        Resources ress_;
        vector<uint8_t> depth_bytes_zstd_;
};
