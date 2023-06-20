#include <cstdio>
#include <iostream>
#include <bitset>
#include <fstream>
#include <sstream>
#include <chrono>
#include <math.h>

#include "Frame.hpp"
#include "JpegEncoder.hpp"


// color coding order read from file
int* image_coder_order_;

void compressColors(vector<uint8_t> orig_colors, vector<uint8_t>& compressed_colors, JpegEncoder* jpegEncoder_)
{
    int color_size = orig_colors.size();
    int image_width = 1024;
    int image_height = 1024;

    if(orig_colors.size()/3 < 1024*512)
    {
        image_height = 512;
    }
    else if(orig_colors.size() / 3 >= 1024 * 1024)
    {
        image_width = 2048;
    }

    vector<uint8_t> reordered_color(image_width * image_height *3, 255);
    for(int i = 0 ; i < image_width * image_height; i++)
    {
        // for GROOT use this
        int idx = image_coder_order_[i];
        // for SERIAL use this
        // int idx = i;
        if(idx < orig_colors.size()/3)
        {
            reordered_color[3*i] = orig_colors[3*idx];
            reordered_color[3*i+1] = orig_colors[3*idx+1];
            reordered_color[3*i+2] = orig_colors[3*idx+2];
        }
    }  
    
    printf("COmpress color\n");
    //    compressed_colors.resize(orig_colors.size());

    /* for no compression use this
    compressed_colors.resize(reordered_color.size());
    memcpy(&compressed_colors[0], &reordered_color[0], reordered_color.size());*/

    // for GROOT
    jpegEncoder_->encode(reordered_color, compressed_colors, image_width, image_height);

    // FILE* pFile = fopen("test_output_groot.jpeg", "wb");
    // fwrite(&compressed_colors[0], sizeof(uint8_t), compressed_colors.size(), pFile);
 
}


void readImageCoder(string imagecoder)
{
    int imgSize = 1024;
    image_coder_order_ = new int [imgSize * imgSize];
    FILE* pFile = fopen(imagecoder.c_str(), "rb");
    fread(&image_coder_order_[0], sizeof(int), imgSize * imgSize, pFile);
    fclose(pFile);
} 

int main(int argc, char* argv[])
{
    std::string dataset_name = argv[1];
    std::string inputpath = argv[2];
    std::string morton_code = argv[3];
    std::string outputpath = argv[4];
    // 0 == encoding only, 1 == user adaptive
    int is_user_adaptive = atoi(argv[5]);

    // read in color coder file
    readImageCoder(morton_code);

    // set transformation and voxel size for input dataset
    float scale = 1.0;
    bool isFlip = false;
    float x = 0;
    float y = 0;
    float z = 0;
    float voxelSize = 0;
    // if isshort ODB is 2 bytes not 3 bytes
    bool isShort = false;
    if(dataset_name == "twopeople")
    {
        voxelSize= 0.001;
    }
    else if(dataset_name == "longdress")
    {
        scale = 0.001;
        voxelSize = 0.001;
        //isShort = true;
    }
    else if(dataset_name == "panoptic")
    {
        voxelSize = 0.001;
        y = -1;
        isFlip = true;
    }

    // prepare file list
    vector<std::string> filelist;
    vector<std::string> outfilelist;
    for(auto& p : boost::filesystem::directory_iterator(inputpath)){
        std::string filename = inputpath + p.path().filename().string();
        std::string outputfile = outputpath + p.path().stem().string();
        filelist.push_back(filename);
        outfilelist.push_back(outputfile);
    }

    sort(filelist.begin(), filelist.end());
    sort(outfilelist.begin(), outfilelist.end());

    // prepare jpeg encoder
    JpegEncoder* jpegEncoder_;
    jpegEncoder_ = new JpegEncoder();
    unsigned int avgCompSize = 0;
    for(int i = 0; i < filelist.size() ; i++)
    {
        Frame currentFrame;
        cout << "[MAIN] File name " << filelist[i] << endl;
        currentFrame.readPointCloud(i, filelist[i], scale, isFlip,  x, y, z);
        printf("[MAIN] Finished reading point cloud\n");
        currentFrame.generateOctree(voxelSize);
        printf("[MAIN] Finished generating octree\n");
        currentFrame.compressPDTree(is_user_adaptive, isShort);
        printf("[MAIN] Finished pdtree compression\n");
        std::string out_file = outfilelist.at(i);
        if(is_user_adaptive)
        {
            currentFrame.generatePayload();
            out_file = out_file + "_useradaptive.bin";
        }
        else
        {
            printf("[MAIN] Finished geometry compression\n");
            // TODO move color compression to Frame class
            vector<uint8_t> colors = currentFrame.getColorBytes();
            vector<uint8_t> compressed_colors;
            compressColors(colors, compressed_colors, jpegEncoder_);
            currentFrame.generatePayload(compressed_colors);
            out_file = out_file + "_enc.bin";
        }
        printf("[MAIN] write frame\n");
        currentFrame.writeFrame(out_file,  &avgCompSize);
    }

    printf("[MAIN] Final Compressed Size: %d\n", avgCompSize / filelist.size());
}


