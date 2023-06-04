//
//  TcpStream.h
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/07/15.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//


#ifndef TcpStream_h
#define TcpStream_h

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

#include <time.h>


#include "LibJpeg/include/turbojpeg.h"
#include <simd/simd.h>
#include "LibZstd/zstd.h"
#include "ShaderTypes.h"

#define MAX(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })
  
#define MIN(a,b) \
({ __typeof__ (a) _a = (a); \
__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })

typedef struct{
    int key_frame_interval;
    float max_breadth_depth_sidelength;
} Manifest;


//200316 TODO struct header changed change server side
typedef struct{
    int totalSize;
    char frame_type;
    int num_points;
    int num_breadth_bytes;
    int num_breadth_nodes;
    int num_depth_bytes;
    int num_color_bytes;
    //These two added for iframe set to zero

    int num_icp_nodes;
    int num_icp_points;
    float root_center[3];
    float root_sidelength;
} RxFrameHeader;


typedef struct{
    RxFrameHeader header;
    uint8_t* breadth_bytes;
    uint8_t* depth_bytes;
    uint8_t* breadth_leaf_num;
    uint8_t* color_bytes;

} RxFrame;



typedef struct{
    char frame_type;
    int num_points;
   
    
    //int num_depth_bytes;
    
    vector_float4* center_list;
   // uint8_t* raw_depth_bytes;
    uint8_t* depth_bytes;
    uint8_t* color_bytes;
    
} RenderFrame;

typedef struct{
    int index;
    int cnt;
    uint8_t* depth_bytes;
    uint8_t* decoded_bytes;
} DecodeDepthThreadArgs;


int tcp_connect(const char *ipaddr, int port);

void receive_manifest(Manifest* manifest);


void cReceive(int socket, RxFrame* rxFrame);
void receive_payload(int socket, RxFrame* rxFrame);
void cSend_viewAck(int socket, int chunkIdx, matrix_float4x4 viewMat, matrix_float4x4 projMat);

void cReadFrameFromFile(const char *fileName, float rootTransform, RenderFrame* renderFrame);
void cGenerateMortonCode();

char cDecode(RxFrame* rxFrame, RenderFrame* renderFrame);

int decode_breadth_bytes(RxFrameHeader header, uint8_t* breadth_bytes, uint8_t* breadth_leaf_num, RenderFrame* renderFrame);

int decode_depth_bytes(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame);

void decode_depth_bytes_serial(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame);
void decode_depth_bytes_parallel(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame);
void decode_depth_bytes_gpu(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame);
void decode_depth_bytes_zstd(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame);



int decode_color_bytes(int num_points, int num_color_bytes, uint8_t* color_bytes, RenderFrame* renderFrame);

vector_float4 get_child_center(vector_float4 parentCenter, float sidelen, uint8_t idx);

#endif
