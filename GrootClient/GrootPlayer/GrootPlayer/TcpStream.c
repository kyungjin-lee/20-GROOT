//
//  TcpStream.c
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/07/15.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//

/*
 This code is shared between online version and offline player version
 When testing with the offline version ignore all network related codes
 */

#include "TcpStream.h"


#define MAX_THREAD 2

#define IMAGE_HEIGHT 1024

int* morton_list = NULL;
tjhandle handle;

void initialize_decoder()
{
    handle =   tjInitDecompress();
    if(handle < 0) printf("Can't initialize decoder\n");
    
    
}

// generate Morton code once in the beginning
// Use it as LUT
void cGenerateMortonCode()
{
    int* morton_image[IMAGE_HEIGHT];
    for(int i= 0 ; i < IMAGE_HEIGHT ; i++)
    {
        morton_image[i] = (int *)malloc(IMAGE_HEIGHT * sizeof(int));
        memset(&morton_image[i][0], 0,IMAGE_HEIGHT * 4);
    }
    int start_x = 0;
    int start_y = 0;
    morton_image[0][0]  = 0;
    morton_image[0][1] = 1;
    morton_image[1][0] = 2;
    morton_image[1][1] = 3;
    for(int i = 1 ; i < 10; i++)
    {
        int offset = pow(2,i);
        start_x = 0;
        start_y = offset;
        
        for(int m = 0 ; m < offset  ; m++)
        {
            for(int n = 0 ; n < offset ; n++)
            {
                morton_image[start_x + m][start_y + n] = morton_image[start_x+m][start_y+n - offset] + offset * offset;
            }
        }
        
        start_x = offset;
        start_y = 0;
        for(int m = 0 ; m < offset ; m++)
        {
            for(int n = 0 ; n < offset ; n++)
            {
                morton_image[start_x + m][start_y + n] = morton_image[start_x + m - offset][start_y + n] + 2*offset * offset;
            }
        }
        
        
        start_x = offset;
        start_y = offset;
        
        for(int m = 0 ; m < offset ; m++)
        {
            for(int n = 0 ; n < offset ; n++)
            {
                morton_image[start_x + m][start_y + n] = morton_image[start_x + m - offset][start_y + n - offset] + 3*offset * offset;
            }
        }
        
        
    }
    
    morton_list = (int*)malloc(IMAGE_HEIGHT * IMAGE_HEIGHT * sizeof(int));
    
    for(int i = 0 ; i < IMAGE_HEIGHT ; i++)
    {
        memcpy(morton_list + i * IMAGE_HEIGHT, morton_image[i], IMAGE_HEIGHT * sizeof(int));
    }
    
    
    for(int i= 0 ; i < IMAGE_HEIGHT ; i++)
    {
        free(morton_image[i]);
    }
    
    initialize_decoder();
    
}


int tcp_connect(const char *ipaddr, int port)
{
    int socket_ = socket(PF_INET, SOCK_STREAM, 0);
    printf("KJLEE socket %d\n",socket_);
    
    if((socket_ = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        printf("[KJLEE] make new socket failed\n");
        return -1;
        
    }
    
    int port_num = port;
    const char *IP_String = ipaddr;
    
    struct sockaddr_in Server_address;
    memset(&Server_address, 0, sizeof(Server_address));
    Server_address.sin_family = AF_INET;
    Server_address.sin_port = htons(port_num);
    Server_address.sin_addr.s_addr = inet_addr(IP_String); //inet_addr("224.0.0.1");
    
    if(connect(socket_, (struct sockaddr*) &Server_address, sizeof(Server_address)) == -1)
    {
        printf("[KJLEE] Cannot bind socket to %s, %d", IP_String, port_num);
        return -1;
    }else{
        printf("[KJLEE] Successfully binded socket to %s:%d", IP_String, port_num);
        return socket_;
    }
}
void receive_manifest(Manifest* manifest)
{
    
    manifest->key_frame_interval = 1;
    manifest->max_breadth_depth_sidelength = 0.008;
    cGenerateMortonCode();
}

void cReceive(int socket, RxFrame* rxFrame)
{
    
    int readlen = (int) recv(socket, &rxFrame->header, sizeof(RxFrameHeader), 0);
    assert(readlen == sizeof(RxFrameHeader));
    
    /* For debugging */
    /*printf("[TcpStream]======RxFrameHeader=====\n");
     printf("[TcpStream] frame type %c\n", rxFrame->header.frame_type);
     printf("[TcpStream] num points %d\n", rxFrame->header.num_points);
     printf("[TcpStream] num breadth bytes : %d\n", rxFrame->header.num_breadth_bytes);
     printf("[TcpStream] num breadth nodes : %d\n", rxFrame->header.num_breadth_nodes);
     printf("[TcpStream] num depth bytes : %d\n", rxFrame->header.num_depth_bytes);
     printf("[TcpStream] num color bytes : %d\n", rxFrame->header.num_color_bytes);
     printf("[TcpStream] root center : %f %f %f\n", rxFrame->header.root_center[0], rxFrame->header.root_center[1], rxFrame->header.root_center[2]);
     printf("[TcpStream] root sidelength : %f\n", rxFrame->header.root_sidelength);
     
     printf("[TcpStream]========================\n");*/
    
    
    receive_payload(socket, rxFrame);
    
}
void cReadFrameFromFile(const char *fileName, float rootTransform, RenderFrame* renderFrame)
{
    // Read GROOT header
    RxFrame* rxFrame = (RxFrame*)malloc(sizeof(RxFrame));
    FILE* pFile = fopen(fileName, "rb");
    RxFrameHeader rxHeader;
    fread(&rxHeader, 1, sizeof(RxFrameHeader), pFile);
    rxFrame->header = rxHeader;
    rxFrame->header.root_center[2] -= rootTransform;
    
    /* For debugging */
    /*printf("======RxFrameHeader=====\n");
     printf("frame type %c\n", rxFrame->header.frame_type);
     printf("num points %d\n", rxFrame->header.num_points);
     printf("num breadth bytes : %d\n", rxFrame->header.num_breadth_bytes);
     printf("num breadth nodes : %d\n", rxFrame->header.num_breadth_nodes);
     printf("num depth bytes : %d\n", rxFrame->header.num_depth_bytes);
     printf("num color bytes : %d\n", rxFrame->header.num_color_bytes);
     printf("root center : %f %f %f\n", rxFrame->header.root_center[0], rxFrame->header.root_center[1], rxFrame->header.root_center[2]);
     printf("root sidelength : %f\n", rxFrame->header.root_sidelength);
     
     printf("========================\n");*/
    
    // Read body 
    rxFrame->breadth_bytes  = (uint8_t*)malloc(rxHeader.num_breadth_bytes * sizeof(uint8_t));
    rxFrame->depth_bytes = (uint8_t*)malloc(rxHeader.num_depth_bytes * sizeof(uint8_t));
    rxFrame->breadth_leaf_num = (uint8_t*)malloc(rxHeader.num_breadth_nodes * sizeof(uint8_t));
    rxFrame->color_bytes = (uint8_t*)malloc(rxHeader.num_color_bytes * sizeof(uint8_t));
    
    fread(rxFrame->breadth_bytes, rxHeader.num_breadth_bytes, sizeof(uint8_t), pFile);
    fread(rxFrame->depth_bytes, rxHeader.num_depth_bytes, sizeof(uint8_t), pFile);
    fread(rxFrame->breadth_leaf_num, rxHeader.num_breadth_nodes, sizeof(uint8_t), pFile);
    fread(rxFrame->color_bytes, rxHeader.num_color_bytes, sizeof(uint8_t), pFile);
    
    
    cDecode(rxFrame, renderFrame);
    
    free(rxFrame->breadth_bytes);
    free(rxFrame->depth_bytes);
    free(rxFrame->breadth_leaf_num);
    free(rxFrame->color_bytes);
    free(rxFrame);
    fclose(pFile);
    
}

void cSend_viewAck(int socket, int chunkIdx, matrix_float4x4 viewMat, matrix_float4x4 projMat)
{
    printf("[TCP] send view ack\n");
    float* send_buffer = (float*) malloc(33 * sizeof(float));
    send_buffer[0]= (float) chunkIdx;
    memcpy(send_buffer+1, &viewMat, sizeof(float) * 16);
    memcpy(send_buffer+17, &projMat, sizeof(float) * 16);
    
    send(socket, send_buffer, 33 * sizeof(float), 0);
    
}
void receive_payload(int socket,  RxFrame* rxFrame)
{
    int max_buffer_size = 32768;
    
    RxFrameHeader header = rxFrame->header;
    rxFrame->breadth_bytes  = (uint8_t*)malloc(header.num_breadth_bytes * sizeof(uint8_t));
    rxFrame->depth_bytes = (uint8_t*)malloc(header.num_depth_bytes * sizeof(uint8_t));
    rxFrame->breadth_leaf_num = (uint8_t*)malloc(header.num_breadth_nodes * sizeof(uint8_t));
    rxFrame->color_bytes = (uint8_t*)malloc(header.num_color_bytes * sizeof(uint8_t));
    
    int expectedLen = header.num_breadth_bytes;
    int offset = 0;
    int readlen = 0;
    do{
        readlen = (int) recv(socket, rxFrame->breadth_bytes + offset, MIN(expectedLen, max_buffer_size), 0);
        expectedLen -= readlen;
        offset += readlen;
    } while (expectedLen > 0);
    
    
    printf("Finish receiving breadth\n");
    expectedLen = header.num_depth_bytes;
    offset = 0;
    do{
        readlen = (int) recv(socket, rxFrame->depth_bytes + offset, MIN(expectedLen, max_buffer_size), 0);
        expectedLen -= readlen;
        offset += readlen;
    } while (expectedLen > 0);
    
    
    printf("Finish receiving depth\n");
    
    
    expectedLen = header.num_breadth_nodes ;
    offset = 0;
    do{
        readlen = (int) recv(socket, rxFrame->breadth_leaf_num + offset, MIN(expectedLen, max_buffer_size), 0);
        expectedLen -= readlen;
        offset += readlen;
    } while (expectedLen > 0);
    
    printf("Finish receiving breadth num\n");
    
    
    expectedLen = header.num_color_bytes;
    offset = 0;
    do {
        readlen = (int) recv(socket, rxFrame->color_bytes + offset, MIN(expectedLen, max_buffer_size), 0);
        expectedLen -= readlen;
        offset += readlen;
    } while(expectedLen > 0);
    
    
    
    
}

char cDecode(RxFrame* rxFrame, RenderFrame* renderFrame)
{
    
    
    
    clock_t begin_breadth = clock();
    //printf("start decoding breadth bytes\n");
    
    int numBreadthBytes = decode_breadth_bytes(rxFrame->header, rxFrame->breadth_bytes, rxFrame->breadth_leaf_num, renderFrame);
    
    
    
    //printf("Finished decoding breadth bytes\n");
    clock_t end_breadth = clock();
    long breadthTime = end_breadth - begin_breadth;
    clock_t begin_depth = clock();
    
    //decode_depth_bytes_zstd(rxFrame->header.num_points, rxFrame->header.num_depth_bytes, rxFrame->depth_bytes, renderFrame);
    int numDepthBytes = decode_depth_bytes(rxFrame->header.num_points, rxFrame->header.num_depth_bytes, rxFrame->depth_bytes, renderFrame);
    
    clock_t end_depth = clock();
    long depthTime = end_depth - begin_depth;
    /*  clock_t end_time = clock();
     long msec = end_time - begin_time;
     printf("[Decoder] decode time : %lu\n", (double)((double)msec / CLOCKS_PER_SEC));
     */
    clock_t begin_color = clock();
    int numColorBytes = decode_color_bytes(rxFrame->header.num_points, rxFrame->header.num_color_bytes, rxFrame->color_bytes, renderFrame);
    clock_t end_color = clock();
    long colorTime = end_color - begin_color;
    //  printf("[Decoder] breadth depth color : %f %f %f\n", (double)breadthTime/ CLOCKS_PER_SEC,  (double)depthTime/ CLOCKS_PER_SEC,  (double)colorTime/ CLOCKS_PER_SEC);
    // printf("[Decoder] Num : %d %d %d\n", numBreadthBytes, numDepthBytes, numColorBytes);
    
    
    
    
    renderFrame->frame_type = rxFrame->header.frame_type;
    renderFrame->num_points = rxFrame->header.num_points;
    
    if(renderFrame->num_points == 0)
    {
        return 0;
    }
    return 1;
    
}


int decode_breadth_bytes(RxFrameHeader header, uint8_t* breadth_bytes, uint8_t* breadth_leaf_num, RenderFrame* renderFrame)
{
    vector_float4* centers;
    centers = (vector_float4*) malloc(header.num_breadth_nodes*sizeof(vector_float4));
    vector_float4* next_centers;
    next_centers = (vector_float4*) malloc(header.num_breadth_nodes*sizeof(vector_float4));
    
    centers[0][0] = header.root_center[0];
    centers[0][1] = header.root_center[1];
    centers[0][2] = header.root_center[2];
    centers[0][3] = 1.0;
    
    
    
    int cntBytes = 0;
    int cntCenters = 1;
    float currentSideLength = header.root_sidelength;
    int cntDepth = 0;
    while(cntBytes < header.num_breadth_bytes - 1)
    {
        int cntNextCenters = 0;
        
        for(int i = 0; i < cntCenters ; i++)
        {
            uint8_t currentByte = *(breadth_bytes + cntBytes + i);
            for(int k = 0 ; k < 8 ; k++)
            {
                if(((currentByte >> k) & 1 )!= 0)
                {
                    vector_float4 child_center = get_child_center(centers[i], currentSideLength, k);
                    next_centers[cntNextCenters] = child_center;
                    cntNextCenters++;
                }
            }
            
        }
        cntBytes += cntCenters;
        cntCenters = cntNextCenters;
        memcpy(centers, next_centers, cntNextCenters * sizeof(vector_float4));
        currentSideLength = currentSideLength / 2;
        
        cntDepth ++;
    }
    
    renderFrame->center_list = (vector_float4*) malloc(header.num_points * sizeof(vector_float4));
    
    
    
    int cntNodes = 0;
    for(int i = 0 ; i < header.num_breadth_nodes  ; i++)
    {
        uint8_t currentNumNode = *(breadth_leaf_num + i);
        vector_float4 currentCenter = *(centers + i);
        for(int j = 0 ; j < currentNumNode ; j++)
        {
            renderFrame->center_list[cntNodes + j] = currentCenter;
        }
        cntNodes += currentNumNode;
    }
    
    
    
    free(centers);
    free(next_centers);
    
    return cntNodes;
}




void* get_depth_bytes(void* t_args)
{
    
    
    DecodeDepthThreadArgs* thread_args = (DecodeDepthThreadArgs*) t_args;
    for(int i = 0 ; i < thread_args->cnt; i++)
    {
        
        int current_val = (*(thread_args->depth_bytes + 3 * i) << 16) +(*(thread_args->depth_bytes+ 3 * i+1)  << 8) + *(thread_args->depth_bytes+3 * i+2) ;
        for(int j = 7 ; j >= 0 ; j--)
        {
            int localIdx =  (7-j) ;
            uint8_t temp = current_val / pow(2, j * 3);
            current_val -= temp * pow(2, j * 3);
            *(thread_args->decoded_bytes + 8 * i + localIdx) = temp;
        }
        
    }
    
    pthread_exit((void *) 0);
}

int decode_color_bytes(int num_points, int num_color_bytes, uint8_t* color_bytes, RenderFrame* renderFrame)
{
    renderFrame->color_bytes = (uint8_t*) malloc(num_points * 4 * sizeof(uint8_t));
    memset(renderFrame->color_bytes, 1, num_points * 4 * sizeof(uint8_t));
    
    
    
    int width = 0;
    int height = 0;
    int jpegSubsamp = 0;
    int jpegColorSpace = 0;
    
    
    int numCompressedBytes = num_color_bytes;
    
    width = 1024;
    height = 1024;
    if(num_color_bytes <= 1024 * 512 * 3)
    {
        height = 512;
    }
    
    
    
    
    tjDecompressHeader3(handle, color_bytes, numCompressedBytes, &width, &height, &jpegSubsamp,
                        &jpegColorSpace);
    
    
    unsigned char *imgBuf = (unsigned char *) malloc(width * height * 3);
    
    //printf("Jpeg decoder header (filesize , width, height, jpegSubsamp, jpegColorSpace) %d %d %d %d %d\n", num_color_bytes, width, height, jpegSubsamp, jpegColorSpace);
    if( tjDecompress2(handle, color_bytes, numCompressedBytes, imgBuf ,width,width * 3, height, jpegColorSpace,  0) < 0)
    {
        printf("decompress failed\n");
    }
    
    // for uncompressed use this comment above
    //memcpy(imgBuf, color_bytes, width * height * 3);
    
    
    
    
    int cntNode = 0;
    for(int i = 0 ; i < width*height ; i++)
    {
        // for GROOT use this
        int idx = morton_list[i];
        // for SERIAL use this
        //int idx = i;
        if(idx < num_points)
        {
            //  for GROOT
            renderFrame->color_bytes[4*idx+0] = imgBuf[3*i+2];
            renderFrame->color_bytes[4*idx+1] = imgBuf[3*i+1];
            renderFrame->color_bytes[4*idx+2] = imgBuf[3*i+0];
            //for ORIGINAL uncompressed
            //  renderFrame->color_bytes[4*idx+0] = imgBuf[3*i+0];
            //  renderFrame->color_bytes[4*idx+1] = imgBuf[3*i+1];
            //  renderFrame->color_bytes[4*idx+2] = imgBuf[3*i+2];
            
            cntNode++;
        }
        
        
    }
    
    free(imgBuf);
    
    return cntNode;
    
}

void decode_depth_bytes_zstd(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame)
{
    renderFrame->depth_bytes = (uint8_t*) malloc(num_points * 4 * sizeof(uint8_t));
    memset(renderFrame->depth_bytes, 1, num_points * 4 * sizeof(uint8_t));
    
    
    unsigned long long const rSize = ZSTD_getFrameContentSize(depth_bytes, num_depth_bytes);
    uint8_t* const rBuff = (uint8_t*) malloc(rSize);
    
    size_t const dSize = ZSTD_decompress(rBuff, rSize, depth_bytes, num_depth_bytes);
    
    //memcpy(renderFrame->depth_bytes, rBuff, rSize);
    for(int i = 0 ; i < num_points ; i++)
    {
        
        renderFrame->depth_bytes[4*i + 0] = rBuff[3*i + 0];
        renderFrame->depth_bytes[4*i + 1] = rBuff[3*i + 1];
        renderFrame->depth_bytes[4*i + 2] = rBuff[3*i + 2];
        
        
    }
    free(rBuff);
    
    
    
}


int decode_depth_bytes(int num_points, int num_depth_bytes, uint8_t* depth_bytes, RenderFrame* renderFrame)
{
    renderFrame->depth_bytes = (uint8_t*) malloc(num_points * 4 * sizeof(uint8_t));
    
    
    memset(renderFrame->depth_bytes, 0,num_points * 4 * sizeof(uint8_t));
    /*
     for(int i = 0 ; i < num_depth_bytes / 3 ; i++)
     {
     renderFrame->depth_bytes[4*i] = depth_bytes[3*i];
     renderFrame->depth_bytes[4*i+1] = depth_bytes[3*i+1];
     
     renderFrame->depth_bytes[4*i+2] = depth_bytes[3*i+2];
     
     }*/
    
    uint8_t* temp_buffer = (uint8_t*) malloc(num_depth_bytes*4 / 3 * sizeof(uint8_t) + 1);
    //TEST
    clock_t begin_breadth = clock();
    
    
    int idx = 0;
    for(int i = 0 ; i < num_depth_bytes ; i++)
    {
        if(i%3==1)
        {
            temp_buffer[idx]  = depth_bytes[i];
            temp_buffer[idx+1] = depth_bytes[i];
            idx += 2;
            
        }
        else
        {
            temp_buffer[idx] = depth_bytes[i];
            idx ++;
        }
        
    }
    
    clock_t end_breadth = clock();
    long msec = end_breadth - begin_breadth;
    
    for(int i = 0 ; i < num_points ; i++)
    {
        
        renderFrame->depth_bytes[4*i + 0] = temp_buffer[2*i + 0];
        renderFrame->depth_bytes[4*i + 1] = temp_buffer[2*i + 1];
        
        
    }
    
    
    
    free(temp_buffer);
    
    return idx;
}



vector_float4 get_child_center(vector_float4 parentCenter, float sidelen, uint8_t idx)
{
    float margin = sidelen / 4;
    vector_float4 childCenter = parentCenter;
    switch(idx)
    {
        case 0:
            childCenter[0] -= margin;
            childCenter[1] -= margin;
            childCenter[2] -= margin;
            break;
        case 1:
            childCenter[0] -= margin;
            childCenter[1] -= margin;
            childCenter[2] += margin;
            break;
        case 2:
            childCenter[0] -= margin;
            childCenter[1] += margin;
            childCenter[2] -= margin;
            break;
        case 3:
            childCenter[0] -= margin;
            childCenter[1] += margin;
            childCenter[2] += margin;
            break;
        case 4:
            childCenter[0] += margin;
            childCenter[1] -= margin;
            childCenter[2] -= margin;
            break;
        case 5:
            childCenter[0] += margin;
            childCenter[1] -= margin;
            childCenter[2] += margin;
            break;
        case 6:
            childCenter[0] += margin;
            childCenter[1] += margin;
            childCenter[2] -= margin;
            break;
        case 7:
            childCenter[0] += margin;
            childCenter[1] += margin;
            childCenter[2] += margin;
            break;
        default:
            
            break;
            
    }
    return childCenter;
}
