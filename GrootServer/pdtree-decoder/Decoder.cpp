#include "Decoder.hpp"


#define MAX_THREAD 4

static void* get_depth_bytes(void* t_args)
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


void Decoder::readManifest(std::string filename, Manifest manifest)
{
//    FILE* pFile;
//    pFile = fopen(filename.c_str(), "rb");
//    fread(&manifest, sizeof(Manifest), 1, pFile);
    manifest.key_frame_interval = 1;
    manifest.max_breadth_depth_sidelength = 0.008;
}
void Decoder::decodePDTreeFile(std::string filename)
{
    FILE* pFile;
    pFile = fopen(filename.c_str(), "rb");

    int totalSize = 0;
    fread(&totalSize, sizeof(int), 1, pFile);

    RxFrameHeader header;
    fread(&header, sizeof(RxFrameHeader), 1 , pFile);

    printf("[DECODE] =========RxFrameHeader=========\n");
    printf("[DECODE] frame type %d\n", header.frame_type);
    printf("[DECODE] num breadth bytes %d\n", header.num_breadth_bytes);
    printf("[DECODE] num breadth nodes: %d\n", header.num_breadth_nodes);

    printf("[DECODE] num depth bytes: %d\n", header.num_depth_bytes);
    printf("[DECODE] num color bytes :%d\n", header.num_color_bytes);
    printf("[DECODE] num points %d\n", header.num_points);
    printf("[DECODE] sidelength %f\n", header.root_sidelength);
    printf("================================\n");

    RxFrame rxFrame;
    rxFrame.header = header;
    rxFrame.breadth_bytes = (uint8_t*)malloc(header.num_breadth_bytes * sizeof(uint8_t));
    rxFrame.depth_bytes = (uint8_t*)malloc(header.num_depth_bytes * sizeof(uint8_t));
    rxFrame.breadth_leaf_num = (uint8_t*)malloc(header.num_breadth_nodes * sizeof(uint8_t));
    rxFrame.color_bytes = (uint8_t*)malloc(header.num_color_bytes * sizeof(uint8_t));


    fread(rxFrame.breadth_bytes, sizeof(uint8_t), header.num_breadth_bytes, pFile);
    fread(rxFrame.depth_bytes, sizeof(uint8_t), header.num_depth_bytes, pFile);
    fread(rxFrame.breadth_leaf_num , sizeof(uint8_t), header.num_breadth_nodes, pFile);
    fread(rxFrame.color_bytes, sizeof(uint8_t), header.num_color_bytes, pFile);

    render_frame_.center_list = (Eigen::Vector4f*) malloc(header.num_points * sizeof(Eigen::Vector4f));
    render_frame_.depth_bytes = (uint8_t*) malloc(header.num_points * 3 * sizeof(uint8_t));
    render_frame_.color_bytes = (uint8_t*) malloc(header.num_points * 4 * sizeof(uint8_t));
    clock_t begin_time = clock();
    decodeBreadthBytes(header, rxFrame.breadth_bytes, rxFrame.breadth_leaf_num);
    clock_t end_time = clock();
    long msec = end_time - begin_time;
    printf("[Decoder] decode time1 : %.3f\n", (double)((double)msec / CLOCKS_PER_SEC));

  
    decodeDepthBytes(header.num_points, header.num_depth_bytes, rxFrame.depth_bytes);
    clock_t end_time2 = clock();
    msec = end_time2 - end_time;
    printf("[Decoder] decode time2 : %.3f\n", (double)((double)msec / CLOCKS_PER_SEC));

    decodeColorBytes(header.num_points, header.num_color_bytes, rxFrame.color_bytes);
    clock_t end_time3 = clock();
    msec = end_time3 - end_time2;
    printf("[Decoder] decode time1 : %.3f\n", (double)((double)msec / CLOCKS_PER_SEC));

    //     clock_t end_time = clock();
    //    long msec = end_time - begin_time;
    //    printf("[Decoder] decode time1 : %.3f\n", (double)((double)msec / CLOCKS_PER_SEC));

    render_frame_.frame_type = header.frame_type;
    render_frame_.num_points = header.num_points;

}


void Decoder::decodeBreadthBytes(RxFrameHeader header, uint8_t* breadth_bytes, uint8_t* breadth_leaf_num)
{
    Eigen::Vector4f* centers;
    centers = (Eigen::Vector4f*) malloc(header.num_breadth_nodes*sizeof(Eigen::Vector4f));
    Eigen::Vector4f* next_centers;
    next_centers = (Eigen::Vector4f*) malloc(header.num_breadth_nodes*sizeof(Eigen::Vector4f));

    centers[0].x() = header.root_center[0];
    centers[0].y() = header.root_center[1];
    centers[0].z() = header.root_center[2];
    centers[0].w() = 1.0;

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
                    Eigen::Vector4f child_center = getChildCenter(centers[i], currentSideLength, k);
                    next_centers[cntNextCenters] = child_center;
                    cntNextCenters++;
                }
            }

        }
        cntBytes += cntCenters;
        cntCenters = cntNextCenters;
        memcpy(centers, next_centers, cntNextCenters * sizeof(Eigen::Vector4f));
        currentSideLength = currentSideLength / 2;
        cntDepth ++;
    }



    int cntNodes = 0;

    for(int i = 0 ; i < header.num_breadth_nodes ; i++)
    {
        uint8_t currentNumNode = *(breadth_leaf_num + i);
        Eigen::Vector4f currentCenter = *(centers + i);
        for(int j = 0 ; j < currentNumNode ; j++)
        {
            render_frame_.center_list[cntNodes + j] = currentCenter;

        }
        cntNodes += currentNumNode;
    }


    printf("COUNT NODES: %d numPoints %d\n", cntNodes, header.num_points);


}

void Decoder::decodeDepthBytes(int num_points, int num_depth_bytes, uint8_t* depth_bytes)

{


    memset(render_frame_.depth_bytes, 1, num_points * 3 * sizeof(uint8_t));
    if(num_depth_bytes * 2 / 3 == num_points )
    {
       printf("[DECODER] depth bytes number correct\n");
    }


    for(int i = 0 ; i < num_depth_bytes ; i++)
    {
        uint8_t current_byte = depth_bytes[i];
        render_frame_.depth_bytes[2*i+0] = (current_byte & 15) - 1;
        render_frame_.depth_bytes[2*i+1] = (current_byte >> 4) - 1;
    }


     



}



void Decoder::decodeColorBytes(int num_points, int num_color_bytes, uint8_t* color_bytes)
{
    memset(render_frame_.color_bytes, 1, num_points * 4 * sizeof(uint8_t));
    int num_nodes = num_color_bytes / 3;
    for(int i = 0 ; i < num_nodes ; i++)
    {
        render_frame_.color_bytes[4*i + 0] = color_bytes[3*i + 0];
        render_frame_.color_bytes[4*i + 1] = color_bytes[3*i + 1];
        render_frame_.color_bytes[4*i + 2] = color_bytes[3*i + 2];

    }


    /*
       tjhandle handle = tjInitDecompress();

       uint8_t* decompressed_color;
       int width = 0;
       int height = 0;
       int jpegSubsamp = 0;
       int jpegColorSpace = 0;



       tjDecompressHeader3(handle, color_bytes, num_color_bytes, &width, &height, &jpegSubsamp,
       &jpegColorSpace);

       decompressed_color = malloc(width * height * 4);
       tjDecompress2(handle, color_bytes, num_color_bytes, decompressed_color,width,width * 4, height, jpegColorSpace,  0);

       renderFrame->color_bytes = (uint8_t*) malloc(num_points * 4 * sizeof(uint8_t));
       memcpy(renderFrame->color_bytes, decompressed_color, num_points * 4 * sizeof(uint8_t));
     */


}


void Decoder::test_sum(int cnt, vector<uint8_t>* current_input)
{
    int sum = 0;
    for(int i= 0 ; i < current_input->size() ; i++)
    {
        sum = sum + 1;
    }
}

Eigen::Vector4f Decoder::getChildCenter(Eigen::Vector4f parentCenter, float sidelen, uint8_t idx)
{
    float margin = sidelen / 4;
    Eigen::Vector4f childCenter = parentCenter;
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

void Decoder::generatePointCloud(std::string filename, Manifest manifest)
{
    for(int i = 0 ; i < 10 ; i++)
    {
        printf("[BREADTH] %f %f %f\n", render_frame_.center_list[i].x(), render_frame_.center_list[i].y(), render_frame_.center_list[i].z());
    }

    for(int i = 0 ; i < 30 ; i++)
    {
        printf("[DEPTH] %d\n", render_frame_.depth_bytes[i]);
    }

    for(int i = 0 ; i < 10 ; i++)
    {
        printf("[COLOR] %d\n", render_frame_.color_bytes[i]);
    }
    delete(render_frame_.center_list);
    delete(render_frame_.depth_bytes);
    delete(render_frame_.color_bytes);
}

void Decoder::write_decode(std::string outfile)
{}
