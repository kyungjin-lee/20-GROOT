#include "Frame.hpp"

#define MAX_FILE_SIZE 3000000

bool cmpColor(const Color &p1, const Color &p2){       

    if(sqrt(p1.r*p1.r + p1.g*p1.g + p1.b*p1.b) < sqrt(p2.r*p2.r + p2.g*p2.g + p2.b*p2.b)) 
    {
        return true;                                     
    }
    return false;
}

bitset<3> get_depth_bits(uint8_t val)                  
{ 
    bitset<3> bs(0);                                     

    switch(val)
    {
        case 1:                                           
            break;
        case 2:
            bs.set(0);
            break;
        case 4:
            bs.set(1);
            break;
        case 8:
            bs.set(0);                                    
            bs.set(1);                                        
            break;                                            
        case 16:                                          
            bs.set(2);
            break;        
        case 32:
            bs.set(0);
            bs.set(2);
            break;
        case 64:
            bs.set(1);
            bs.set(2);
            break;
        case 128:
            bs.set();
            break;
        default:
            printf("ERROR\n");
            break;


    }
    return bs;
}

uint8_t getDepthFourBits(uint8_t val)                  
{ 

    uint8_t mapped = 0; 
    switch(val)
    {
        case 1:                                           
            mapped = 1;
        case 2:
            mapped = 2;
            break;
        case 4:
            mapped = 3;
            break;
        case 8:
            mapped = 4;                                    
            break;                                            
        case 16:                                          
            mapped = 5;
            break;        
        case 32:
            mapped = 6;
            break;
        case 64:
            mapped = 7;
            break;
        case 128:
            mapped = 8;
            break;
        default:
            printf("ERROR %d\n", val);
            break;


    }
    return mapped;
}


Frame::Frame(): octree_(1.0)
{
    pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
    cloud_ = temp_cloud;


    /*-------TEST - for zstd compression-------
      ress_.fBufferSize = MAX_FILE_SIZE;
      ress_.cBufferSize = ZSTD_compressBound(MAX_FILE_SIZE);
      ress_.fBuffer = (uint8_t*) malloc(ress_.fBufferSize);
      ress_.cBuffer = (uint8_t*) malloc(ress_.cBufferSize);
      ress_.cctx = ZSTD_createCCtx();
      ------------------------------------------*/

    initialize();
}

void Frame::initialize()
{
    header_ = FrameHeader();
    header_.frame_type = '0';
    payload_ = FramePayload();

}

pcl::PointCloud<PointType>::Ptr Frame::getPointCloud()
{
    return cloud_;
}

vector<vector<uint8_t>> Frame::getBreadthBytes()
{
    return breadth_bytes_;
}


void Frame::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr out_cloud, float scale, bool isFlip, float x, float y , float z)
{ 
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1(0,0) = scale;
    transform_1(1,1) = scale;
    transform_1(2,2) = scale;
    if(isFlip) transform_1(1,1) = (-1)*scale;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << x, y, z;
    Eigen::Matrix4f transform = transform_2 * transform_1;

    pcl::transformPointCloud (*cloud, *out_cloud, transform);
}


void Frame::generateOctree(float voxelSize)
{

    octree_.setResolution(voxelSize);
    octree_.setInputCloud(cloud_);
    octree_.defineBoundingBox();
    octree_.addPointsFromInputCloud();

    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    root_center_ = Eigen::Vector3f((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);
    root_sidelength_ = sqrt(octree_.getVoxelSquaredSideLen(0));

    octree_depth_ = octree_.getTreeDepth();
    max_breadth_depth_ = octree_depth_ - 3;

    printf("--------------Frame %d Info---------------\n", frame_index_);
    printf("Number of Points: %d\n",cloud_->width);
    printf("Maximum octree depth: %d\n", octree_depth_);
    printf("Root center: %f %f %f\n",root_center_.x(), root_center_.y(), root_center_.z());
    printf("Side Length: %f\n", root_sidelength_);
    printf("------------------------------------------\n");

}

void Frame::compressPDTree(int is_user_adaptive, bool isShort)
{
    // Encode only last two depths if point cloud is dense
    // default is three for sparse point clouds
    if(isShort)
    {
        max_breadth_depth_ += 1;
        compressBreadthBytesShort();
        reorderDepthColorShort();
    }
    else{
        compressBreadthBytes();
        reorderDepthColor();
    }
    compressDepthBytes(is_user_adaptive);
}

/*
   Use the octree data structure to generate the Octree Breadth Bytes.
   Also save the Octree Depth Bytes to be compressed in the next step.
   This is the short version meaning only two bytes for octree depth bytes.
*/
void Frame::compressBreadthBytesShort()
{

    /*
      Serialize the occupancy bytes untill maximum breadth depth in breadth first order.
      Easier to use breadth first iterator but too slow.
    */
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator dfIt = octree_.depth_begin();
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator dfIt_end = octree_.depth_end();               


    uint8_t current_first = 0;                         
    uint8_t current_second = 0;                        


    int current_first_count = 0 ;
    int current_second_count = 0;

    vector<uint8_t> current_first_config;              
    vector<uint8_t> current_second_config;
    vector<int> leaf_node_indices;                     

    // create 2D vector
    for(int i = 0 ; i < octree_depth_ ; i++)           
    {                                                  
        vector<uint8_t> temp;                            
        breadth_bytes_.push_back(temp);                  
    }

    bool isFirst = true;
    Eigen::Vector3f calc_point = root_center_;
    float sidelength = root_sidelength_;
    while(dfIt!=dfIt_end)
    {
        int currentDepth = dfIt.getCurrentOctreeDepth();
        
        // This is for debugging purpose
        if(isFirst)
        {
            uint8_t temp_byte = dfIt.getNodeConfiguration();
            bitset<8> bs(temp_byte);
            for(int i = 0 ; i < 8 ; i++)
            {
                if(bs[i])
                {
                    calc_point = getChildCenter(calc_point,sidelength, i);
                    break;

                }
            }
            sidelength = sidelength / 2;
        }

        // This is for octree breadth bytes
        if(currentDepth < octree_depth_)
        {
            breadth_bytes_.at(currentDepth).push_back(dfIt.getNodeConfiguration());
        }
        // This is for first byte of octree depth bytes
        if(currentDepth == max_breadth_depth_)         
        {                                              
            current_first = dfIt.getNodeConfiguration();
            bitset<8> bs(current_first);               
            current_first_config.clear();              
            current_first_count = 0;                   
            for(int i = 0 ; i < 8 ; i++)               
            {
                bitset<8> currentBs(0);                
                if(bs[i])
                {
                    currentBs.set(i);
                    current_first_config.push_back(static_cast<uint8_t>(currentBs.to_ulong()));
                }
            }
        }
        // This is for the second byte of the octree depth bytes
        else if(currentDepth == max_breadth_depth_+1)
        {
            current_second = dfIt.getNodeConfiguration();
            current_first = current_first_config[current_first_count];
            bitset<8> bs(current_second);              
            for(int i = 0 ; i < 8 ; i++)               
            {
                bitset<8> currentBs(0);
                if(bs[i])
                {
                    currentBs.set(i);
                    uint8_t current_last = static_cast<uint8_t>(currentBs.to_ulong());
                    depth_list_.push_back(current_first);
                    depth_list_.push_back(current_last);
                }
            }
            current_first_count ++;
        }
        /*
         This is to get the leaf node order of the octree data structure
         childIndices indicates the index in the original PLY file
         Leaf node order is needed to find the color information 
        */
        else if(currentDepth == octree_depth_)
        {
            vector<int> childIndices;
            getNodesInOctreeContainer(dfIt.getCurrentOctreeNode(), childIndices);
            point_indices_.push_back(childIndices[0]);
            // For debugging
            if(isFirst)
            {
                printf("from bounding box: %f %f %f\n", calc_point.x(), calc_point.y(), calc_point.z());
                printf("original : %f %f %f\n", cloud_->points[childIndices[0]].x, cloud_->points[childIndices[0]].y, cloud_->points[childIndices[0]].z);
                calc_point = calc_point - Eigen::Vector3f(cloud_->points[childIndices[0]].x, cloud_->points[childIndices[0]].y, cloud_->points[childIndices[0]].z);
                isFirst = false;
            }
        }
        dfIt++;
    }

    generateLeafNodeIndices();

}

/*
   Use the octree data structure to generate the Octree Breadth Bytes.
   Also save the Octree Depth Bytes to be compressed in the next step.
   This is the long version meaning only three bytes for octree depth bytes.
*/
void Frame::compressBreadthBytes()
{

    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator dfIt = octree_.depth_begin();
    pcl::octree::OctreePointCloud<PointType>::DepthFirstIterator dfIt_end = octree_.depth_end();               


    uint8_t current_first = 0;                         
    uint8_t current_second = 0;                        
    uint8_t current_third = 0;                         


    int current_first_count = 0 ;
    int current_second_count = 0;
    vector<uint8_t> current_first_config;              
    vector<uint8_t> current_second_config;
    vector<int> leaf_node_indices;                     

    for(int i = 0 ; i < octree_depth_ ; i++)           
    {                                                  
        vector<uint8_t> temp;                            
        breadth_bytes_.push_back(temp);                  
    }

    bool isFirst = true;
    Eigen::Vector3f calc_point = root_center_;
    float sidelength = root_sidelength_;
    while(dfIt!=dfIt_end)
    {
        int currentDepth = dfIt.getCurrentOctreeDepth();

        if(isFirst)
        {
            uint8_t temp_byte = dfIt.getNodeConfiguration();
            bitset<8> bs(temp_byte);
            for(int i = 0 ; i < 8 ; i++)
            {
                if(bs[i])
                {
                    calc_point = getChildCenter(calc_point,sidelength, i);
                    break;

                }
            }
            sidelength = sidelength / 2;
        }
        if(currentDepth < octree_depth_)
        {
            breadth_bytes_.at(currentDepth).push_back(dfIt.getNodeConfiguration());
        }
        if(currentDepth == max_breadth_depth_)         
        {                                              
            current_first = dfIt.getNodeConfiguration();
            bitset<8> bs(current_first);               
            current_first_config.clear();              
            current_first_count = 0;                   
            for(int i = 0 ; i < 8 ; i++)               
            {
                bitset<8> currentBs(0);                
                if(bs[i])
                {
                    currentBs.set(i);
                    current_first_config.push_back(static_cast<uint8_t>(currentBs.to_ulong()));
                }
            }
        }
        else if(currentDepth == max_breadth_depth_+1)
        {
            current_second = dfIt.getNodeConfiguration();
            current_first = current_first_config[current_first_count];
            bitset<8> bs(current_second);              
            current_second_config.clear();             
            current_second_count = 0;                  
            for(int i = 0 ; i < 8 ; i++)               
            {
                bitset<8> currentBs(0);
                if(bs[i])
                {
                    currentBs.set(i);
                    current_second_config.push_back(static_cast<uint8_t>(currentBs.to_ulong()));
                }
            }
            current_first_count ++;
        }

        else if(currentDepth == max_breadth_depth_+2)
        {
            current_second = current_second_config[current_second_count];
            current_third = dfIt.getNodeConfiguration();
            bitset<8> bs(current_third);
            for(int i = 0 ; i< 8 ; i++)
            {
                bitset<8> currentBs(0);
                if(bs[i])
                {
                    currentBs.set(i);                    
                    uint8_t current_last = static_cast<uint8_t>(currentBs.to_ulong());
                    depth_list_.push_back(current_first);
                    depth_list_.push_back(current_second);
                    depth_list_.push_back(current_last);
                }

            }
            current_second_count ++;
        }

        else if(currentDepth == octree_depth_)
        {
            vector<int> childIndices;
            getNodesInOctreeContainer(dfIt.getCurrentOctreeNode(), childIndices);
            point_indices_.push_back(childIndices[0]);
            if(isFirst)
            {
                printf("from bounding box: %f %f %f\n", calc_point.x(), calc_point.y(), calc_point.z());
                printf("original : %f %f %f\n", cloud_->points[childIndices[0]].x, cloud_->points[childIndices[0]].y, cloud_->points[childIndices[0]].z);
                calc_point = calc_point - Eigen::Vector3f(cloud_->points[childIndices[0]].x, cloud_->points[childIndices[0]].y, cloud_->points[childIndices[0]].z);
                isFirst = false;
            }
        }
        dfIt++;
    }

    generateLeafNodeIndices();

}

void Frame::generatePayload()
{

    // Add breadth bytes to final payload
    for(int i =  0 ; i < max_breadth_depth_ ; i++)
    {
        payload_.breadth_bytes.insert(payload_.breadth_bytes.end(), breadth_bytes_.at(i).begin(), breadth_bytes_.at(i).end());
    }

    // Add compressed depth bytes to final payload
    payload_.depth_bytes.resize(compressed_depth_bytes_.size());
    memcpy(&payload_.depth_bytes[0], &compressed_depth_bytes_[0], compressed_depth_bytes_.size());

    // Save number of leaf nodes per parent nodes at maximum breadth depth for decoding
    int cntBreadthLeafNum = 0;
    for(int i = 1 ; i < leaf_indices_.at(max_breadth_depth_).size() ; i++)
    {
        int start = leaf_indices_.at(max_breadth_depth_).at(i-1);
        int end = leaf_indices_.at(max_breadth_depth_).at(i);

        payload_.breadth_leaf_indices.push_back(uint8_t(end-start)); 
        cntBreadthLeafNum += end-start;
    }

    // Add color bytes for decoding not compressed yet
    for(int i= 0 ; i < color_list_.size() ; i++)
    {
        Color currentColor = color_list_.at(i);
        payload_.color_bytes.push_back(currentColor.r);
        payload_.color_bytes.push_back(currentColor.g);
        payload_.color_bytes.push_back(currentColor.b);
    }

}

void Frame::generatePayload(vector<uint8_t> compressed_colors)
{
    for(int i =  0 ; i < max_breadth_depth_ ; i++)
    {
        payload_.breadth_bytes.insert(payload_.breadth_bytes.end(), breadth_bytes_.at(i).begin(), breadth_bytes_.at(i).end());
    }

    payload_.depth_bytes.resize(compressed_depth_bytes_.size());
    memcpy(&payload_.depth_bytes[0], &compressed_depth_bytes_[0], compressed_depth_bytes_.size());

    int cntBreadthLeafNum = 0;
    for(int i = 1 ; i < leaf_indices_.at(max_breadth_depth_).size() ; i++)
    {
        int start = leaf_indices_.at(max_breadth_depth_).at(i-1);
        int end = leaf_indices_.at(max_breadth_depth_).at(i);

        payload_.breadth_leaf_indices.push_back(uint8_t(end-start)); 
        cntBreadthLeafNum += end-start;
    }

    payload_.color_bytes.resize(compressed_colors.size());
    memcpy(&payload_.color_bytes[0], &compressed_colors[0], compressed_colors.size());
}

void Frame::generateHeader(char type)
{
    printf("[FRAME] generate header \n");
    memset(&header_, 0,sizeof(FrameHeader));
    printf("============Write header=============\n");  
    header_.frame_type =  type;                          
    printf("Frame type: %c\n" , header_.frame_type);
    header_.num_breadth_bytes = payload_.breadth_bytes.size();
    printf("Number of breadth bytes till maxbreadthdepth: %d\n", header_.num_breadth_bytes);
    header_.num_breadth_nodes = payload_.breadth_leaf_indices.size();                                           
    header_.num_depth_bytes = payload_.depth_bytes.size();
    printf("Number of depth bytes : %d\n", header_.num_depth_bytes);
    header_.num_color_bytes = payload_.color_bytes.size();
    header_.num_points = point_indices_.size();
    header_.num_icp_nodes = 0;
    header_.num_icp_points = 0;

    printf("Number of points: %d\n", header_.num_points);
    header_.root_center = root_center_;
    printf("Root center: %f %f %f\n", header_.root_center.x(), header_.root_center.y(), header_.root_center.z());
    header_.root_sidelength = root_sidelength_;
    printf("Root sidelength : %f\n", header_.root_sidelength); 
}

/*
 Reorder depth bytes regarding the order of reordered color (following morton code)
 This is the short version
*/
void Frame::reorderDepthColorShort()
{
    int numBreadthNodes = breadth_bytes_.at(max_breadth_depth_).size() ;
    //for octree depth nodes reorder by similar colors
    vector<uint8_t> reordered_depth_nodes;
    vector<Color> reordered_color_nodes;


    // FOR TEST
    pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<PointType>);

    for(int i = 0 ; i < numBreadthNodes ; i++)
    {
        int startIndex = leaf_indices_.at(max_breadth_depth_).at(i);
        int endIndex = leaf_indices_.at(max_breadth_depth_).at(i+1);
        for(int j = startIndex ; j < endIndex ; j++)
        {
            Color col;
            col.index = j;
            int point_index = point_indices_[j];
            col.r = cloud_->points[point_index].r;
            col.g = cloud_->points[point_index].g;
            col.b = cloud_->points[point_index].b;
            reordered_color_nodes.push_back(col);

            //For debugging
//            Eigen::Vector4f current_point(cloud_->points[point_index].x, cloud_->points[point_index].y, cloud_->points[point_index].z, 1.0);
//            debug_point_list_.push_back(current_point);
//            debug_color_list_.push_back(col.r);
//            debug_color_list_.push_back(col.g);
//            debug_color_list_.push_back(col.b);
//            PointType p;
//            p.x = cloud_->points[point_index].x;
//            p.y = cloud_->points[point_index].y;
//            p.z = cloud_->points[point_index].z;
//            p.r = col.r;
//            p.g = col.g;
//            p.b = col.b;
//            out_cloud->push_back(p);
//
        }

        // for UNSORTED colors comment this
        sort(reordered_color_nodes.begin() + startIndex, reordered_color_nodes.begin() + endIndex, cmpColor);

        for(int j = startIndex ; j < endIndex ; j++)
        {
            int currentIdx = reordered_color_nodes[j].index;
            reordered_depth_nodes.push_back(depth_list_[2*currentIdx+0]);
            reordered_depth_nodes.push_back(depth_list_[2*currentIdx+1]);
        }
    }

    depth_list_ = reordered_depth_nodes;
    color_list_ = reordered_color_nodes;                 


} 


/*
 Reorder depth bytes regarding the order of reordered color (following morton code)
 This is the long version
*/
void Frame::reorderDepthColor()
{
    int numBreadthNodes = breadth_bytes_.at(max_breadth_depth_).size() ;
    //for octree depth nodes reorder by similar colors
    vector<uint8_t> reordered_depth_nodes;
    vector<Color> reordered_color_nodes;


    // FOR TEST
    pcl::PointCloud<PointType>::Ptr out_cloud(new pcl::PointCloud<PointType>);

    for(int i = 0 ; i < numBreadthNodes ; i++)
    {
        int startIndex = leaf_indices_.at(max_breadth_depth_).at(i);
        int endIndex = leaf_indices_.at(max_breadth_depth_).at(i+1);
        for(int j = startIndex ; j < endIndex ; j++)
        {
            Color col;
            col.index = j;
            int point_index = point_indices_[j];
            col.r = cloud_->points[point_index].r;
            col.g = cloud_->points[point_index].g;
            col.b = cloud_->points[point_index].b;
            reordered_color_nodes.push_back(col);

            //For debugging
            Eigen::Vector4f current_point(cloud_->points[point_index].x, cloud_->points[point_index].y, cloud_->points[point_index].z, 1.0);
            debug_point_list_.push_back(current_point);
            debug_color_list_.push_back(col.r);
            debug_color_list_.push_back(col.g);
            debug_color_list_.push_back(col.b);
            PointType p;
            p.x = cloud_->points[point_index].x;
            p.y = cloud_->points[point_index].y;
            p.z = cloud_->points[point_index].z;
            p.r = col.r;
            p.g = col.g;
            p.b = col.b;
            out_cloud->push_back(p);

        }

        // for UNSORTED comment this
        sort(reordered_color_nodes.begin() + startIndex, reordered_color_nodes.begin() + endIndex, cmpColor);

        for(int j = startIndex ; j < endIndex ; j++)
        {
            int currentIdx = reordered_color_nodes[j].index;
            reordered_depth_nodes.push_back(depth_list_[3*currentIdx+0]);
            reordered_depth_nodes.push_back(depth_list_[3*currentIdx+1]);
            reordered_depth_nodes.push_back(depth_list_[3*currentIdx+2]);
        }
    }

    // for TEST
    //pcl::PLYWriter writer;
    //pcl::PCLPointCloud2 blob;
    //pcl::toPCLPointCloud2(*out_cloud, blob);
    //writer.writeASCII("longdress_1051_octree.ply", blob);
    //  FILE* testFile = fopen("temp_comp.bin", "wb");
    //  fwrite(&reordered_depth_nodes[0], sizeof(uint8_t), reordered_depth_nodes.size(), testFile);


    depth_list_ = reordered_depth_nodes;
    color_list_ = reordered_color_nodes;                 


} 

// get original color(rgb) list
vector<uint8_t> Frame::getColorBytes()
{
    vector<uint8_t> serial_color(3 * color_list_.size());
    for(int i = 0 ; i < color_list_.size() ; i++)
    {
        serial_color[3*i] = color_list_[i].r;
        serial_color[3*i+1] = color_list_[i].g;
        serial_color[3*i+2] = color_list_[i].b;
    }
    return serial_color;
}

// Compress each depth bytes to 4 bits
void Frame::compressDepthBytes(int is_user_adaptive)
{
    if(is_user_adaptive == 1)
    {
        compressed_depth_bytes_.resize(depth_list_.size(), 0);
        int size = depth_list_.size();
        for(int i = 0 ; i < depth_list_.size() ; i++)
        {
            uint8_t modified = getDepthFourBits(depth_list_[i]);
            compressed_depth_bytes_[i] = modified;
        }
    }

    else
    {
        if(depth_list_.size() % 2 != 0)
        {
            depth_list_.push_back(0);
        }
        compressed_depth_bytes_.resize(depth_list_.size()/2, 0);
        for(int i = 0 ; i < depth_list_.size()/2 ; i++)
        {
            uint8_t first_byte = getDepthFourBits(depth_list_[2*i]);
            uint8_t second_byte = getDepthFourBits(depth_list_[2*i+1]);
            second_byte = second_byte << 4;
            uint8_t final_byte = first_byte | second_byte;
            compressed_depth_bytes_[i] = final_byte;
        }
    }
}

/*
  Testing zstd compression library
*/
void Frame::compressDepthBytesZstd(vector<uint8_t> depth_bytes)
{
    for(int i= 0 ; i < 10 ; i++)
    {
        printf("[FRAME] sample depth bytes : %d\n", depth_bytes[i]);
    }
    for(int i = depth_bytes.size() - 10 ; i < depth_bytes.size() ; i++)
    {
        printf("[FRAME] sample depth bytes : %d\n", depth_bytes[i]);
    }
    printf("[FRAME] compress zstd\n");
    memcpy(ress_.fBuffer, &depth_bytes[0], depth_bytes.size() * sizeof(uint8_t));
    int fSize = depth_bytes.size();
    size_t const cSize = ZSTD_compressCCtx(ress_.cctx, ress_.cBuffer, ress_.cBufferSize, ress_.fBuffer, fSize, 1);

    printf("[Frame] compressed depth bytes : %d\n", cSize);
    depth_bytes_zstd_.resize(cSize);
    memcpy(&depth_bytes_zstd_[0], ress_.cBuffer, cSize);
}

void Frame::writeFrame(std::string filename, unsigned int* avgSize)
{
    generateHeader('I');

    int totalSize = sizeof(FrameHeader) + payload_.breadth_bytes.size() +  payload_.depth_bytes.size() + payload_.breadth_leaf_indices.size() + payload_.color_bytes.size();

    *avgSize += totalSize;
    FILE* pFile;
    pFile = fopen(filename.c_str(), "wb");

    fwrite(&totalSize, sizeof(int), 1, pFile);
    fwrite(&header_, sizeof(FrameHeader), 1, pFile);
    printf("Write breadth %d depth %d color %d\n", payload_.breadth_bytes.size(), payload_.depth_bytes.size(), payload_.color_bytes.size());


    fwrite(&payload_.breadth_bytes[0], sizeof(uint8_t), payload_.breadth_bytes.size(), pFile);
    fwrite(&payload_.depth_bytes[0], sizeof(uint8_t), payload_.depth_bytes.size(), pFile);



    fwrite(&payload_.breadth_leaf_indices[0], sizeof(uint8_t), payload_.breadth_leaf_indices.size(), pFile);
    fwrite(&payload_.color_bytes[0], sizeof(uint8_t), payload_.color_bytes.size(), pFile);
    //  fwrite(&compressed_color[0], sizeof(uint8_t), compressed_color.size(), pFile);
    fclose(pFile);
/*    printf("[TEST WRITE HEADER]\n");
    printf("Header num points : %d\n", header_.num_points);
    printf("Header num breadth byte: %d\n", header_.num_breadth_bytes);
    printf("Header num breadth nodes: %d\n", header_.num_breadth_nodes);
    printf("Header num depth nodes: %d\n", header_.num_depth_bytes);
    printf("Header num color bytes : %d\n", header_.num_color_bytes);
    printf("Header root center : %f %f %f\n", header_.root_center[0], header_.root_center[1], header_.root_center[2]);
    printf("Header sidelength: %f\n", header_.root_sidelength);
*/
    cloud_.reset();

}      

/*
   generate a 2d vector that saves leaf node indices
   which is the starting leaf node index for each octree node from depth 0 to maximum octree depth
*/
void Frame::generateLeafNodeIndices()
{ 
    for(int i = 0 ; i < octree_depth_ ; i++)
    { 
        vector<int> temp;
        leaf_indices_.push_back(temp);
    }

    int cntChild = 0;
    for(int i= 0 ; i < breadth_bytes_.at(octree_depth_-1).size() ; i++)
    { 
        leaf_indices_.at(octree_depth_-1).push_back(cntChild);
        bitset<8> bs(breadth_bytes_.at(octree_depth_-1).at(i));
        cntChild += bs.count();

    }
    leaf_indices_.at(octree_depth_-1).push_back(cntChild);
    int leafNodeNum = cntChild; 
    for(int i = octree_depth_-2 ; i >=0 ; i--)
    { 
        int cntChild = 0; 
        for(int j = 0 ; j < breadth_bytes_.at(i).size() ; j++)
        { 
            bitset<8> bs(breadth_bytes_.at(i).at(j));
            int childIdx = leaf_indices_.at(i+1).at(cntChild);
            leaf_indices_.at(i).push_back(childIdx);
            cntChild += bs.count();
        }
        leaf_indices_.at(i).push_back(leafNodeNum);
    }



}

void Frame::generateChildNodeIndices()
{
    for(int i = 0 ; i < octree_depth_ ; i++)
    {
        vector<int> temp;
        child_indices_.push_back(temp);
    }

    for(int i = 0 ; i < octree_depth_ ; i++)
    {
        int cntChild = 0;
        for(int j = 0 ; j < breadth_bytes_.at(i).size() ; j++)
        {
            bitset<8> bs(breadth_bytes_.at(i).at(j));
            child_indices_.at(i).push_back(cntChild);
            cntChild += bs.count();
        }
        child_indices_.at(i).push_back(cntChild);
    }


}


Eigen::Vector3f Frame::getChildCenter(Eigen::Vector3f parentCenter,float sidelen, int idx)
{
    float margin = sidelen / 4;
    Eigen::Vector3f childCenter;
    switch(idx)
    {
        case 0:
            childCenter(0) = parentCenter(0) - margin;
            childCenter(1) = parentCenter(1) - margin;
            childCenter(2) = parentCenter(2) - margin;
            break;
        case 1:
            childCenter(0) = parentCenter(0) - margin;
            childCenter(1) = parentCenter(1) - margin;
            childCenter(2) = parentCenter(2) + margin;
            break;
        case 2:
            childCenter(0) = parentCenter(0) - margin;
            childCenter(1) = parentCenter(1) + margin;
            childCenter(2) = parentCenter(2) - margin;
            break;
        case 3:
            childCenter(0) = parentCenter(0) - margin;
            childCenter(1) = parentCenter(1) + margin;
            childCenter(2) = parentCenter(2) + margin;
            break;
        case 4:
            childCenter(0) = parentCenter(0) + margin;
            childCenter(1) = parentCenter(1) - margin;
            childCenter(2) = parentCenter(2) - margin;
            break;
        case 5:
            childCenter(0) = parentCenter(0) + margin;
            childCenter(1) = parentCenter(1) - margin;
            childCenter(2) = parentCenter(2) + margin;
            break;
        case 6:
            childCenter(0) = parentCenter(0) + margin;
            childCenter(1) = parentCenter(1) + margin;
            childCenter(2) = parentCenter(2) - margin;
            break;
        case 7:
            childCenter(0) = parentCenter(0) + margin;
            childCenter(1) = parentCenter(1) + margin;
            childCenter(2) = parentCenter(2) + margin;
            break;

        default:
            childCenter = parentCenter;
            break;

    }
    return childCenter;
}

// recursive function to get all the leaf nodes included in an octree node
// for leaf nodes -> return list of leaf nodes
// for non-leaf nodes -> recurse through all child nodes to get list of leaf nodes
void Frame::getNodesInOctreeContainer(pcl::octree::OctreeNode* currentNode, std::vector<int> &indices)
{
    if(currentNode->getNodeType() == pcl::octree::LEAF_NODE)
    {
        pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>* leaf_node = static_cast<pcl::octree::OctreeLeafNode<pcl::octree::OctreeContainerPointIndices>*>(currentNode);
        pcl::octree::OctreeContainerPointIndices container = leaf_node->getContainer();
        std::vector<int> pointIndices;
        container.getPointIndices(pointIndices);
        for(int j = 0 ; j < pointIndices.size() ; j++)
        {
            indices.push_back(pointIndices.at(j));
            //std::cout << "point indices: " << pointIndices.at(j) << std::endl;
        }
    }
    else
    {
        pcl::octree::OctreeBranchNode<pcl::octree::OctreeContainerEmpty>* branch_node = static_cast<pcl::octree::OctreeBranchNode<pcl::octree::OctreeContainerEmpty>*>(currentNode);
        for(int i = 0 ; i < 8 ; i++)
        {
            pcl::octree::OctreeNode* childNode = branch_node->getChildPtr(i);
            if(childNode != NULL){
                //      std::cout<< "child ptr type: " << childNode->getNodeType()<<std::endl;
                getNodesInOctreeContainer(childNode, indices);
            }
        }
    }

}


void Frame::readPointCloud(int frame_index, std::string filename, float scale, bool isFlip, float x, float y, float z)
{
    const string PLY = ".ply";
    const string PCD =".pcd";

    frame_index_ = frame_index;
    pcl::PointCloud<PointType>::Ptr orig_cloud( new pcl::PointCloud<PointType>);
    if(filename.find(PLY) != string::npos)
    {
        pcl::PLYReader reader;
        reader.read(filename, *orig_cloud);
    }

    else if(filename.find(PCD) != string::npos)
    {
        pcl::io::loadPCDFile(filename, *orig_cloud);
    }

    orig_num_points_ = orig_cloud->width;
    printf("[FRAME] original num points : %d\n", orig_num_points_);

    transformPointCloud(orig_cloud, cloud_, scale, isFlip, x, y, z);
    orig_cloud.reset();
}


// For debugging purposes
vector<Eigen::Vector4f> Frame::get_debug_point_list()
{
    return debug_point_list_;
}

vector<uint8_t> Frame::get_debug_colors()
{  
    return debug_color_list_; 
}



