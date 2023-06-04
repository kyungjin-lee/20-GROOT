#include "JpegEncoder.hpp"
#include <fstream>
#include <cstdio>
JpegEncoder::JpegEncoder()
{
    handle_ = tjInitCompress();

    jpegQual_ =96;
    nbands_ = 3;
    flags_ = 0;
    pixelFormat_ = TJPF_RGB;
    jpegSubsamp_ = TJSAMP_420;
}

int JpegEncoder::encode(vector<uint8_t> rgb_list, vector<uint8_t> &jpeg, int width, int height)
{

    printf("[JPEG] start encode\n");
    uint8_t* srcBuf = new uint8_t[width * height * nbands_];
    memset(srcBuf, 0, width * height * nbands_);
    memcpy(srcBuf, &rgb_list[0], rgb_list.size());

    uint8_t* jpegBuf =NULL;
    unsigned long jpegSize;
    int tj_stat = tjCompress2(handle_, srcBuf, width, width * nbands_, height, pixelFormat_, 
            &(jpegBuf), &jpegSize, jpegSubsamp_, jpegQual_, flags_);

    jpeg.resize(jpegSize, 0);
    memcpy(&jpeg[0], jpegBuf, jpegSize);

    FILE* qFile= fopen("test_image.jpg", "wb");
    fwrite(jpegBuf, sizeof(uint8_t), jpegSize, qFile);
    if(tj_stat != 0)
    {   
        const char *err = (const char *) tjGetErrorStr();
        cerr << "TurboJPEG Error: " << err << " UNABLE TO COMPRESS JPEG IMAGE\n";
        tjDestroy(handle_);
        handle_ = NULL;
        return -1;
    }

    fclose(qFile);

    delete jpegBuf;
    delete []srcBuf;
    return 1;

}



void JpegEncoder::testDecoder(vector<uint8_t> compressed_bytes, vector<uint8_t>& decoded_bytes)
{
    tjhandle handle = tjInitDecompress();
    if(handle < 0) printf("Can't initialize decoder\n");
    int width = 0;
    int height = 0;
    int jpegSubsamp = 0;
    int jpegColorSpace = 0;


    int numCompressedBytes = compressed_bytes.size();


    tjDecompressHeader3(handle, &compressed_bytes[0], numCompressedBytes, &width, &height, &jpegSubsamp,
            &jpegColorSpace);

    unsigned char *imgBuf = (unsigned char *) malloc(width * height * 3);
    printf("Jpeg decoder header (filesize , width, height, jpegSubsamp, jpegColorSpace) %lu %d %d %d %d\n", compressed_bytes.size(), width, height, jpegSubsamp, jpegColorSpace); 
    if( tjDecompress2(handle, &compressed_bytes[0], numCompressedBytes, imgBuf ,width,width * 3, height, jpegColorSpace,  0) < 0)
    {
        printf("decompress failed\n");
    }

    for(int i = 0 ; i  < 30 ; i++)
    {
        printf("%d\n", *(imgBuf+i));
    }
    decoded_bytes.resize(width  *height * 3);
    memcpy(&decoded_bytes[0], imgBuf, width * height * 3);
}
