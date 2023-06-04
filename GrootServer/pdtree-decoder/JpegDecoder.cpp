#include "JpegDecoder.hpp"
#include <fstream>
#include <cstdio>
JpegDecoder::JpegDecoder()
{
}


void JpegDecoder::decode(vector<uint8_t> compressed_bytes, vector<uint8_t>& decoded_bytes)
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
