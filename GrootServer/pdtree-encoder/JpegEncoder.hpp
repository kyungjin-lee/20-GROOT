#pragma once

#include <cstdlib>
#include <iostream>
#include <cstdio>
#include "turbojpeg.h"
#include <vector>
#include <cstring>
using namespace std;

class JpegEncoder{
public:
    JpegEncoder();
    ~JpegEncoder(){ printf("[JPEG] encoder deleted\n");};

    int encode(vector<uint8_t> rgb_list, vector<uint8_t> &jpeg, int width, int height);
    void testDecoder(vector<uint8_t> compressed_bytes, vector<uint8_t>& decoded_bytes);
private:
    tjhandle handle_;
    int jpegQual_;
    int nbands_;
    int flags_;
    int pixelFormat_;
    int jpegSubsamp_;

};


