#pragma once

#include <cstdlib>
#include <iostream>
#include <cstdio>
#include "turbojpeg.h"
#include <vector>
#include <cstring>
using namespace std;

class JpegDecoder{
public:
    JpegDecoder();
    ~JpegDecoder(){ printf("[JPEG] decoder deleted\n");};

    void decode(vector<uint8_t> compressed_bytes, vector<uint8_t>& decoded_bytes);
private:
    tjhandle handle_;
    int jpegQual_;
    int nbands_;
    int flags_;
    int pixelFormat_;
    int jpegSubsamp_;

};


