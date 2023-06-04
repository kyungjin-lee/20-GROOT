#include <cstdio>
#include <iostream>
#include <bitset>
#include <fstream>
#include <sstream>
#include <chrono>
#include <math.h>


#include "Decoder.hpp"

using namespace std;


int main(int argc, char* argv[])
{
    std::string inputpath = argv[1];
    std::string outputpath = argv[2];
    

    printf("Start collecting files\n");
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
    printf("Finish sorting\n");


    Decoder decoder;
    Manifest manifest;
    std::string manifestPath = "temp.txt";
    decoder.readManifest(manifestPath, manifest);
    for(int i = 0 ; i < filelist.size() ; i++)
    {
        printf("Generate %d frame\n", i);
        cout << " File name " << filelist[i] << endl;
        std::string outfile = outfilelist.at(i) + "_enc.bin";
        std::string ply_file = outfilelist.at(i) + "_dec.ply";
        decoder.decodePDTreeFile(filelist[i]);
        printf("[MAIN] Finished decoding\n");
        decoder.generatePointCloud(ply_file, manifest);
        decoder.write_decode(outfile);
    }


}
