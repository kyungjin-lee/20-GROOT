<div id="top"></div>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <h3 align="center">GROOT</h3>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
    </li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#citation">Citation</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
Source code for project "GROOT: A Real-time Streaming System of High-Fidelity Volumetric Videos" (MobiCom'20)


<p align="right">(<a href="#top">back to top</a>)</p>



### Built With

* [Point Cloud Library (PCL)](https://pointclouds.org/)
* [TurboJpeg](https://libjpeg-turbo.org/)
* [Zstandard](https://github.com/facebook/zstd)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started
Currently only offline testing only. Online streaming code coming soon.

### Dataset
Download dataset from [8i Voxelized Full Bodies](http://plenodb.jpeg.org/pc/8ilabs/)

### Server
Test encode/decode Groot on server

#### Prerequisites
Tested on Ubuntu 18.04
* Install PCL
'''
apt install libeigen3-dev libflann-dev libvtk6-qt-dev libpcap-dev  libboost-all-dev freeglut3-dev
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
tar zvfx pcl-1.9.1.tar.gz

cd pcl-pcl-1.9.1
mkdir build && cd build
cmake ..
make -j2
make -j2 install
'''

* Install TurboJpeg
'''
apt install libturbojpeg libturbojpeg0-dev
'''

* Install Zstandard
'''
git clone https://github.com/facebook/zstd.git
cd zstd
make
make install
'''

#### Test Encoding/Decoding
* Encoding
'''
# build
cd GrootServer/pdtree-encoding
mkdir build
cd build
cmake ..
make 

# refer to GrootServer/pdtree-encoding/run.sh for example
./server <dataset name> path/to/ply/files path/to/mortoncode path/to/output/folder 0 
'''

### Mobile
iOS-based Groot decoder and renderer

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Kyungjin Lee - [@kyungjinleeee](https://twitter.com/kyungjinleeee) - kjlee818@gmail.com

Project Link: [https://kyungjin-lee.github.io/groot/](https://kyungjin-lee.github.io/groot/)

<p align="right">(<a href="#top">back to top</a>)</p>


## Citation

If you find our work useful, please cite our paper below!
```
@inproceedings{10.1145/3372224.3419214,
author = {Lee, Kyungjin and Yi, Juheon and Lee, Youngki and Choi, Sunghyun and Kim, Young Min},
title = {GROOT: A Real-Time Streaming System of High-Fidelity Volumetric Videos},
year = {2020},
isbn = {9781450370851},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3372224.3419214},
doi = {10.1145/3372224.3419214},
booktitle = {Proceedings of the 26th Annual International Conference on Mobile Computing and Networking},
articleno = {57},
numpages = {14},
keywords = {point cloud, mobile augmented reality, virtual reality, volumetric video, video streaming},
location = {London, United Kingdom},
series = {MobiCom '20}
}
```
<!-- ADD your bibtex -->
<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

<p align="right">(<a href="#top">back to top</a>)</p>
