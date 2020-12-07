<div align="center">
  <img src="./logo.png" alt="">
  <h1>RHCL</h1>
  <blockquote> Rapid Hand-eye Calibration Library </blockquote>
</div>
### ​Introduction

**_RHCL_** is a Rapid Hand-eye Calibration Library.

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Hardware](#hardware)
- [Usage](#usage)
- [Contributor](#contributor)

### Modules

1. **RHCL::Camera** Camera module is used to process the point cloud from depth camera, such as Kinect, realsense
2. **RHCL::Model** Model module is used to obtain the point cloud which generated by the 3D model of manipulator
3. **RHCL::Registration** Registration module is used to process the two point clouds to get the RT-Matrix
4. **RHCL::Correction** Correction module is used to correct the error of RT-Matrix which caused by the camera or 3D model

### Dependencies

1. [PCL](https://pointclouds.org/) Point Cloud Library
2. [Qt](https://www.qt.io/cn) Qt GUI libraries
3. [OpenGL](https://www.opengl.org/) 3D visualization
4. [Assimp](https://www.assimp.org/) Loading 3D model format
6. [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) Eigen is a C++ template library for linear algebra (:warning: unused)
7. [Boost](https://www.boost.org/) Free peer-reviewed portable C++ source libraries (:warning: unfinished)
7. [OpenCV](https://opencv.org/) Processing RGBD image of Kinect (:warning: unfinished)

### Hardware

### Usage

```bash
$ git clone https://github.com/thinkexist1989/RHCL.git
$ cd RHCL/
$ mkdir build && cd build
$ cmake ..
$ make -j`nproc`
```

### Contributor

:bust_in_silhouette:[**Yang Luo (luoyang@sia.cn)**](mailto:luoyang@sia.cn), :bust_in_silhouette:[**Shun Su (sushun@sia.cn)**](mailto:sushun@sia.cn)
