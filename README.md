<div align="center">
  <img src="./logo.png" alt="">
  <h1>RHCL</h1>
  <blockquote> Rapid Hand-eye Calibration Library </blockquote>
</div>

:cn: [中文说明](README_ZH.md)

### Introduction

**_RHCL_** is a Rapid Hand-eye Calibration Library.

![mv](./MotionViewer/res/mv.gif)

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Hardware](#hardware)
- [Usage](#usage)
- [Contributor](#contributor)

### Dependencies

1. [Qt](https://www.qt.io/cn) Qt GUI libraries (Need Modules : Widgets Network SerialPort Xml LinguistTools OpenGL )
2. [OpenGL](https://www.opengl.org/) 3D visualization
3. [Assimp](https://www.assimp.org/) Loading 3D model format
4. [OpenCV](https://opencv.org/) Processing RGBD image of Kinect (:warning: unfinished)
5. [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) Eigen is a C++ template library for linear algebra (:warning: unused)
6. [Boost](https://www.boost.org/) Free peer-reviewed portable C++ source libraries (:warning: unfinished)

### Hardware

1. NDI Spectra

<img src="MotionViewer/res/ndi-spectra.jpg" width="300" alt="NDI" />

1. HoloLens 1st

<img src="MotionViewer/res/HoloLens1.jpg" width="300" alt="HoloLens" />

3. Kinect v2

<img src="MotionViewer/res/kinect.jpg" width="300" alt="Kinect" />

### Usage

```bash
$ git clone https://github.com/thinkexist1989/MotionViewer.git
$ cd MotionViewer
$ mkdir build && cd build
$ cmake ..
$ make -j`nproc`
```

### Contributor

:bust_in_silhouette:[**Yang Luo (luoyang@sia.cn)**](mailto:luoyang@sia.cn), :bust_in_silhouette:[**Shun Su (sushun@sia.cn)**](mailto:sushun@sia.cn)
