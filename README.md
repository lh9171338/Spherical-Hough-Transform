[<img height="23" src="https://github.com/lh9171338/Outline/blob/master/icon.jpg"/>](https://github.com/lh9171338/Outline) Spherical Hough Transform
===

# 1. Introduction
This repository is the spherical Hough transform algorithm implemented in Ubuntu, and you can also access the [Windows version](https://github.com/lh9171338/Spherical-Hough-Transform/tree/main).

* Spherical Hough space
![image](https://github.com/lh9171338/Spherical-Hough-Transform/blob/main/SphericalHoughTransform/image/SphericalHoughSpace.png)  

* Procedures of the spherical Hough transform

![image](https://github.com/lh9171338/Spherical-Hough-Transform/blob/main/SphericalHoughTransform/image/Flowchart.png)  

# 2. Requirements
* opencv==4.4.0
* opencv_contrib==4.4.0
* CMake>=3.5

The code is written and tested in Ubuntu 16.04.

# 3. File Structure
```
SphericalHoughTransform:
  |-- image: the folder containing image files
  |-- include: the folder containing header files
  |-- param: the folder containing parameter files
  |-- src: the folder containing source files
  |-- main.cpp: the main program file
  |-- CMakeLists.txt: the CMake file
```

| File | Description |
| --- | --- |
| SphericalCamera.h SphericalCamera.cpp| Realize the transformation of spherical image, fisheye image and 3D pointcloud  |
| pointcloud.h pointcloud.cpp | Implement a PointCloud class |
| hough.h hough.cpp | Realize the spherical Hough transform, including *HoughLinesProbabilistic* and *HoughLinesStandard* functions |
| main.cpp | A demo about how to carry out the spherical Hough transform |
