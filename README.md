[<img height="23" src="https://github.com/lh9171338/Outline/blob/master/icon.jpg"/>](https://github.com/lh9171338/Outline) Spherical Hough Transform
===

# 1. Introduction
This repository is the spherical Hough transform algorithm implemented in Windows, and you can also access the [Linux version](https://github.com/lh9171338/Spherical-Hough-Transform/tree/Linux).

![image](https://github.com/lh9171338/Spherical-Hough-Transform/blob/main/SphericalHoughTransform/image/figure.png)  

# 2. Requirements
* opencv-4.0.0
* opencv_contrib-4.0.0

The code is written and tested in Visual Studio 2019.

# 3. File Structure
```
SphericalHoughTransform:
  |-- image: the folder containing image files
  |-- include: the folder containing header files
  |-- param: the folder containing parameter files
  |-- src: the folder containing source files
  |-- PropertySheet.props: opencv configure file for Release mode
  |-- PropertySheetd.props: opencv configure file for Debug mode
```

| File | Description |
| --- | --- |
| SphericalCamera.h SphericalCamera.cpp| Realize the transformation of spherical image, fisheye image and 3D pointcloud  |
| pointcloud.h pointcloud.cpp | Implement a PointCloud class |
| hough.h hough.cpp | Realize the spherical Hough transform, including *HoughLinesProbabilistic* and *HoughLinesStandard* functions |
| main.cpp | A demo about how to carry out the spherical Hough transform |
