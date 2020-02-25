# DetectWindows
DetectWindows from point clouds of buildings from mobile mapping drives. HighRes point clouds are required, e.g. higher than what a Velodyne64 laser can achieve. However, the issue with the Velodyne scanners might not just be the lower resolution but the organized structure of the data that could lead to singularities (not confirmed, just an assumption).

## Requirements
The code was developed on a Ubunutu 16.04 system and has only be build and tested on this system. Theoretically, the could should be able to run on Windows, if all the dependencies are fulfilled - this was not tested.

The following libraries and versions were used:

* PCL 1.8 (and dependencies, such as Boost 1.5.8, Eigen3 3.2.92, Flann 1.8.4, etc.) http://www.pointclouds.org/downloads or https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/
* libLAS 1.8.1 (and dependencies) https://liblas.org/download.html
* vtk 8.1 
* OpenCV 3.2.0

Note 1: You could remove the support for las files from the code in order to reduce the dependency on liblas because PCD and TXT files are also supported.

Note 2: VTK is used for visualization of the intermediate steps. For batch processing this would not be required and VTK might be removed as well. However, note that PCL might need to be rebuild without VTK support in this case.

Note 3: Only few routines from the OpenCV library were used in order to project the detected window corner points onto a 2d plain and estimate a bounding box. You might write your own routine to do this to reduce dependencies. 

## General information about the software

**Input**: is a single point cloud file as *.PCD, *.las or *.txt/*.xyz (tested only with unstructured / unorganized point clouds but others might work as well).
Data needs to be stored in columns of X Y Z [intensity] for each point. Intensity is not used in this method.
You can easily create your own PCD files from simple TXT / XYZ files be manually adding the header info. For this please see the documentation of the PCD file format at: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
Note that PCD files do not store information about the spatial reference system (SRS), you could however add a comment into the header.
**Units in the input file are assumed to be in meters, not degrees.**

**Output**: Along the processing chain, certain point clouds are written to disc, such as the hull, the redued input point cloud in PCD format, and so on (this can all be commented out to reduce clutter).
Main outputs are: cloudWithWindows.pcd which contains all points of the contour found of the window found where each point beloing to the same window will have the same ID encoded as RGB values: e.g.
```
     X           Y           Z       R G B
358951.02199 5701437.075 132.3170307 8 8 8
358951.01914 5701437.074 132.6170413 8 8 8
....
```
Also there is `STEP_05_bBox.pcd` which contains only the corner points of each window. Each corner point of a particular window will have the same ID. In this file, there are always 4 points associated with each window, thus 4 points having the same ID. The difference here is also, that the IDs are not encoded as RGB but as single values, which could be viewed as the intensity channel. 

```
    X         Y         Z     I
57.277615 30.209835 17.021194 14
53.980743 28.266335 18.219912 16
53.968925 28.26738 17.02471 16
52.551678 27.431767 17.037991 16
52.563492 27.430721 18.233192 16
61.172234 32.497196 17.019188 18

```
Note that the **IDs are incremented by a value of two starting from ID 4**. This has historical reasons and can be changed. I did not get aorund it yet.

## Usage:
run from shell as: .detectWindows /home/of/your/DATAset.las [-setZero] [-downsample 0.4]
- setZero will center your point cloud

[] are optional. If point clouds are very large, e.g. > 10M points, than it is a good idea to downsample the data so the program runs more stable and much faster. For large number of input points and large facades, you can quickly run out of memorey!





## License
The MIT License
Copyright (c) 2018 Sven Schneider, Hochschule fuer Technik Stuttgart





