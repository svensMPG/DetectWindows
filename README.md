# DetectWindows
## General information
DetectWindows from point clouds of buildings from mobile mapping drives. HighRes point clouds are required, e.g. higher than what a Velodyne64 laser can achieve. However, the issue with the Velodyne scanners might not just be the lower resolution but the organized structure of the data that could lead to singularities (not confirmed, just an assumption).

Detailed explaination of the steps of the method can be found in the following paper (in German):

*Schneider, S., Coors, V. (2018). Automatische Extraktion von Fenstern in 3D Punktwolken mittels einer hierarchischen Methode. In T. P. Kersten, E. Gülch, J. Schiewe, T. H. Kolbe, & U. Stilla (Hrsg.), Wissenschaftlich-Technische Jahrestagung der DGPF und PFGK18 Tagung (pp. 559–572). Munich, Germany: Publikationen der DGPF.*
https://www.dgpf.de/src/tagung/jt2018/proceedings/proceedings/papers/44_PFGK18_P03_Schneider_Coors.pdf

View demo at: https://urbanvis.hft-stuttgart.de/EssenPC.html

**If you use this code for your work, please cite the work above.**

## Requirements
The code was developed on a Ubunutu 16.04 system and has only be build and tested on this system. Theoretically, the code should be able to be build and run on Windows, if all the dependencies are fulfilled - this was not tested.

The following libraries and versions were used:

* PCL 1.8 (and dependencies, such as Boost 1.5.8, Eigen3 3.2.92, Flann 1.8.4, etc.) http://www.pointclouds.org/downloads or https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/
* libLAS 1.8.1 (and dependencies) https://liblas.org/download.html
* vtk 8.1 
* OpenCV 3.2.0

Note 1: You could remove the support for las files from the code in order to reduce the dependency on liblas because PCD and TXT files are also supported.

Note 2: VTK is used for visualization of the intermediate steps. For batch processing this would not be required and VTK might be removed as well. However, note that PCL might need to be rebuild without VTK support in this case.

Note 3: Only few routines from the OpenCV library were used in order to project the detected window corner points onto a 2d plain and estimate a bounding box. You might write your own routine to do this to reduce dependencies. 

## Software description

**Input**: is a single point cloud file as *.PCD, *.las or *.txt/*.xyz (tested only with unstructured / unorganized point clouds but others might work as well).
Data needs to be stored in columns of X Y Z [intensity][R G B] for each point. Intensity and RGB is not used in this method.
You can easily create your own PCD files from simple TXT / XYZ files be manually adding the header info. For this please see the documentation of the PCD file format at: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
Note that PCD files do not store information about the spatial reference system (SRS), you could however add a comment into the header and prcesses this by writing your own routines.
**Units in the input file are assumed to be in meters, not degrees.**

**Output**: Along the processing chain, certain point clouds are written to disc, such as the (concave) hull, the reduced input point cloud in PCD format, and so on (this can all be commented out to reduce clutter).
Main outputs are: cloudWithWindows.pcd which contains all points of the contour found of the window found where each point beloing to the same window will have the same ID encoded as RGB values: e.g.
```
     X           Y           Z       R G B
358951.02199 5701437.075 132.3170307 8 8 8
358951.01914 5701437.074 132.6170413 8 8 8
....
```
Also there is `STEP_05_bBox.pcd` which contains only the corner points of each window. Each corner point of a particular window will have the same ID. In this file, there are always 4 points associated with each window, thus 4 points having the same ID. 

```
    X         Y         Z     I
358951.02199 5701437.075 132.3170307 8
358952.01914 5701436.074 132.6170413 8 
358953.02199 5701436.075 132.3170307 8 
358951.01914 5701437.074 132.6170413 8
```
Note that the **IDs are incremented by a value of two starting from ID 4**. This has historical reasons and can be changed. I did not get aorund it yet.
Also note that the **PCL only supports float data types** for point clouds (**not double**). This has some implications if data is stored in geographic SRS, as the values are very large and **precision will be lost. This leads to artifacts in your the data**. To solve this issue, data needs to be centered using the `-setZero` option (see Usage below).

## Usage:
run from shell as: .detectWindows /home/of/your/DATAset.las [-setZero] [-downsample 0.4]
- setZero will center your point cloud to Xmin, Ymin, Zmin to avoid artifacts for data with large coordinates 
- downsample FLOAT-VALUE : will reduce the voxel size by a given amount. Valid values are between [0.02,1]

[argument] are optional. If point clouds are very large, e.g. > 10M points, it is a good idea to downsample the data so the program runs more stable and much faster. For large number of input points and large facades, you can quickly run out of memorey!


## License
GNU GENERAL PUBLIC LICENSE Version 3
Copyright (c) 2018 Sven Schneider





