# DetectWindows
DetectWindows from point clouds of buildings from mobile mapping drives. HighRes point clouds are required, e.g. higher than what a Velodyne64 laser can achieve. However, the issue with the Velodyne scanners might not just be the lower resolution but the organized structure of the data that could lead to singularities (not confirmed, just an assumption).

## Requirements
The code was developed on a Ubunutu 16.04 system and has only be build and tested on this system. Theoretically, the could should be able to run on Windows, if all the dependencies are fulfilled - this was not tested.

The following libraries and versions were used:

* PCL 1.8 
* libLAS 
* vtk
* OpenCV

Note 1: You could remove the support for las files from the code in order to reduce the dependency on liblas because PCD and TXT files are also supported.

Note 2: VTK is used for visualization of the intermediate steps. For batch processing this would not be required and VTK might be removed as well. However, note that PCL might need to be rebuild without VTK support in this case.

Note 3: Only few routines from the OpenCV library were used in order to project the detected window corner points onto a 2d plain and estimate a bounding box. You might write your own routine to do this to reduce dependencies. 

## General information about the software

**Input**: is a single point cloud file as *.PCD, *.las or *.txt/*.xyz (tested only with unstructured / unorganized point clouds but others might work as well).
Data needs to be stored in columns of X Y Z [intensity] for each point. Intensity is not used in this method.
You can easily create your own PCD files from simple TXT / XYZ files be manually adding the header info. For this please see the documentation of the PCD file format at: http://pointclouds.org/documentation/tutorials/pcd_file_format.php
Note that PCD files do not store information about the spatial reference system (SRS), you could however add a comment into the header.



## License
The MIT License
Copyright (c) 2018 Sven Schneider





