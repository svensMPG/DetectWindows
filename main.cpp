#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>  // std::ifstream
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

// Boost
#include <boost/filesystem.hpp>

// libLAS library
//#include <liblas/liblas.hpp>

// OpenCV
//#include <opencv2/core/core.hpp>
//#include <opencv2/viz/vizcore.hpp>


//include own libs and classes here
#include "./src/miscFunctions.hpp"
#include "./src/hull2d.hpp"
#include "./src/CondEuclClustering.hpp"
#include "./h/cloudsubtractor.h"
#include "./src/pcl_las_helper.hpp"
#include "./h/extractfacade.h"
#include "./h/areafromconcavehull.h"
#include "./h/removeconcavehullframe.h"

#define PI atan(1.)*4.0L

typedef pcl::PointXYZI PointT;

using namespace std;


void processFacades(std::vector< pcl::PointCloud<PointT> > facadeVec,
                    pcl::PointCloud<PointT>::Ptr cloudWithWindows)
{
    float facadeArea = 0;
    float windowArea = 0;



    std::vector<float> facadeAreaVec, windowAreaVec;

    for (int f=0 ; f < facadeVec.size(); f++)
    {
        pcl::PointCloud<PointT>::Ptr tmpFacade (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr invertedCloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr concaveCloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr lines (new pcl::PointCloud<PointT>);

        *tmpFacade = facadeVec.at(f);
        facadeArea = 0;
        windowArea = 0;

        AreaFromConcaveHull<PointT> AObj;
        AObj.setInputCloud(tmpFacade);
        //AObj.greedyArea(facadeArea);

        facadeAreaVec.push_back(facadeArea);

        // read CLoudSubtractor values from config file
        std::vector<float> params;
        readParamsFromFile("../cfg/cloudSubtractor.cfg", params, true);


        // create cloudSubtractor object and call the necessary input functions
        CloudSubtractor<PointT> cs(params[0], params[1], params[2]);
        /*
        CloudSubtractor<PointT> cs;
        cs.setPointIncrement(params[0]);  // grid spacing between points for the grid / plane creation
        cs.setKdTreeRadius(params[2]);      // kdtree search radius for point search in original point cloud for point cloud subtraction.
        cs.setKdTreeRadiusPCA(0.05);
        cs.setPlaneDistThreshold(params[1]);*/
        cs.setInputCloud(tmpFacade);
        cs.applyCloudSubtraction(invertedCloud);

        // run again for inverted cloud.
        applyHull(invertedCloud, concaveCloud);
        //viewer<PointT> (concaveCloud);

        RemoveConcaveHullFrame<PointT> rmFrame;
        rmFrame.setInputCloud(concaveCloud);
        rmFrame.setTollerance(0.1);
        rmFrame.apply(concaveCloud);
        viewer<PointT> (concaveCloud);

/*
        AreaFromConcaveHull<PointT> AreaObj;
        AreaObj.setInputCloud(tmpFacade);
        AreaObj.greedyArea();

        AreaObj.setInputCloud(concaveCloud);
        AreaObj.setLineDistThreshold(0.2);
        AreaObj.detectLines(lines);

        viewer<PointT> (lines);
*/

        CondEuclClustering(concaveCloud,cloudWithWindows);

        int maxLabel = 3; // assuming -2 and 2 for too small and too large clusters, respectively
        int nObj = 0;
        for (int p=0; p<concaveCloud->points.size(); p++)
            if (maxLabel < concaveCloud->points[p].intensity ){
                maxLabel = concaveCloud->points[p].intensity;
                nObj++;
            }


        int label = 4; //assuming first true label == 4, see CondEuclClustering
        pcl::PointCloud<PointT>::Ptr tmpWindow (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr boxCloud (new pcl::PointCloud<PointT>);   //store the four corners of the bounding box in here
        tmpFacade.reset();

        for (int obj=0; obj < nObj; obj++, label+=2 ) { //assuming min label == 4 and a label difference of 2

            // Create the filtering object
             pcl::PassThrough<PointT> pass;
             pass.setInputCloud (concaveCloud);
             pass.setFilterFieldName ("intensity");
             pass.setFilterLimits (label, label);  // get only one window with that specific label
             //pass.setFilterLimitsNegative (true);
             pass.filter (*tmpWindow);  // this variable stores the potential window points

            tmpWindow->width = tmpWindow->points.size();
            tmpWindow->height = 1;

            std::vector<Eigen::Vector3f> bBox;
            float aspectRatio, tmpA;
            findBoundingBox(tmpWindow, bBox);  // apply fit bounding box in openCV

            //windowArea += calcAreaFromBbox(bBox,aspectRatio);
            tmpA = calcAreaFromBbox(bBox,aspectRatio);
            if (aspectRatio > 0.33){
                windowArea += tmpA;
                std::cout << "accumulated window area: " << std::setprecision(4) << windowArea << std::endl;
            }
            else
                continue;


            for (int ptNr=0; ptNr < bBox.size(); ptNr++){  // for all 4 corners store one point in the the boxCloud variable
                PointT pt;
                Eigen::Vector3f tmp(bBox.at(ptNr) );
                pt.x = tmp(0);
                pt.y = tmp(1);
                pt.z = tmp(2);
                boxCloud->points.push_back( pt );  // store point
                //tmpFacadel->points.push_back( pt );
            }

            boxCloud->width = boxCloud->points.size();
            boxCloud->height = 1;

        }
        windowAreaVec.push_back(windowArea);

        std::cout<< "Total window Area / Facade area = " << windowArea << " / " <<
                    facadeArea << " = " << windowArea / facadeArea <<
                    " = window-to-wall ratio." <<
                    "But the facadeArea was calculated without windows." << std::endl;
        std::cout << "corrected ratio is: windowArea / (windowArea+FacadeArea)= " <<
                     windowArea / (windowArea+facadeArea) << std::endl;

        //viewer<PointT>(boxCloud);
        //viewer<PointT>(tmpWindow);

    }
}

int  main (int argc, char** argv){


   if (argc == 1){
        PCL_ERROR ("No file has been specified... aborting.");
        return -1;
    }

   //check if pcd or las file was given
   std::string extension = boost::filesystem::extension(argv[1]);
   bool isPCD = false;
   if ( (extension.compare(1,3,".pcd",1,3) == 0 ) ||  ( (extension.compare(1,3,".PCD",1,3)) == 0 ) )
      isPCD = true;
   else if ( ( (extension.compare(1,3,".las",1,3)) == 0 ) ||  ( (extension.compare(1,3,".LAS",1,3)) == 0 ) )
       isPCD = false;
   else{
       PCL_ERROR ("Wrong file extension. Aborting.\n .pcd or .las is required");
       return -1;
   }


 //  pcl::PointCloud<PointT>::Ptr cloudIn = boost::make_shared<pcl::PointCloud<PointT> >();
  pcl::PointCloud<PointT>::Ptr cloudIn (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr invertedCloud (new pcl::PointCloud<PointT>);



// Checking parameters and input arguments
  if (argc < 2){
      pcl::console::print_error ("Syntax is: %%s <LAS-INPUT>  \t are mandatory.\nOtional are: \n "
                                         "-setZero: setMinXYZ to zero  no value required. If flag is set it is applied. \n "
                                         "-downsample: float [0.02,1] \n", argv[0]);
      return -1;
   }

    // check optional parameters for subtracting the min Values of the coordinates from all x,y,z coordinates respectively
    // this is done to work around problems caused by the loss of precision using float data types. -> poor PCD point clouds.
    bool subtractMinVals = true;
    subtractMinVals = pcl::console::parse (argc, argv, "-setZero", subtractMinVals);

    // check if the user wants to downsample the point cloud using the voxelgrid function. Specfiy standard values for the leafSize.
    float gridLeafSize = 0.25;
    bool LeafSize_specified = pcl::console::find_switch (argc, argv, "-downsample");
    if (LeafSize_specified){
        pcl::console::parse (argc, argv, "-downsample", gridLeafSize);
        if (gridLeafSize > 1){
            pcl::console::print_highlight("Warning: LeafSize is out of bounds [0, 1]. Setting it to default to proceed (0.25).\n");
            gridLeafSize = 0.25;
        }
    }


    ///// end checking parameters.

    if (isPCD){  // if pcd no values will be subtracted because pcd cannot store this precision anyway
        // load pcd file
        if(pcl::io::loadPCDFile<PointT> (argv[1], *cloudIn) == -1) // load the file
         {
           PCL_ERROR ("Couldn't read file ");
           return -1;
         }

        if (gridLeafSize > 0.029 && gridLeafSize < 1 && LeafSize_specified){
          std::cout << "\nApplying Uniform downsampling with leafSize " << gridLeafSize << ". Processing...";

          pcl::UniformSampling<PointT> uniform_sampling;
          uniform_sampling.setInputCloud (cloudIn);
          uniform_sampling.setRadiusSearch (gridLeafSize); //the 3D grid leaf size
          uniform_sampling.filter(*cloudFiltered);
          pcl::copyPointCloud(*cloudFiltered, *cloudIn);  // cloud is given by reference so the downsampled cloud has to be copied in there

          /*
          std::string fileToWrite = string(argv[1]) + "_downsampled.pcd";
          std::cout << "Writing PCD output file: " << fileToWrite << std::endl;
          pcl::io::savePCDFile (fileToWrite, *cloudFiltered,true);
          std::cerr << "Saved " << cloudFiltered->points.size () << " Points to " << fileToWrite << std::endl;*/

        }


    }
    else{


        // store minvalues to restore the original coordinates later after processing is completed
        std::vector<double> minXYZValues;
        // reading in LAS file and returning a PCD cloud XYZI data.
        readLAS2PCD<PointT>(argv[1], cloudIn, minXYZValues, gridLeafSize, subtractMinVals);
    }

    viewer<PointT> (cloudIn);

    //extract planes from point cloud
    std::vector< pcl::PointCloud<PointT> > facadeVec;  //construct a vector containing the all vertical planes, i.e. hopfully all facades

    getFacades<PointT>(cloudIn, facadeVec, 0.4).applyFilter(facadeVec);  //template class  / function to obtain all vertical planes / facades

    processFacades(facadeVec, cloudFiltered);


    if (invertedCloud->points.size() > 0){
      pcl::PCDWriter writer;
      writer.write<PointT> ("inverted_subtracted_Cloud.pcd", *invertedCloud, true);
    }


  // tt.tic();
   //std::cout << "starting to compute something...." << std::endl;

   //applyHull(invertedCloud, concaveHull, convexHull);
   //applyHull(invertedCloud, concaveHull);

  // viewer<PointT> (concaveHull);
 //  std::cout << "Number of points after processing " << concaveHull->points.size () << " .\nAll in all it took " << tt.toc() / 1000. << " seconds." << std::endl;
 //  viewer<PointT> (convexHull);

 //  tt.tic();
  // std::cout << " starting conditional Euclidean clustering..." << std::endl;
 //  CondEuclClustering(invertedCloud,cloudFiltered);
  // std::cout << "Conditional Euclidean  clustering applied to " << invertedCloud->points.size () << " points took: " << tt.toc() / 1000. << " seconds." << std::endl;


/*

    //////////////////////////////////////////////////////////////
    // Visualization of keypoints along with the original cloud
     pcl::visualization::PCLVisualizer viewer("PCL Viewer");
     pcl::visualization::PointCloudColorHandlerCustom<PointT> keypoints_color_handler (concaveHull, 0, 255, 0);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler (cloudIn, 255, 0, 0);
     pcl::visualization::PointCloudColorHandlerCustom<PointT> outputfinal_color_handler (convexHull, 0, 0, 255);
     viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
     viewer.addPointCloud(cloudIn, cloud_color_handler, "cloud");

     viewer.addPointCloud(concaveHull, keypoints_color_handler, "keypoints");
     viewer.addPointCloud(convexHull, outputfinal_color_handler, "outputfinal");
     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "outputfinal");

     while(!viewer.wasStopped ())
     {
     viewer.spinOnce ();
     }
*/
    return (0);
}

