#include <iostream>
#include <string>
#include <fstream>  // std::ifstream
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/uniform_sampling.h>


#include <liblas/liblas.hpp>


template <typename PointT>
int
readLAS2PCD(std::string fileToRead,
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            std::vector<double> minXYZValues,
            float gridLeafSize,
            bool subtractMinVals)
{

    // 1) create a file stream object to access the file
    std::ifstream ifs;
    ifs.open(fileToRead, std::ios::in | std::ios::binary);
    // if the LAS file could not be opend. throw an error (using the PCL_ERROR functionality).
    if (!ifs.is_open())
    {
        PCL_ERROR ("Couldn't read file ");
        return -1;
    }

    // set up ReaderFactory of the LibLAS library for reading in the data.
    std::cout << "Reading in LAS input file: " << fileToRead << std::endl;

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    liblas::Header const& header = reader.GetHeader();

    long int nPts = header.GetPointRecordsCount();
    std::cout << "Compressed:  " << (header.Compressed() == true) ? "true\n":"false\n";
    std::cout << "\nSignature: " << header.GetFileSignature() << '\n';
    std::cout << "Points count: " << nPts << '\n';



    // Fill in the PCD cloud data
    cloud->width    = nPts;
    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int i = 0;

    std::vector<double> PCPointsX;
    std::vector<double> PCPointsY;
    std::vector<double> PCPointsZ;


    double minX = 9999999999, minY= 9999999999, minZ = 9999999999;


    while (reader.ReadNextPoint()){
        liblas::Point const& p = reader.GetPoint();


        if (p.GetX() < minX)
            minX = p.GetX();

        if (p.GetY() < minY)
            minY = p.GetY();

        if (p.GetZ() < minZ)
            minZ = p.GetZ();

        // push in the original double values to subtract the minXYZ values from those
        // befor!! casting them to float.
        PCPointsX.push_back(p.GetX());
        PCPointsY.push_back(p.GetY());
        PCPointsZ.push_back(p.GetZ());

        // in case the user has not specified the -setZero flag. Take the values as they
        // are, without subtracting the min values and just cast them to float.
        cloud->points[i].x = (float) p.GetX();
        cloud->points[i].y = (float) p.GetY();
        cloud->points[i].z = (float) p.GetZ();
        cloud->points[i].intensity = p.GetIntensity();

        if (i % 500000 == 0)
            std::cout << i  << "  x: " << p.GetX() << "  y: " << p.GetY() << "  z: " << p.GetZ() << "\n";


        i++;
    }

    // store the minValues so they can be added in the final processing step to get the true coordinates
    minXYZValues.push_back(minX); minXYZValues.push_back(minY); minXYZValues.push_back(minZ);

    // if true, subtract the min values now. This will normally be 'true'
    if (subtractMinVals){
      pcl::console::print_highlight("\n\nRemoving min x,y,z coordinate values.\n\n");
      pcl::console::print_highlight("minX = %f  , minY = %f , minZ = %f \n\n", minX, minY, minZ);
      for (int i=0; i< cloud->points.size(); ++i){
          cloud->points[i].x = PCPointsX[i] - minX;
          cloud->points[i].y = PCPointsY[i] - minY;
          cloud->points[i].z = PCPointsZ[i] - minZ;

          if (i % 500000 == 0){
            std::cout << i  << "  x: " << cloud->points[i].x << "  y: " << cloud->points[i].y <<
                         "  z: " << cloud->points[i].z  << "    intensity:  " << cloud->points[i].intensity << std::endl;
          }
      }
    }


    pcl::console::TicToc time;
    time.tic ();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered =  boost::make_shared<pcl::PointCloud<PointT> >();


    if (gridLeafSize > 0.029 && gridLeafSize < 1){
      std::cout << "\nApplying Uniform downsampling with leafSize " << gridLeafSize << ". Processing...";

      pcl::UniformSampling<PointT> uniform_sampling;
      uniform_sampling.setInputCloud (cloud);
      uniform_sampling.setRadiusSearch (gridLeafSize); //the 3D grid leaf size
      uniform_sampling.filter(*cloudFiltered);
      std::cout << "Downsampled in " << time.toc() / 1000. << " s" << std::endl;
      pcl::copyPointCloud(*cloudFiltered, *cloud);  // cloud is given by reference so the downsampled cloud has to be copied in there

    }
    else  // else copy original cloud in cloud Filtered and save file...
        pcl::copyPointCloud(*cloud,*cloudFiltered);

    std::string fileToWrite = fileToRead + ".pcd";
    std::cout << "Writing PCD output file: " << fileToWrite << std::endl;
    pcl::io::savePCDFile (fileToWrite, *cloudFiltered,true);
    std::cerr << "Saved " << cloudFiltered->points.size () << " Points to " << fileToWrite << std::endl;


}

/*
void
viewer (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{


  pcl::visualization::CloudViewer viewer ("View Point cloud");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
}
*/


void readLAS2PCLsubMinVals(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::vector<double> &minValues,  int downSampleFactor ){

    // reading data from LAS file:
    // 1) create a file stream object to access the file

    std::ifstream ifs;
    ifs.open(filename.c_str(), std::ios::in | std::ios::binary);

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    liblas::Header const& header = reader.GetHeader();

    long int nPts = header.GetPointRecordsCount();
    std::cout << "Compressed:  " << (header.Compressed() == true) ? "true\n":"false\n";
    std::cout << "\nSignature: " << header.GetFileSignature() << '\n';
    std::cout << "Points count: " << nPts << '\n';


    minValues.push_back(header.GetMinX());
    minValues.push_back(header.GetMinY());
    minValues.push_back(header.GetMinZ());


    // Fill in the PCD cloud data
    if (downSampleFactor == 0){
        cloud->width    = nPts;
        downSampleFactor = 1;
        std::cerr << "downSampleFactor may not be zero. Applying no downsampling. " << std::endl;
    }
    else if (downSampleFactor == 1){
        cloud->width    = nPts;
    }
    else{
        cloud->width    = int (nPts/downSampleFactor)+1;
    }

    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);

    int i = 0;
    int k = 0;

    while (reader.ReadNextPoint()){
          liblas::Point const& p = reader.GetPoint();

          if (i % downSampleFactor == 0){
              cloud->points[k].x = p.GetX() - minValues[0];
              cloud->points[k].y = p.GetY() - minValues[1];
              cloud->points[k].z = p.GetZ() - minValues[2];
              cloud->points[k].intensity = p.GetIntensity();
              ++k;
          }

          i++;
      }

}
