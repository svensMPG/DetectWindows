#ifndef PI
#define PI atan(1.)*4.0L
#endif

// standard c++
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/distances.h>



// openCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/viz/vizcore.hpp>



typedef pcl::PointXYZI PointT;

///////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void
viewer (typename pcl::PointCloud<PointInT>::Ptr &cloud)
{


  pcl::visualization::CloudViewer viewer ("View Point cloud");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
}


//////////////////////////////////////////////////////////////////////
/// \brief calcAreaFromBbox
/// \param bBox in
/// \return area of the bounding box
///
float calcAreaFromBbox(std::vector<Eigen::Vector3f> bBox, float &aspectRatio){

    pcl::PointXYZ p0,p1,p2;

    p0.x = bBox.at(0)[0];
    p0.y = bBox.at(0)[1];
    p0.z = bBox.at(0)[2];

    p1.x = bBox.at(1)[0];
    p1.y = bBox.at(1)[1];
    p1.z = bBox.at(1)[2];

    p2.x = bBox.at(2)[0];
    p2.y = bBox.at(2)[1];
    p2.z = bBox.at(2)[2];

    float s0 = pcl::euclideanDistance(p0,p1);
    float s1 = pcl::euclideanDistance(p1,p2);


    /*
    std::cout << s0 << std::endl;
    std::cout << s1 << std::endl;

*/

    if (s0 < s1){
        std::cout<< "aspect ratio s0/s1 = " << s0 / s1 << std::endl;
        aspectRatio=s0/s1;
    }
    else{
        std::cout<< "aspect ratio s1/s0 = " << s1 / s0 << std::endl;
        aspectRatio=s1/s0;
    }



    float area = fabs(s0 * s1);
    std::cout << "Window area: " << area << std::endl;
    return area;
}


///////////////////////////////////////////////////////////////////////////////////////

void
readParamsFromFile(std::string cfgFilename, std::vector<float> & params, bool printParameterNames){

// read in a parameter file so no recompilation is necessary to test parameters.
// create a file stream to the appropriate config file for the specfic method
std::fstream parameterFile;
parameterFile.open(cfgFilename);
// initiate a variable vector and a string of the variable name in the text file (for debug only, not used here after)
std::string discard;
//std::vector<float> params;
float value;

// perform the actual file reading.
int par=0;
if (parameterFile.is_open()){

    while (parameterFile >> discard >> value) {
      params.push_back(value);

      if (printParameterNames)
        std::cout << "reading " << cfgFilename << ": paramter-value # " << par+1 << " is: '" << discard << "' its value is: " << value << std::endl;

      par++;
    }
    parameterFile.close();
}
else
    std::cerr << "ERROR: could not open specified file: " << cfgFilename << ". Check path and spelling and make sure the file exists\n";


}


void
findBoundingBox(pcl::PointCloud<PointT>::Ptr &cloud, std::vector<Eigen::Vector3f> & table_top_bbx)
{
  //std::vector<Eigen::Vector3f> table_top_bbx;

  // Project points onto the table plane
  pcl::SACSegmentation<PointT> seg;
  pcl::ModelCoefficients::Ptr planeCoefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.25);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *planeCoefficients);

  pcl::ProjectInliers<PointT> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  pcl::PointCloud<PointT> projected_cloud;
  proj.setInputCloud(cloud);
  proj.setModelCoefficients(planeCoefficients);
  proj.filter(projected_cloud);

  // store the table top plane parameters
  Eigen::Vector3f plane_normal;
  plane_normal.x() = planeCoefficients->values[0];
  plane_normal.y() = planeCoefficients->values[1];
  plane_normal.z() = planeCoefficients->values[2];
  // compute an orthogonal normal to the plane normal
  Eigen::Vector3f v = plane_normal.unitOrthogonal();
  // take the cross product of the two normals to get
  // a thirds normal, on the plane
  Eigen::Vector3f u = plane_normal.cross(v);

  // project the 3D point onto a 2D plane
  std::vector<cv::Point2f> points;
  // choose a point on the plane
  Eigen::Vector3f p0(projected_cloud.points[0].x,
                      projected_cloud.points[0].y,
                      projected_cloud.points[0].z);
  for(unsigned int ii=0; ii<projected_cloud.points.size(); ii++)
  {
    Eigen::Vector3f p3d(projected_cloud.points[ii].x,
                         projected_cloud.points[ii].y,
                         projected_cloud.points[ii].z);

    // subtract all 3D points with a point in the plane
    // this will move the origin of the 3D coordinate system
    // onto the plane
     p3d = p3d - p0;

    cv::Point2f p2d;
    p2d.x = p3d.dot(u);
    p2d.y = p3d.dot(v);
    points.push_back(p2d);
  }


  cv::Mat points_mat(points);
  cv::RotatedRect rrect = cv::minAreaRect(points_mat);
  cv::Point2f rrPts[4];
  rrect.points(rrPts);

  /*
  double arclength = cv::arcLength(points, true);
  double contArea = cv::contourArea( points);

  double compactness = (4.*PI)* contArea / (4.*arclength*arclength);

  std::cout << "arclength: " << arclength << "  contArea: " <<
               contArea << "  compactness: " << compactness << std::endl;

*/

  //store the table top bounding points in a vector
  for(unsigned int ii=0; ii<4; ii++)
  {
    Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0);
    table_top_bbx.push_back(pbbx);
  }
 // Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
 // table_top_bbx.push_back(center);

}
