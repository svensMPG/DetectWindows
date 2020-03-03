#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <numeric>




typedef pcl::PointXYZI PointT;


float
getAveragePointsInNeighborhood(pcl::PointCloud<PointT>::Ptr cloud, float searchRadius )
{
    // create a kdtree object
    pcl::KdTreeFLANN<PointT> kdtreeObj;
    // set the input cloud for the kdtree. this should start creating the tree.
    kdtreeObj.setInputCloud (cloud);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // determine size of PC before entering for, as the size changes by one during the loop.
    size_t nIters = cloud->points.size();

    std::vector<float> nPtsVec;
    int nPts; // number of points found within searchRadius. This will also be returned by function.


    #pragma omp parallel for // use parallel processing to speed up the process slightly. More cores would be better i guess...
    for (size_t p = 0; p < nIters; p++){

        //clear the vectors before each search.
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();

        const PointT searchPoint = cloud->points[p];

        // if no neighbours were found add point to result point cloud;.
        nPts = kdtreeObj.radiusSearch (searchPoint, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if ( nPts > 1 )
        {
/*
                    if (p % 10000 == 0)
                        std::cout << "Performed " << p+1 << " kdtree searches out of "
                              << nIters << ". " << std::setprecision(4) << "Found " << nPts
                              << " points within searchRadius of " << searchRadius << ". "
                              << float((p+1.))/float(nIters)*100.0 << " % completed." << std::endl;
*/
            nPtsVec.push_back((float)nPts);

        }
        else
        {
            nPts = -1;
        }

    }


    return std::accumulate(nPtsVec.begin(), nPtsVec.end(), 0.0f) / (float) nPtsVec.size();
}

int
applyHull (pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr hull, pcl::PointCloud<PointT>::Ptr convexhull )
{

    pcl::console::TicToc tt;
    tt.tic();

      pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);
      // read in parameter file
      std::vector<float> params;
      readParamsFromFile("../cfg/hull2d.cfg", params, true);



      // Create a Concave Hull representation of the projected inliers
      pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
      pcl::ConcaveHull<PointT> chull;
      chull.setInputCloud (cloud_filtered);
      chull.setAlpha (params[0]);

      int nPts = (int)getAveragePointsInNeighborhood(cloud_hull, 0.25 ); // param2 = searchDistance in meters
      std::cout << "number of average points within searchRadius found: " <<  nPts << "  alpha= " << chull.getAlpha() << std::endl;

      if (nPts <= 8)
          std::cout << "average number of neighboring points < 8. Continuing with processing... " << std::endl;
      else
      {
        std::cout << "average number of neighboring points > 8, increasing Alpha by 0.15 => Alpha: " << chull.getAlpha() + 0.15 << " and reapplying hullDetection... " << std::endl;
        chull.setAlpha (chull.getAlpha() + 0.1);
        chull.reconstruct (*cloud_hull);
      }

      pcl::copyPointCloud(*cloud_hull, *hull);

      std::cerr << "Concave hull has: " << cloud_hull->points.size ()
                << " data points." << std::endl;

      pcl::PCDWriter writer;
      writer.write ("ConcavHull.pcd", *cloud_hull, false);


      pcl::ConvexHull<PointT> convhull;
      convhull.setInputCloud (cloud_projected);
      convhull.setComputeAreaVolume( false );
      convhull.reconstruct (*cloud_hull);
      pcl::copyPointCloud(*cloud_hull, *convexhull);


      std::cerr << "Convex hull has: " << cloud_hull->points.size ()
                << " data points." << " Processing took: " <<
                   tt.toc()/1000.0 << " seconds." << std::endl;

      //writer.write ("Convexhull.pcd", *cloud_hull, false);

      return (0);

}


int
applyHull (pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr hull)
{

    pcl::console::TicToc tt;
    tt.tic();

//  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);

  // read in parameter file
  std::vector<float> params;
  readParamsFromFile("../cfg/hull2d.cfg", params, true);

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
  pcl::ConcaveHull<PointT> chull;
  chull.setInputCloud (cloud_filtered);
  chull.setAlpha (params[0]);

  chull.reconstruct (*cloud_hull);

  int nPts = (int)getAveragePointsInNeighborhood(cloud_hull, 0.25 ); // param2 = searchDistance in meters
  std::cout << "number of average points within searchRadius found: " <<  nPts << "  alpha= " << chull.getAlpha() << std::endl;

  if (nPts <= 8)
      std::cout << "average number of neighboring points < 8. Continuing with processing... " << std::endl;
  else
  {
    std::cerr << "average number of neighboring points > 8, increasing Alpha by 0.05 => Alpha: " << chull.getAlpha() + 0.05 << " and reapplying hullDetection... " << std::endl;
    chull.setAlpha (chull.getAlpha() + 0.05);
    chull.reconstruct (*cloud_hull);
  }





  //viewer<PointT>(cloud_hull);
  pcl::copyPointCloud(*cloud_hull, *hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << " Processing took: " <<
               tt.toc()/1000.0 << " seconds." << std::endl;

  pcl::PCDWriter writer;
  if (cloud_hull->size() > 0)
    writer.write ("ConcavHull.pcd", *cloud_hull, false);

  return (0);
}

