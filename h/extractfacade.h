#ifndef EXTRACTFACADE_H
#include <boost/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/pcl_visualizer.h>


#define EXTRACTFACADE_H


template<typename T>
class extractFacade : protected pcl::PCLBase<T>
{

public:

    extractFacade(typename pcl::PointCloud<T>::Ptr In, std::vector< typename pcl::PointCloud<T> > Out, float distanceThresh) :
        input_(In),
        facades_(Out),
        distThresh_(distanceThresh),
        grndOffset_ (1.)
        {};

    virtual ~extractFacade ()
    {

    }

    // public functions
    inline void setGrndOffset(float offset) {grndOffset_ = offset;}
    bool applyFilter( std::vector< typename pcl::PointCloud<T> > & Out) { detectFacades(); Out = facades_; }  //return true if within range else return false.
private:

    // member variables
     typename pcl::PointCloud<T>::Ptr input_;
     std::vector< pcl::PointCloud<T> > facades_;
     float distThresh_;
     float grndOffset_;

     // member functions
     void detectFacades();


};


template<typename T>
extractFacade<T> getFacades(typename pcl::PointCloud<T>::Ptr In,  std::vector< typename pcl::PointCloud<T> > Out, float distanceThresh)
{
    return extractFacade<T>(In, Out, distanceThresh);
}

template<typename T>
void extractFacade<T>::detectFacades()
{

        std::cout << "starting to detect planes in point cloud..." << std::endl;
        pcl::console::TicToc tt;
        tt.tic();


      typename pcl::PointCloud<T>::Ptr cloud_f (new pcl::PointCloud<T>);
      typename pcl::PointCloud<T>::Ptr cloudFiltered (new pcl::PointCloud<T>);
       typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
        typename pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);


        pcl::copyPointCloud(*input_,*cloudFiltered);

    // estimate normals in point cloud
        std::cerr << "Computing normals...\n", tt.tic ();
        pcl::copyPointCloud (*input_, *cloud_with_normals);
        pcl::NormalEstimationOMP<T, pcl::Normal> ne;
        // create kd tree datastructure for normal search
        typename pcl::search::KdTree<T>::Ptr search_tree (new pcl::search::KdTree<T>);
        ne.setInputCloud (input_);
        ne.setSearchMethod (search_tree);
        ne.setRadiusSearch ( 0.3 );
        ne.compute (*normals);
        pcl::concatenateFields(*input_, *normals,*cloud_with_normals);
        std::cerr << ">> Done: " << tt.toc () / 1000.0  << " s\n";

    // show cloud with normals
        boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer;
        pclViewer = normalVis<T>(input_, normals);
        while (!pclViewer->wasStopped ())
          {
            pclViewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
          }


        // extract individual clusters using regionGrowing, using Noramls and Curcature as criteria
        regionGrowing<T> (input_, normals);

        // Create the segmentation object for the planar model and set all the parameters
      typename pcl::SACSegmentation<T> seg;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      typename pcl::PointCloud<T>::Ptr cloudPlane (new pcl::PointCloud<T> ());

      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (500);
      seg.setDistanceThreshold (distThresh_);

      int i=0, nr_points = (int) cloudFiltered->points.size ();
      while (cloudFiltered->points.size () > 0.25 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloudFiltered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        typename pcl::ExtractIndices<T> extract;
        extract.setInputCloud (cloudFiltered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloudPlane);

        // see if the average z value of the detected plane is within +/- grndOffset_ (5m default) meters of the minPt of the entire pointcloud;
        pcl::CentroidPoint<T> centroid;
        T minPt;
        T maxPt;
        pcl::getMinMax3D(*cloudPlane, minPt, maxPt);
        // get coefficients of plane to get the normal vector to see if it is point upward (road) or sideways (vertical plane)
        const Eigen::Vector3f normVec (coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2));
        const Eigen::Vector3f w(0,0,1); // road normal vector. assuming road is parallel to x-y plane
        for (size_t c=0; c < cloudPlane->points.size(); c++)
            centroid.add(cloudPlane->points[c]);

        T centPt;
        centroid.get(centPt);
        //if ( (centPt.z < minPt.z + grndOffset_ &&  centPt.z > minPt.z - grndOffset_) && normVec.dot(w) > 0.9  )
        if ( normVec.dot(w) > 0.9  )
            std::cout << "road or non-facade detected. Not included into facade vector\n";
        else
            facades_.push_back(*cloudPlane);  // it is not a road surface


        std::cout << "PointCloud representing the planar component: " << cloudPlane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);

        std::stringstream ss;
        pcl::PCDWriter writer;
        ss << "STEP_01_SurfaceExtracted_" << i << ".pcd";
        writer.write<T> (ss.str (), *cloudPlane, false); //*

        //viewer<T> (cloudPlane);


        *cloudFiltered = *cloud_f;

       // viewer<T> (cloud_f);
     ++i;
      }

      /////////////////////////////////////////////////////////////////////////////////
      /// refine the found planes by reducing the distancethreshold to 0.1m. this will stay hard-coded.
      /// 1) copy all cloud clusters in a new cloud
      /// 2) perform the same plane estimation as above
      ///

      typename pcl::PointCloud<T>::Ptr combClusters (new pcl::PointCloud<T>);

      for (int c = 0; c< facades_.size() ; ++c ){
          typename pcl::PointCloud<T> tmpCloud;
          pcl::copyPointCloud(facades_.at(c),tmpCloud);
          for (size_t p=0; p < tmpCloud.points.size(); p++)
            combClusters->points.push_back( tmpCloud.points[p] );
      }
      combClusters->width = combClusters->points.size();
      combClusters->height = 1;

      pcl::copyPointCloud(*combClusters, *cloudFiltered);
      i=0;
      nr_points = cloudFiltered->points.size ();
      facades_.clear();
      seg.setDistanceThreshold (0.25);

      std::cout << "  \n";

      while (cloudFiltered->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloudFiltered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        typename pcl::ExtractIndices<T> extract;
        extract.setInputCloud (cloudFiltered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloudPlane);

        // see if the average z value of the detected plane is within +/- grndOffset_ (5m default) meters of the minPt of the entire pointcloud;
        pcl::CentroidPoint<T> centroid;
        T minPt;
        T maxPt;
        pcl::getMinMax3D(*cloudPlane, minPt, maxPt);
        // get coefficients of plane to get the normal vector to see if it is point upward (road) or sideways (vertical plane)
        const Eigen::Vector3f normVec (coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2));
        const Eigen::Vector3f w(0,0,1); // road normal vector. assuming road is parallel to x-y plane
        for (size_t c=0; c < cloudPlane->points.size(); c++)
            centroid.add(cloudPlane->points[c]);

        T centPt;
        centroid.get(centPt);
        if ( (centPt.z < minPt.z + grndOffset_ &&  centPt.z > minPt.z - grndOffset_) || normVec.dot(w) > 0.9 || cloudPlane->points.size() < 0.05 * nr_points )
            std::cout << "road or non-facade detected. Not included into facade vector\n";
        else
            facades_.push_back(*cloudPlane);  // it is not a road surface


        std::cout << "PointCloud representing the planar component: " << cloudPlane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);

        *cloudFiltered = *cloud_f;
      }



      for(int j = 0; j < facades_.size(); j++)
      {       
        std::stringstream ss;
        pcl::PCDWriter writer;
        ss << "STEP_02_cloud_cluster_" << j << ".pcd";
        writer.write<T> (ss.str (), facades_[j], false); //*
        typename pcl::PointCloud<T>::Ptr tmpFacade (new pcl::PointCloud<T>);
        pcl::copyPointCloud(facades_.at(j), *tmpFacade);
        viewer<T> (tmpFacade);

      }
      std::cout << "detecting planes took : " << tt.toc()/1000 << " seconds. " << std::endl;

}


#endif // EXTRACTFACADES_H
