#ifndef CLOUDSUBTRACTOR_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include "../h/inrange.h"
#define CLOUDSUBTRACTOR_H

// by Sven Schneider 07/2017

template <typename PointInT>
class CloudSubtractor: public pcl::PCLBase<PointInT>
{

protected:    
    typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointInT>::ConstPtr PointCloudConstPtr;


public:

    typedef boost::shared_ptr< CloudSubtractor<PointInT> > Ptr;
    typedef boost::shared_ptr< const CloudSubtractor<PointInT> > ConstPtr;


    CloudSubtractor () :
          pointIncrement_ (0.25),
          planeDistThreshold_ (0.1),
          kdRadius_ (1),
          kdRadiusPCA_ (1),
          coefficients_ (new pcl::ModelCoefficients),
          gridPC_ (new pcl::PointCloud<PointInT>),
          invertedPC_ (new pcl::PointCloud<PointInT>)
          { };

    CloudSubtractor (float pointInc, float planeDist, float kdR) :
          pointIncrement_ (pointInc),
          planeDistThreshold_ (planeDist),
          kdRadius_ (kdR),
          kdRadiusPCA_ (1),
          coefficients_ (new pcl::ModelCoefficients),
          gridPC_ (new pcl::PointCloud<PointInT>),
          invertedPC_ (new pcl::PointCloud<PointInT>)
          { };

    inline void setPointIncrement(float inc) {pointIncrement_ = inc;}
    inline void setKdTreeRadius(float radius) {kdRadius_ = radius;}
    inline void setKdTreeRadiusPCA(float radius) {kdRadiusPCA_ = radius;}
    inline void setPlaneDistThreshold(float threshold) {planeDistThreshold_ = threshold;}
    inline float getPointIncrement() {return pointIncrement_;}    
    inline float getPlaneDistThreshold() {return planeDistThreshold_;}
    inline float getKdTreeRadius() {return kdRadius_;}

    void dummySearch();
    void applyCloudSubtraction(typename pcl::PointCloud<PointInT>::Ptr cloud_out);
    void pointwisePCA(typename pcl::PointCloud<PointInT>::Ptr cloudOut);




    virtual ~CloudSubtractor ()
    {

    }

protected:

    float kdRadius_;
    float kdRadiusPCA_;
    /** \brief The grid spacing in x,yz, direction. Smaller values lead to to larger and denser point clouds (default: 0.25). */
    float pointIncrement_;

    float planeDistThreshold_;
    /** \brief An input point cloud describing the surface that is to be used for nearest neighbors estimation. */
    using pcl::PCLBase<PointInT>::input_;
    /** \brief The subtracted (inverted) point cloud. */
    typename pcl::PointCloud<PointInT>::Ptr invertedPC_;
    typename pcl::PointCloud<PointInT>::Ptr gridPC_;

    pcl::ModelCoefficients::Ptr coefficients_;


    void detectPlane();
    void makePlaneFromCoefficients();
    void gridSubtraction();


};

template <typename PointInT>
void CloudSubtractor<PointInT>::dummySearch ()//, typename pcl::PointCloud<PointInT>::Ptr invertedPC_)
{
    //typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*input_,*invertedPC_);
    for (int i=0; i< input_->size(); ++i){
        invertedPC_->points[i].x = input_->points[i].x + 3;
        invertedPC_->points[i].y = input_->points[i].y + 5;
    }
}

template <typename PointInT>
void CloudSubtractor<PointInT>::applyCloudSubtraction (typename pcl::PointCloud<PointInT>::Ptr cloud_out)
{
    detectPlane();
    makePlaneFromCoefficients();
    gridSubtraction();
    //viewer<PointInT> (invertedPC_);
    pcl::copyPointCloud(*invertedPC_, *cloud_out);

}

template <typename PointInT>
void CloudSubtractor<PointInT>::detectPlane(){

    typename pcl::PointCloud<PointInT>::Ptr cloudFiltered (new pcl::PointCloud<PointInT>);
    typename pcl::PointCloud<PointInT>::Ptr tmpPlane (new pcl::PointCloud<PointInT>);


        pcl::console::TicToc tt;
/*      // Create the filtering object
        pcl::VoxelGrid<PointInT> sor;
        sor.setInputCloud (input_);
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        tt.tic();
        sor.filter (*cloud_filtered);
        std::cerr << "PointCloud after VoxelGrid Downsampling has: " << cloud_filtered->size() << " points. Took " << tt.toc() / 1000. << " s." << std::endl;
*/
    pcl::copyPointCloud(*input_,*cloudFiltered);



      //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointInT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (planeDistThreshold_);

      tt.tic ();
      seg.setInputCloud (cloudFiltered);
      seg.segment (*inliers, *coefficients_);
      std::cerr << "PointCloud after plane segmentation has: " << inliers->indices.size () << " inliers. Took " << tt.toc() / 1000. << " s." << std::endl;

      std::cout << "Normalized coeffs of Pt-Normal Eq. (Ax+By+Cz=D): " << coefficients_->values[0] << " " << coefficients_->values[1] << " " << coefficients_->values[2] << " " << coefficients_->values[3] << std::endl;

      //TODO: Detect 1 more plane: ~30 cm +/- 10cm before (direction of the street) the facadePlane
      // this is done to detect Erkers.
      //TODO: Combine both facadePoint clouds

      // For DEBUGING ONLY
      // Extract the planar inliers from the input cloud
      typename pcl::ExtractIndices<PointInT> extract;
      extract.setInputCloud (cloudFiltered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      // Get the points associated with the planar surface
      extract.filter (*tmpPlane);

      std::stringstream ss;
      pcl::PCDWriter writer;
      ss << "STEP_03_moreRefinedPlane_.pcd";
      writer.write<PointInT> (ss.str (), *tmpPlane, false); //*




}

template <typename PointInT>
void CloudSubtractor<PointInT>::makePlaneFromCoefficients()
{
    PointInT minPt;
    PointInT maxPt;
    pcl::getMinMax3D(*input_, minPt, maxPt);

    PointInT point;
    // construct new points on the plane by rearranging the point normal equation for one of the coordinates.
       for (float x=minPt.x; x<=maxPt.x; x+=pointIncrement_)
        {
          for (float y=minPt.y; y<= maxPt.y; y+=pointIncrement_)
          {
              for(float z=minPt.z; z <= maxPt.z ; z+=pointIncrement_)
              {

                point.x = x;

                if (coefficients_->values[1] != 0) // to avoid division by zero
                    point.y = -1.0 * (1.0 * coefficients_->values[3] + coefficients_->values[0] * x + coefficients_->values[2] * z  ) / coefficients_->values[1] ;
                else  // divisor is set to 0.0000001 if the coefficient is zero
                    point.y = -1.0 * (1.0 * coefficients_->values[3] + coefficients_->values[0] * x + coefficients_->values[2] * z  ) / 0.000001 ;

                point.z = z;
                gridPC_->points.push_back (point);
              }
          }
        }

        gridPC_->width = (int) gridPC_->points.size ();  gridPC_->height = 1;
        std::cout << "size of gridPlane: " << gridPC_->points.size() << std::endl;

        std::stringstream ss;
        pcl::PCDWriter writer;
        ss << "STEP_04_PlaneFromCoeffs.pcd";
        writer.write<PointInT> (ss.str (), *gridPC_, false); //*


}

template <typename PointInT>
void CloudSubtractor<PointInT>::gridSubtraction()
{
    pcl::console::TicToc tt;
    typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT>);
    pcl::copyPointCloud(*input_,*cloud);  // make a copy of original cloud

    // create a kdtree object
    pcl::KdTreeFLANN<PointInT> kdtreeObj;
    // set the input cloud for the kdtree. this should start creating the tree.
    kdtreeObj.setInputCloud (cloud);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // determine size of PC
    size_t nIters = gridPC_->size();

    //typename pcl::PointCloud<PointInT>::iterator pointIdxZERO;

    tt.tic();
    #pragma omp parallel for // use parallel processing to speed up the process slightly. More cores would be better i guess...
    for (size_t p = 0; p < nIters; p++){


        //clear the vectors before each search.
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();

        const PointInT searchPoint = gridPC_->points[p];

        // if no neighbours were found add point to result point cloud;.
        if ( !(kdtreeObj.radiusSearch (searchPoint, kdRadius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1) )
        {
                    invertedPC_->points.push_back(searchPoint);
                    if (p % 100000 == 0)
                        std::cout << "Performed " << p+1 << " kdtree searches out of "
                              << nIters << ". " << std::setprecision(2)
                              << float((p+1.))/float(nIters)*100.0 << " % completed..." << std::endl;

        }

    }

    std::cout << "gridSearch took: " << tt.toc() / 1000. << " seconds." << std::endl;
}


template <typename PointInT>
void CloudSubtractor<PointInT>::pointwisePCA(typename pcl::PointCloud<PointInT>::Ptr cloudOut)
{

    pcl::console::TicToc tt;
    typename pcl::PointCloud<pcl::PointNormal>::Ptr visCloud (new pcl::PointCloud<pcl::PointNormal>);

    // create a kdtree object
    pcl::KdTreeFLANN<PointInT> kdtreeObj;
    // set the input cloud for the kdtree. this should start creating the tree.
    kdtreeObj.setInputCloud (input_);
    pcl::IndicesPtr indices (new std::vector <int>);
    std::vector<float> pointRadiusSquaredDistance;

    // create PCA object
    pcl::PCA<PointInT> pcaObj;
    pcaObj.setInputCloud(input_);

    // determine size of PC before entering for, as the size changes by one during the loop.
    size_t cloudSize = input_->size();
    Eigen::Vector3f EigenVals;

    tt.tic();
    #pragma omp parallel for // use parallel processing to speed up the process slightly. More cores would be better i guess...
    for (size_t p = 0; p < cloudSize; p++){

        PointInT searchPoint = input_->points[p];
        pcl::PointNormal visPoint;  // this allows to store Lambda0,1,2 in the normal_x,_y,_z values

        // if neighbours were found, calculate PCA for this neighbourhood
        if ( (kdtreeObj.radiusSearch (searchPoint, kdRadiusPCA_, *indices, pointRadiusSquaredDistance) > 3) )
        {
                pcaObj.setIndices(indices);
                EigenVals = pcaObj.getEigenValues();

               const float alpha = EigenVals(0) +EigenVals(1) + EigenVals(2);


               // store planarity value in intensity field of point cloud for easy visualisation                
               searchPoint.intensity = EigenVals(0)/alpha;

                visPoint.x = searchPoint.x;
                visPoint.y = searchPoint.y;
                visPoint.z = searchPoint.z;                
                visPoint.normal_x = EigenVals(0)/alpha;
                visPoint.normal_y = EigenVals(1)/alpha;
                visPoint.normal_z = EigenVals(2)/alpha;


                // curvature is used to save to classification results based on thresholds
                if ( inrange(0.62, 0.7).contains(visPoint.normal_x) && inrange(0.17, 0.3).contains(visPoint.normal_y) && inrange(0.0, 0.1).contains(visPoint.normal_z)  )
                { // basically test if L0~2/3 and L1~1/3 and L2~0,  where L=Lambda (Eigenvalues)
                    visPoint.curvature = 0.25; // its a border point, labeled as 0.25
                    /*if (p % 1000 == 0)
                        std::cout << "Border" << std::endl;*/
                }

                else if ( inrange(0.20, 0.3).contains(visPoint.normal_x) && inrange(0.20, 0.3).contains(visPoint.normal_y) && inrange(0.1, 0.3).contains(visPoint.normal_z)  )
                { // basically test if L0 ~ L1 ~ L2 ~= 1/3
                    visPoint.curvature = 0.5; //its a corner point labeled as 0.5
                    /*if (p % 1000 == 0)
                        std::cout << "Corner" << std::endl;*/
                }
                else if ( inrange(0.8, 1.1).contains(visPoint.normal_x) && inrange(0.0, 0.1).contains(visPoint.normal_y) && inrange(0.0, 0.1).contains(visPoint.normal_z)  )
                {    // basically test if L0 ~ 1 and L2 L1 = 0
                    visPoint.curvature = 0.75; //its a line point labeld as 0.75
                    /*if (p % 1000 == 0)
                        std::cout << "Line" << std::endl;*/
                }
                else{
                    visPoint.curvature = 0.0; //its an inlier labeld as 0
                   /* if (p % 1000 == 0)
                       std::cout << "Inlier" << std::endl;  */
                }

                invertedPC_->points.push_back(searchPoint);  // push_back the point with planarity info into result PC
                visCloud->points.push_back(visPoint);  // cloud vor visualization, take generated point and add it to cloudvector

                // user info - output
                if (p % 100000 == 0)  // give processing info to the user...
                    std::cout << "PCA Performed. " << p+1 << " kdtree searches out of "
                          << cloudSize << ". " << std::setprecision(3)
                          << float((p+1.))/float(cloudSize)*100.0 << " % completed..." << std::endl;



        }
        indices->clear();  // clear indices for new neighbour search

    }
    // set basic cloud information.
    invertedPC_->width = invertedPC_->points.size();
    invertedPC_->height = 1;

    visCloud->width = visCloud->points.size();
    visCloud->height = 1;


    // write the data to a file. This will be removed in a final version... or?
    std::cout << "pointwisePCA took: " << std::setprecision(5) << tt.toc() / 1000. << " seconds." << std::endl;
    if (invertedPC_->points.size() > 0){
      pcl::PCDWriter writer;
      writer.write<PointInT> ("pca_cloud.pcd", *invertedPC_, false);
    }
    // view result. Will be removed in final version??
    viewer<PointInT>(invertedPC_);

  /////////////////////////////////////////////////
  // creating point cloud for visualization
  ///////////////////////////////////////////////
    if (visCloud->points.size() > 0){
      pcl::PCDWriter writer;
      writer.write<pcl::PointNormal> ("pca_visCloud.pcd", *visCloud, false);
    }

    for (size_t i=0; i < visCloud->points.size(); i++)
    {

        if (visCloud->points[i].curvature == 0.25)
        {
            pcl::PointXYZI borderPoint;
            borderPoint.x = visCloud->points[i].x;
            borderPoint.y = visCloud->points[i].y;
            borderPoint.z = visCloud->points[i].z;
            borderPoint.intensity = visCloud->points[i].normal_y;
            cloudOut->points.push_back(borderPoint);
        }
    }
    cloudOut->width = cloudOut->points.size();
    cloudOut->height = 1;

}

#endif // CLOUDSUBTRACTOR_H
