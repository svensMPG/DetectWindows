#ifndef REMOVECONCAVEHULLFRAME_H
#define REMOVECONCAVEHULLFRAME_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>



template <typename PointInT>
class RemoveConcaveHullFrame: public pcl::PCLBase<PointInT>
{

protected:
    typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointInT>::ConstPtr PointCloudConstPtr;


public:

    typedef boost::shared_ptr< RemoveConcaveHullFrame<PointInT> > Ptr;
    typedef boost::shared_ptr< const RemoveConcaveHullFrame<PointInT> > ConstPtr;


    RemoveConcaveHullFrame () :
          tollerance_ (0.0),
          hullFrame_ (new pcl::PointCloud<PointInT>),
          cloudWithoutFrame_ (new pcl::PointCloud<PointInT>)
          { };



    inline void setTollerance(float toll) {tollerance_ = toll;}

    void apply(typename pcl::PointCloud<PointInT>::Ptr cloudOut);


    virtual ~RemoveConcaveHullFrame ()
    {

    }

protected:


    float tollerance_;
    /** \brief An input point cloud describing the surface that is to be used for nearest neighbors estimation. */
    using pcl::PCLBase<PointInT>::input_;
    /** \brief The subtracted (inverted) point cloud. */
    typename pcl::PointCloud<PointInT>::Ptr hullFrame_;
    typename pcl::PointCloud<PointInT>::Ptr cloudWithoutFrame_;


};

template <typename PointInT>
void RemoveConcaveHullFrame<PointInT>::apply(typename pcl::PointCloud<PointInT>::Ptr cloudOut){

    typename pcl::PointCloud<PointInT>::Ptr cloudFiltered (new pcl::PointCloud<PointInT>);
    typename pcl::PointCloud<PointInT>::Ptr cloudFiltered2 (new pcl::PointCloud<PointInT>);

    //get extension of point cloud
    PointInT minPt;
    PointInT maxPt;
    pcl::getMinMax3D(*input_, minPt, maxPt);

    // add and subtract a small tolerance to the determined bounds to generate a wider range
    // to be sure to filter all the outside borders

    if (tollerance_ != 0){
        minPt.x = minPt.x + tollerance_;
        minPt.y = minPt.y + tollerance_;
        minPt.z = minPt.z + tollerance_;

        maxPt.x = maxPt.x - tollerance_;
        maxPt.y = maxPt.y - tollerance_;
        maxPt.z = maxPt.z - tollerance_;
    }

    // Create the filtering object
      pcl::PassThrough<PointInT> pass;
      pass.setInputCloud (input_);
      pass.setFilterFieldName ("x");        // filter in x
      pass.setFilterLimits (minPt.x, maxPt.x);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloudFiltered);

      // filter in y
      pass.setInputCloud (cloudFiltered);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (minPt.y, maxPt.y);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloudFiltered2);


      // filter in z
      pass.setInputCloud (cloudFiltered2);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (minPt.z, maxPt.z);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloudOut);
      //viewer<PointInT> (cloudOut);


}

#endif // REMOVECONCAVEHULLFRAME_H
