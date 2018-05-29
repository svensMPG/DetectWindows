#ifndef AREAFROMCONCAVEHULL_H
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>



// STL
#include <tgmath.h>

// own
#include "removeconcavehullframe.h"
#include "../h/inrange.h"


#define AREAFROMCONCAVEHULL_H


template <typename PointInT>
class AreaFromConcaveHull: public  pcl::PCLBase<PointInT>
{

protected:
    typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;
    typedef typename pcl::PointCloud<PointInT>::ConstPtr PointCloudConstPtr;


public:

    typedef boost::shared_ptr< AreaFromConcaveHull<PointInT> > Ptr;
    typedef boost::shared_ptr< const AreaFromConcaveHull<PointInT> > ConstPtr;


    AreaFromConcaveHull () :
          lineDistThreshold_ (0.1),
          pointIncrement_ (0.1),
          area_ (0.0),
          kdRadius_ (1),
          coefficients_ (new pcl::ModelCoefficients),
          gridLine_ (new pcl::PointCloud<PointInT>),
          extractedLine_ (new pcl::PointCloud<PointInT>)
          { };

  //  inline void setPointIncrement(float inc) {pointIncrement_ = inc;}
    inline void setKdTreeRadius(float radius) {kdRadius_ = radius;}


    inline void setLineDistThreshold(float threshold) {lineDistThreshold_ = threshold;}


    void greedyArea(float &area);
    void detectLines(typename pcl::PointCloud<PointInT>::Ptr cloud_out);
    void calculateAreaFromHull(typename pcl::PointCloud<PointInT>::Ptr cloudOut);
    void makeGridLine();

    virtual ~AreaFromConcaveHull ()
    {

    }

protected:

    float kdRadius_;
    float area_;
    /** \brief The grid spacing in x,yz, direction. Smaller values lead to to larger and denser point clouds (default: 0.25). */
    const float pointIncrement_;

    float lineDistThreshold_;
    /** \brief An input point cloud describing the surface that is to be used for nearest neighbors estimation. */
    using pcl::PCLBase<PointInT>::input_;
    /** \brief The subtracted (inverted) point cloud. */
    typename pcl::PointCloud<PointInT>::Ptr extractedLine_;
    typename pcl::PointCloud<PointInT>::Ptr gridLine_;

    pcl::ModelCoefficients::Ptr coefficients_;


};

template <typename PointInT>
void AreaFromConcaveHull<PointInT>::greedyArea(float &area){

    // read in parameter file
    std::vector<float> params;
    readParamsFromFile("../cfg/GreedyArea.cfg", params, true);

        typename pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    typename pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormalsMLS (new pcl::PointCloud<pcl::PointNormal>);
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr inputXYZ (new pcl::PointCloud<pcl::PointXYZ>);
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr filteredXYZ (new pcl::PointCloud<pcl::PointXYZ>);

        // manually copy all points over to pointXYZ data type
        pcl::PointXYZ p;
        for (int i = 0; i < input_->points.size(); i++){
            p.x= input_->points[i].x;
            p.y= input_->points[i].y;
            p.z= input_->points[i].z;

            inputXYZ->points.push_back(p);
        }
        inputXYZ->width = inputXYZ->points.size();
        inputXYZ->height = 1;

        // Create the filtering object
          pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
          sor.setInputCloud (inputXYZ);
          sor.setMeanK (50);
          sor.setStddevMulThresh (0.75);
          sor.filter (*inputXYZ);


        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (params[0]);
        seg.setInputCloud (inputXYZ);
        seg.segment (*inliers, *coefficients_);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (inputXYZ);
            extract.setIndices (inliers);
            extract.setNegative (false);
//            extract.filter (*filteredXYZ);

           extract.filter (*inputXYZ);


           // estimate normals
           typename  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
           ne.setInputCloud (inputXYZ);
           typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
           ne.setSearchMethod (tree);
           // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch (params[1]);
            ne.compute (*cloudWithNormals);

            // concaternate the normals and the input xyz data
            pcl::concatenateFields(*inputXYZ, *cloudWithNormals, *cloudWithNormals);

/*
            typename pcl::search::KdTree<pcl::PointXYZ>::Ptr treemls (new pcl::search::KdTree<pcl::PointXYZ> ());

                        // Init object (second point type is for the normals, even if unused)


           //viewer<pcl::PointXYZ> (inputXYZ);
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
             mls.setInputCloud (inputXYZ);
             mls.setComputeNormals(true);
             mls.setSearchRadius (0.1);
             mls.setSearchMethod(treemls);
             mls.setPolynomialFit (false);
             //mls.setPolynomialOrder (2);
             //mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
             //mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
             //mls.setDilationVoxelSize (0.1);
             //mls.setDilationIterations(5);
             //mls.setUpsamplingRadius (0.5);
             //mls.setUpsamplingStepSize (0.3);
              // Reconstruct
              std::cout << "starting MovingLeastSqr Normal Estimation and Smoothing...";
              mls.process (*cloudWithNormalsMLS);
              std::cout << "Process completed." << std::endl;
             // viewer<pcl::PointXYZ> (*cloudWithNormalsMLS);
*/






         
           typename pcl::search::KdTree<pcl::PointNormal>::Ptr tree1 (new pcl::search::KdTree<pcl::PointNormal> ());
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp;
        pcl::PolygonMesh mesh;


        // GreedyTriangulation
        gp.setInputCloud(cloudWithNormals);
        gp.setSearchMethod(tree1);
        /*gp.setMu(2.20);
        gp.setSearchRadius(10.40);
        gp.setMaximumNearestNeighbors(500);
        gp.setMaximumSurfaceAngle(M_PI/4);
        gp.setMaximumAngle(3*M_PI/4);
        gp.setMinimumAngle(M_PI/18);
        */
        gp.setMu(params[2]);
        gp.setSearchRadius(params[3]);
        gp.setMaximumNearestNeighbors(params[4]);
        gp.setMaximumSurfaceAngle(params[5]);
        gp.setMaximumAngle(params[6]);
        gp.setMinimumAngle(params[7]);
        gp.setNormalConsistency(bool(params[8]));
        std::cout << "\nstarting reconstructing surface mesh..." << std::endl;
        std::cout << " \n";
        gp.reconstruct(mesh);
        std::cout << "Process completed." << std::endl;
        /**/

/*
         pcl::Poisson<pcl::PointNormal> poisson;
               poisson.setDepth(9);

               poisson.setInputCloud(cloudWithNormals);
               poisson.reconstruct(mesh);
*/
        cout<<"The size of polygons is "<<mesh.polygons.size()<<endl;
        pcl::io::saveVTKFile("potting-reconstruction.vtk",mesh);
        //pcl::io::savePLYFile("potting-reconstruction.ply",mesh);

        //calculate area;
        pcl::PointCloud<pcl::PointXYZ> cloud1;
        cloud1=*inputXYZ;

        int index_p1,index_p2,index_p3;
        float x1,x2,x3,y1,y2,y3,z1,z2,z3,a,b,c,q;


        for(int i=0;i<mesh.polygons.size();++i)
        {
              index_p1=mesh.polygons[i].vertices[0];
              index_p2=mesh.polygons[i].vertices[1];
              index_p3=mesh.polygons[i].vertices[2];

              x1=cloud1.points[index_p1].x;
              y1=cloud1.points[index_p1].y;
              z1=cloud1.points[index_p1].z;

              x2=cloud1.points[index_p2].x;
              y2=cloud1.points[index_p2].y;
              z2=cloud1.points[index_p2].z;

              x3=cloud1.points[index_p3].x;
              y3=cloud1.points[index_p3].y;
              z3=cloud1.points[index_p3].z;

                      //Heron's formula;
              a=sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
              b=sqrt(pow((x1-x3),2)+pow((y1-y3),2)+pow((z1-z3),2));
              c=sqrt(pow((x3-x2),2)+pow((y3-y2),2)+pow((z3-z2),2));
              q=(a+b+c)/2;

              float radizierer = q*(q-a)*(q-b)*(q-c);
              if (radizierer < 0)
                  std::cout << "negativ of sqrt not allowed" << std::endl; //area=area+sqrt(fabs(radizierer));
              else
                  area=area+sqrt(radizierer);

        }
        std::cout << area << std::endl;
        area_ = area;


        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer Greedy Mesh Reconstruction"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPolygonMesh(mesh,"meshes",0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }


}

template <typename PointInT>
void AreaFromConcaveHull<PointInT>::makeGridLine(){


    PointInT startPt, endPt, minPt, maxPt;
    pcl::getMinMax3D(*extractedLine_, minPt, maxPt);


    size_t nPts = extractedLine_->points.size();
    startPt = extractedLine_->points[0];
    endPt = extractedLine_->points[nPts-1];

    ////////// detect plane
    ////////////////////////////////
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointInT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (lineDistThreshold_);
    seg.setInputCloud (input_);
    seg.segment (*inliers, *coefficients_);

    std::cout << "Normalized coeffs of Pt-Normal Eq. (Ax+By+Cz=D): " << coefficients_->values[0] << " " << coefficients_->values[1] << " " << coefficients_->values[2] << " " << coefficients_->values[3] << std::endl;

    float z = (startPt.z + endPt.z) / 2.0;
    PointInT point;
    // construct new points on the plane by rearranging the point normal equation for one of the coordinates.
       for (float x=minPt.x; x<=maxPt.x; x+=pointIncrement_)
        {
          for (float y=minPt.y; y<= maxPt.y; y+=pointIncrement_)
          {

                point.x = x;

                if (coefficients_->values[1] != 0) // to avoid division by zero
                    point.y = -1.0 * (1.0 * coefficients_->values[3] + coefficients_->values[0] * x + coefficients_->values[2] * z  ) / coefficients_->values[1] ;
                else  // divisor is set to 0.0000001 if the coefficient is zero
                    point.y = -1.0 * (1.0 * coefficients_->values[3] + coefficients_->values[0] * x + coefficients_->values[2] * z  ) / 0.000001 ;

                point.z = z;
                gridLine_->points.push_back (point);
          }
        }

    std::cerr << "GridLine_ has: " << gridLine_->points.size () << " points. " <<  std::endl;
    viewer<PointInT> (gridLine_);




}

template <typename PointInT>
void AreaFromConcaveHull<PointInT>::detectLines(typename pcl::PointCloud<PointInT>::Ptr cloudOut){

    typename pcl::PointCloud<PointInT>::Ptr cloudFiltered (new pcl::PointCloud<PointInT>);

    pcl::copyPointCloud(*input_, *cloudFiltered);
    pcl::ExtractIndices<PointInT> extract;

      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);      
      // Create the segmentation object
      pcl::SACSegmentation<PointInT> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      //seg.setModelType (pcl::SACMODEL_LINE);
      seg.setModelType(pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (lineDistThreshold_);


      seg.setInputCloud (cloudFiltered);
      seg.segment (*inliers, *coefficients_);
      std::cerr << "PointCloud after LINE segmentation has: " << inliers->indices.size () << " inliers." << std::endl;


      // Extract the inliers
         extract.setInputCloud (cloudFiltered);
         extract.setIndices (inliers);
         extract.setNegative (false);
         extract.filter (*extractedLine_);
         viewer<PointInT> (extractedLine_);

         Eigen::VectorXf coeffs_;

          std::vector<int> indx;
          std::vector<double> distToModel;
          indx.push_back(*inliers->indices.begin());
          indx.push_back(*inliers->indices.end());
          pcl::SampleConsensusModelLine<PointInT> segLine(input_, false);
          segLine.setInputCloud(input_);
          segLine.computeModelCoefficients(indx,coeffs_);

          segLine.getDistancesToModel(coeffs_, distToModel);
          std::cout << coeffs_ << std::endl;



         makeGridLine();

         pcl::copyPointCloud(*gridLine_, *cloudOut);



}


#endif // AREAFROMCONCAVEHULL_H
