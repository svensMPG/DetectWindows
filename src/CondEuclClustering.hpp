#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>


typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZINormal PointTNormal;


bool
enforceIntensitySimilarity (const PointTNormal& point_a, const PointTNormal& point_b, float squared_distance)
{
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  else
    return (false);
}

bool
enforceCurvatureOrIntensitySimilarity (const PointTNormal& point_a, const PointTNormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal(point_a.normal);
  Eigen::Map<const Eigen::Vector3f> point_b_normal(point_b.normal);
  if (fabs (point_a.intensity - point_b.intensity) < 5.0f)
    return (true);
  if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
    return (true);
  return (false);
}

bool
customRegionGrowing (const PointTNormal& point_a, const PointTNormal& point_b, float squared_distance)
{

    // read in parameter file
    std::vector<float> params;
    readParamsFromFile("../cfg/customRegionGrowing.cfg", params, false);

    Eigen::Map<const Eigen::Vector3f> point_a_normal(point_a.normal);
    Eigen::Map<const Eigen::Vector3f> point_b_normal(point_b.normal);



    if (squared_distance < params[0] )
  {
    if (fabs (point_a.intensity - point_b.intensity) < params[1])
      return (true);
    if (fabs (point_a_normal.dot (point_b_normal)) < params[2])
      return (true);
  }
  else
  {
    if (fabs (point_a.intensity - point_b.intensity) < params[3])
      return (true);
  }
  return (false);

}

int
CondEuclClustering (pcl::PointCloud<PointT>::Ptr cloudXyz, pcl::PointCloud<PointT>::Ptr cloudOut)
{
  // Data containers used

  pcl::PointCloud<PointTNormal>::Ptr cloud_with_normals (new pcl::PointCloud<PointTNormal>);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
  pcl::search::KdTree<PointT>::Ptr search_tree (new pcl::search::KdTree<PointT>);
  pcl::console::TicToc tt;

  // read in parameter file
  std::vector<float> params;
  readParamsFromFile("../cfg/CondEuclidClustering.cfg", params, true);
  // the way it is used is prone to access violations as it will never checked if the params[value] really exists...



  // Set up a Normal Estimation class and merge data in cloud_with_normals
  std::cerr << "Computing normals...\n", tt.tic ();
  pcl::copyPointCloud (*cloudXyz, *cloud_with_normals);
  pcl::NormalEstimation<PointT, PointTNormal> ne;
  ne.setInputCloud (cloudXyz);
  ne.setSearchMethod (search_tree);
  ne.setRadiusSearch ( params[0] );
  ne.compute (*cloud_with_normals);
  std::cerr << ">> Done: " << tt.toc () / 1000.0  << " s\n";

  // Set up a Conditional Euclidean Clustering class
  std::cerr << "Segmenting to clusters...\n", tt.tic ();
  pcl::ConditionalEuclideanClustering<PointTNormal> cec (true);
  cec.setInputCloud (cloud_with_normals);
  cec.setConditionFunction (&customRegionGrowing);
  //cec.setConditionFunction(&enforceCurvatureOrIntensitySimilarity);
  cec.setClusterTolerance ( params[1] );
  cec.setMinClusterSize ( params[2] );
  cec.setMaxClusterSize ( params[3] );
  cec.segment (*clusters);
  cec.getRemovedClusters (small_clusters, large_clusters);
  std::cerr << ">> Done: " << tt.toc () / 1000.0 << " s\n";

  std::cerr << "Clusters within defined cluster size: " << clusters->size() << std::endl;
  std::cerr << "Clusters which are too LARGE: " << large_clusters->size() << std::endl;
  std::cerr << "Clusters which are too SMALL: " << small_clusters->size() << std::endl;

  // Using the intensity channel for lazy visualization of the output
  for (int i = 0; i < small_clusters->size (); ++i)
    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
      cloudXyz->points[(*small_clusters)[i].indices[j]].intensity = -2.0;
  for (int i = 0; i < large_clusters->size (); ++i)
    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
      cloudXyz->points[(*large_clusters)[i].indices[j]].intensity = +2.0;

  int label = 4;
  long nPtsOfAllClusters = 0;
  for (int i = 0; i < clusters->size (); ++i)
  {
    //int label = rand () % 8;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
      cloudXyz->points[(*clusters)[i].indices[j]].intensity = label;
    }
    nPtsOfAllClusters += (*clusters)[i].indices.size();
    label += 2;
  }

  //viewer<PointT>(cloudXyz);

  pcl::PointCloud<PointT>::Ptr cloud_withWindows (new pcl::PointCloud<PointT>);
  cloud_withWindows->width = nPtsOfAllClusters;
  cloud_withWindows->height = 1;
  cloud_withWindows->resize(cloud_withWindows->width );

  long k = 0;
  for (int i = 0; i < clusters->size (); ++i)
  {
    //int label = rand () % 8;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j){
      cloud_withWindows->points[k].intensity = cloudXyz->points[(*clusters)[i].indices[j]].intensity;
      cloud_withWindows->points[k].x = cloudXyz->points[(*clusters)[i].indices[j]].x;
      cloud_withWindows->points[k].y = cloudXyz->points[(*clusters)[i].indices[j]].y;
      cloud_withWindows->points[k].z = cloudXyz->points[(*clusters)[i].indices[j]].z;
      ++k;
    }  
  }

  cloud_withWindows->width = cloud_withWindows->points.size();
  cloud_withWindows->height = 1;


  pcl::copyPointCloud(*cloud_withWindows,*cloudOut);
  viewer<PointT>(cloud_withWindows);


  // Save the output point cloud
  std::cerr << "Saving...\n", tt.tic ();
  if (cloudXyz->points.size() > 0)
    pcl::io::savePCDFile ("EuclClusteringOutput.pcd", *cloudXyz);

  if (cloud_withWindows->points.size() > 0)
    pcl::io::savePCDFile ("OnlyWindows.pcd", *cloud_withWindows);

  std::cerr << ">> EuclidCLustering done... "<< "\n";

  return (0);
}
