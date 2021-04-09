// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f maxPoint, Eigen::Vector4f minPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliners, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

  for(int index : inliners->indices){
      planeCloud->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices(inliners);
  extract.setNegative(true);
  extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    //pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliners {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment (*inliners, *coefficients);
    if(inliners->indices.size() == 0){
        std::cout << "could not estimate planar model" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliners,cloud);
    return segResult;
}

// The below function implements Ransac from scratch for a 3D case
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    std::unordered_set<int> inliersResult;

	
	// Ransac algorithm

	
	for (int i = 0; i < maxIterations; i++){

		std::unordered_set<int> inliers;
		int pt_1,pt_2, pt_3;

		pt_1 = rand() % cloud->points.size();
		pt_2 = rand() % cloud->points.size();
		pt_3 = rand() % cloud->points.size();

		while ((pt_1 == pt_2) || (pt_2 == pt_3) || (pt_1 == pt_3)){
			pt_2 = rand() % cloud->points.size();
			pt_3 = rand() % cloud->points.size();
			if(pt_2 == pt_3){
				pt_3 = rand() % cloud->points.size();
			}
		}

		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		x1 = cloud->points[pt_1].x;
		y1 = cloud->points[pt_1].y;
		z1 = cloud->points[pt_1].z;
		x2 = cloud->points[pt_2].x;
		y2 = cloud->points[pt_2].y;
		z2 = cloud->points[pt_2].z;
		x3 = cloud->points[pt_3].x;
		y3 = cloud->points[pt_3].y;
		z3 = cloud->points[pt_3].z;

		float v1[3] = {(x1-x2), (y1-y2), (z1-z2)};
		float v2[3] = {(x3-x2), (y3-y2), (z3-z2)};

		float cross_prod[3] = {(y1-y2)*(z3-z2)-(z1-z2)*(y3-y2), (z1-z2)*(x3-x2)-(z3-z2)*(x1-x2), (x1-x2)*(y3-y2)-(y1-y2)*(x3-x2) };
		float D = (-x2*cross_prod[0] -y2*cross_prod[1] - z2*cross_prod[2]);
		float mag = sqrt(cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1] + cross_prod[2]*cross_prod[2]);
		
		for (int idx = 0; idx < cloud->points.size(); idx++){

			auto point = cloud->points[idx];
			float d = fabs(cross_prod[0]*point.x + cross_prod[1]*point.y + cross_prod[2]*point.z + D)/mag;

			if(d < distanceTol){
				inliers.insert(idx);
			}
		}

		if(inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}
	}

    if(inliersResult.size() == 0){
        std::cout << "could not estimate planar model" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());


	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
    
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index,const std::vector<std::vector<float>>& points, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
  if (processed[index]){
    return;
  }
  processed[index] = true;
  cluster.push_back(index);
  std::vector<int> nearest = tree->search(points[index], distanceTol);
  
  for(int id : nearest){
    if(!processed[id]){
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
    }
  }

}

// The below function implements KDtree from scratch for a 3D case
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // create a KD tree from PCL corresponding to obstacle
	 KdTree* tree = new KdTree();

  	std::vector<std::vector<float>> points;
    for(PointT &p : cloud->points){
		points.push_back({p.x, p.y, p.z});
	}

    for (int i=0; i<points.size(); i++){
      tree->insert(points[i],i); 
    }
  
    //std::cout << "creating kdtree" << std::endl;

  	//std::vector<std::vector<int>> clusters;
    // vector of PCL pointers for each obstacle cluster
    std::vector<std::vector<int>> clusters_indices;
  	std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;
    std::vector<bool> processed(cloud->points.size(),false);
  
    int i = 0;
    while(i < cloud->points.size()){
      if (processed[i])
      {
        i++;
        continue;
      }
      
      //PCL pointers for a single obstacle cluster
      std::vector<int> cluster;
      clusterHelper(i, points, cluster, processed, tree, clusterTolerance);
      if(cluster.size() >= minSize && cluster.size() <= maxSize){
        clusters_indices.push_back(cluster);
      }
      i++;    
    }
    for(std::vector<int> cluster : clusters_indices){
      typename pcl::PointCloud<PointT>::Ptr clusterCloud(new typename pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit = cluster.begin (); pit != cluster.end (); ++pit){
        clusterCloud->points.push_back (cloud->points[*pit]);
      }
      clusterCloud->width = clusterCloud->points.size ();
      clusterCloud->height = 1;
      clusterCloud->is_dense = true;
      clusters.push_back(clusterCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_old(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndicies;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndicies);

    for(pcl::PointIndices getIndices: clusterIndicies){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}