// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
//#include "quiz/ransac/ransac2d.cpp"

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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudfiltered (new pcl::PointCloud<PointT>);
    // Create the filtering object: downsampling the dataset using a leaf size of .2m
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudfiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudfiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    typename pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative(true); // remove the points
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{


    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	

	while(maxIterations--){
		std::unordered_set<int> inliers;
		while(inliers.size() < 3){
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr++;

		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;

		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;



		float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		float d = -(a*x1+b*y1+c*z1);

		for (int index = 0; index < cloud->points.size(); index++){
			if(inliers.count(index)>0){
				continue;
			}

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float dist = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

			if(dist<=distanceTol){
				inliers.insert(index);
			}

		}

		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	
	return inliersResult;

}






template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>&cluster, std::vector<bool>&processed, KdTree* tree, float distanceTol)
{

	processed[indice]= true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
		{
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}


template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int min_size, int max_size)
{

	//TODO: Fill out this function to return list of indices for each cluster
    auto startTime = std::chrono::steady_clock::now();
    KdTree* tree = new KdTree;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters_cloud;
    std::vector<std::vector<float>> vect_pots;

	std::vector<bool>processed(cloud->points.size(), false);

    for(int j=0; j<cloud->size(); j++){
        std::vector<float> pt =  {cloud->points[j].x, cloud->points[j].y, cloud->points[j].z};
        tree->insert(pt,j);
        vect_pots.push_back(pt);
    }

	int i=0;
	while(i<cloud->size())
	{
		if(!processed[i])
		{

		    std::vector<int> cluster;
		    clusterHelper(i, vect_pots, cluster, processed, tree, distanceTol);

            if (cluster.size() > min_size && cluster.size() < max_size){
		        //clusters.push_back(cluster);
                typename pcl::PointCloud<PointT>::Ptr clusters{new pcl::PointCloud<PointT>};

                for(auto ind : cluster){
                    clusters->points.push_back(cloud->points[ind]);
                }


                clusters->is_dense = true;
                clusters->width = clusters->points.size();
                clusters->height = 1;
                clusters_cloud.push_back(clusters);

            }

		}

		i++;
	}
     auto endTime = std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclideanCluster took " << elapsedTime.count() << " milliseconds" << std::endl;
	return clusters_cloud;

}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
        auto startTime = std::chrono::steady_clock::now();

  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    // for (int index : inliers->indices)
    //     {
    //         planeCloud->points.push_back(cloud->points[index]);
    //     }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*planeCloud);
    extract.setNegative(true);
    extract.filter (*obstCloud);

    std::cout << *obstCloud<<std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "SeperateClouds took " << elapsedTime.count() << " milliseconds" << std::endl;
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inliers_set = RansacPlane(cloud,maxIterations, distanceThreshold);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());//
    pcl::PointIndices::Ptr inliers_indices (new pcl::PointIndices());//

    for (int ind : inliers_set){
        inliers_indices->indices.push_back(ind);
    }

    if (inliers_indices->indices.size () == 0)
    {
    std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;

    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_indices,cloud);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
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