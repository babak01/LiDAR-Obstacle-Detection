/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"








void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_Cloud = pointProcessor->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

    auto segmentCloud = pointProcessor->SegmentPlane(filtered_Cloud, 25, 0.3);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
   renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, .53, 10, 500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters= pointProcessor->euclideanCluster(segmentCloud.first, 0.5,10,500);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,1), Color(1,1,0), Color(0,1,1) };

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {

        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);


        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);


        ++clusterId;
    }
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud");


}




//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY: viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side: viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}






int main(int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    //cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}