/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    

    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars,0);
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCld = lidar->scan();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(scanCld, 30,0.3);
    // renderRays(viewer, lidar->position ,scanCld);
    // renderPointCloud(viewer, scanCld, "scan");
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
    }
    
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    // TODO:: Create point processor
  
}
std::vector<BoxQ> tracking_objs;

float getCentroidDistance(BoxQ box1, BoxQ box2)
{
    return sqrt((box1.centroid(0) - box2.centroid(0))* (box1.centroid(0) - box2.centroid(0)) + (box1.centroid(1) - box2.centroid(1))* (box1.centroid(1) - box2.centroid(1)) + (box1.centroid(2) - box2.centroid(2))* (box1.centroid(2) - box2.centroid(2)));
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inCld)
{
    std::cout << "cityBlock start\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCld = pointProcessor->FilterCloud(inCld, .1, Eigen::Vector4f(-20, -7, -2, 1), Eigen::Vector4f(20, 7, 1, 1));
    std::pair<std::vector<float>, std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> > segmentCloud = pointProcessor->mySegmentPlane(filterCld, 30, 0.25);
    // renderPointCloud(viewer, inCld, "inCld", Color(0, 1, 0));
    std::cout << segmentCloud.first[0]<<", " << segmentCloud.first[1] << ", " << segmentCloud.first[2]  << std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->myClustering(segmentCloud.second.first, .8, 5, 300);
    // renderPointCloud(viewer, segmentCloud.first, "segmentCloud.first", Color(1, 0, 0));
    // renderPointCloud(viewer, segmentCloud.second, "segmentCloud.second", Color(0, 0, 1));
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->myClustering(inCld, 1.0, 10, 300);
    std::cout << "inCld->point.size() : " << inCld->points.size()<< std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};
    renderPointCloud(viewer, inCld, "inCld", Color(1, 1, 1));
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "inCld");
    std::vector<BoxQ> current_objs;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "obstCloud" + std::to_string(clusterId));
        // Box box = pointProcessor->BoundingBox(cluster);
        BoxQ box = pointProcessor->BoundingBoxPCA(segmentCloud.first, cluster);
        if(tracking_objs.size() == 0)
        {
            current_objs.push_back(box);
            renderBox(viewer, box, clusterId, colors[clusterId % colors.size()], 1.0, false);
        }
        else
        {
            bool found = false;
            for(int i = 0; i < tracking_objs.size();i++)
            {
                if(getCentroidDistance(tracking_objs[i], box) < 1.0)
                {
                    current_objs.push_back(box);
                    renderBox(viewer, box, clusterId, colors[clusterId % colors.size()], 1.0, true);
                    found = true;
                    break;
                }
            }
            if(!found)
            {
                current_objs.push_back(box);
                renderBox(viewer, box, clusterId, colors[clusterId % colors.size()], 1.0, false);
            }
        }
        ++clusterId;
    }
    tracking_objs = current_objs;
    // renderPointCloud(viewer,inCld,"inCld", Color(0,1,0));
    renderPointCloud(viewer, segmentCloud.second.second, "segmentCloud", Color(0, 1, 0));
    filterCld->clear();
    segmentCloud.second.first->clear();
    segmentCloud.second.second->clear();
    cloudClusters.clear();
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inCld = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCld = pointProcessor->FilterCloud(inCld, .5 , Eigen::Vector4f (-100, -10, -2, 1), Eigen::Vector4f ( 100, 10, 2, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCld, 30,0.7);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 10, 300);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId,colors[clusterId % colors.size()]);
        ++clusterId;
    }
    // renderPointCloud(viewer,inCld,"inputCloud");
    renderPointCloud(viewer,segmentCloud.second,"segmentCloud",Color(0,1,0));
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
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCreateData(std::vector<std::vector<float>> points)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i < points.size(); i++)
    {
        pcl::PointXYZI point;
        point.x = points[i][0];
        point.y = points[i][1];
        point.z = points[i][2];
        point.intensity = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}
int key_cnt =0 ;
void getchar_func()
{
    while(1)
    {
        getchar();
        key_cnt++;
    }
}
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    // std::vector<std::vector<float>> points;
    // for(float i = 0 ; i < 2 ; i +=0.9)
    // {
    //     for(float j = 0; j < 2; j+=0.9)
    //     {
    //         points.push_back({i,j,0});
    //     }
    // }

    // for(float i = 0 ; i < 2 ; i +=0.9)
    // {
    //     for(float j = 0; j < 2; j+=0.9)
    //     {
    //         points.push_back({i+10,j,3});
    //     }
    // }
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = tmpCreateData(points);

    std::thread key_input(getchar_func);
    int key_tmp = 0;
    while (!viewer->wasStopped ())
    {
        if(key_tmp != key_cnt)
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            streamIterator++;
            if (streamIterator == stream.end())
                streamIterator = stream.begin();

            inputCloud = pointProcessor->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessor, inputCloud);
            inputCloud->clear();
            key_tmp = key_cnt;
        }

        // usleep(10000);
        viewer->spinOnce ();
        
    } 
}
