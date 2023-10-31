#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

inline void viewKalmanResult(std::vector<Eigen::Vector2d> measure_pos, std::vector<Eigen::Vector2d> kalman_pos)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int color_map[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};

    pcl::PointCloud<pcl::PointXYZ>::Ptr measureCld(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr kalmanCld(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < measure_pos.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = measure_pos[i](0);
        p.y = measure_pos[i](1);
        p.z = 0.0;
        measureCld->push_back(p);
    }
    for(int i = 0; i < kalman_pos.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = kalman_pos[i](0);
        p.y = kalman_pos[i](1);
        p.z = 0.0;
        kalmanCld->push_back(p);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> measureColor(measureCld, color_map[0][0] * 255, color_map[0][1] * 255, color_map[0][2] * 255);
    viewer->addPointCloud(measureCld, measureColor, "measure_pos");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "measure_pos");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kalmanColor(kalmanCld, color_map[1][0] * 255, color_map[1][1] * 255, color_map[1][2] * 255);
    viewer->addPointCloud(kalmanCld, kalmanColor, "kalman_pos");
    for(int i = 0 ; i < kalmanCld->points.size()-1; i++)
    {
        viewer->addLine(kalmanCld->points[i],kalmanCld->points[i+1], color_map[1][0] * 255, color_map[1][1] * 255, color_map[1][2] * 255, "line" + std::to_string(i));
    }
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "kalman_pos");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return ;
}