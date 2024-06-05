#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <librealsense2/rs.hpp> // Include the RealSense Cross Platform API

int main() {
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    p.start(cfg);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0);

    int v1(0), v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    while (!viewer->wasStopped()) {
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        auto vertices = points.get_vertices();
        cloud->width = static_cast<uint32_t>(points.size());
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            if (vertices[i].z) {  // Ensure depth is valid
                cloud->points[i].x = vertices[i].x;
                cloud->points[i].y = vertices[i].y;
                cloud->points[i].z = vertices[i].z;
            }
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_filtered);

        viewer->removeAllPointClouds(v1);
        viewer->removeAllPointClouds(v2);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud", v1);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "filtered cloud", v2);
        viewer->spinOnce();
    }

    return 0;
}
