#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>


void pcd_visualizer(
    roshell_graphics::RoshellGraphics& rg,
    roshell_graphics::PerspectiveProjection& pp,
    std::string in_pcd_path)
{
    pcl::PCLPointCloud2 in_cloud_blob;
    pcl::io::loadPCDFile(in_pcd_path, in_cloud_blob);

    pcl::PointCloud<pcl::PointXYZ> in_cloud;
    pcl::fromPCLPointCloud2(in_cloud_blob, in_cloud);

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> points = in_cloud.points;

    Eigen::Matrix3Xf points_in_world_frame(3, points.size());
    for (int i = 0; i < points.size(); i++)
    {
        points_in_world_frame.col(i) << points[i].x, points[i].y, points[i].z;
    }
    
    Eigen::Matrix2Xf points_in_image_plane = pp.project_multiple_world_points(points_in_world_frame);
    rg.add_points(points_in_image_plane);
    rg.draw();
}


int main(int argc, char** argv)
{   
    // RoshellGraphics object
    roshell_graphics::RoshellGraphics rg;

    // PixelProjection Object
    roshell_graphics::Camera cam;
    Eigen::Vector3f cam_loc(10, 10, 10);
    cam.location = cam_loc;
    cam.focal_distance = 200;
    roshell_graphics::PerspectiveProjection pp(cam);

    // Load and test a PointCloud PCD file
    std::string in_pcd_path = "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE220/Project/roshell/ros_ws/src/roshell_graphics/test/test_pcd.pcd";
    pcd_visualizer(rg, pp, in_pcd_path);

    return 0;
}
