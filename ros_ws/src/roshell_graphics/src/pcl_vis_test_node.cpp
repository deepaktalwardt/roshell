#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>


void test_pcd_projection(
    roshell_graphics::RoshellGraphics& rg,
    roshell_graphics::PerspectiveProjection& pp,
    std::string in_pcd_path)
{
    pcl::PCLPointCloud2 in_cloud_blob;
    pcl::io::loadPCDFile(in_pcd_path, in_cloud_blob);
}


int main(int argc, char** argv)
{   
    // RoshellGraphics object
    roshell_graphics::RoshellGraphics rg;

    // PixelProjection Object
    roshell_graphics::Camera cam;
    Eigen::Vector3f cam_loc(10000, 10000, 10000);
    cam.location = cam_loc;
    cam.focal_distance = 3000;
    roshell_graphics::PerspectiveProjection pp(cam);

    // Load and test a PointCloud PCD file
    std::string in_pcd_path = "/home/deepak/Dropbox/SJSU/Semesters/Spring2020/CMPE220/Project/roshell/ros_ws/src/roshell_graphics/test/test_pcd.pcd";
    test_pcd_projection(rg, pp, in_pcd_path);

    return 0;
}
