#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>

namespace roshell_graphics
{

class Pcl2VisualizerNode
{
    public:
        Pcl2VisualizerNode(
            const std::string& in_topic,
            const int& cam_x,
            const int& cam_y,
            const int& cam_z,
            const int& cam_focal_distance);

        ~Pcl2VisualizerNode();

        void pcl_visualizer_callback(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud_msg);
    
    private:
        std::string in_topic_;
        std::shared_ptr<roshell_graphics::RoshellGraphics> rg_;
        std::shared_ptr<roshell_graphics::PerspectiveProjection> pp_;

        ros::Subscriber pcl_sub_;
};

Pcl2VisualizerNode::Pcl2VisualizerNode(
    const std::string& in_topic,
    const int& cam_x,
    const int& cam_y,
    const int& cam_z,
    const int& cam_focal_distance):
    in_topic_(in_topic)
{
    ros::NodeHandle nh;

    // RoshellGraphics object
    rg_ = std::make_shared<roshell_graphics::RoshellGraphics>();

    // PixelProjection Object
    roshell_graphics::Camera cam;
    Eigen::Vector3f cam_loc(cam_x, cam_y, cam_z);
    cam.location = cam_loc;
    cam.focal_distance = cam_focal_distance;
    pp_ = std::make_shared<roshell_graphics::PerspectiveProjection>(cam);

    pcl_sub_ = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>
        (in_topic_, 3, &Pcl2VisualizerNode::pcl_visualizer_callback, this);
}

Pcl2VisualizerNode::~Pcl2VisualizerNode()
{
}

void Pcl2VisualizerNode::pcl_visualizer_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in_cloud_msg)
{
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>
        points = in_cloud_msg->points;

    Eigen::Matrix3Xf points_in_world_frame(3, points.size());
    for (int i = 0; i < points.size(); i++)
    {
        points_in_world_frame.col(i) << points[i].x, points[i].y, points[i].z;
    }

    Eigen::Matrix3Xf points_in_image_plane_with_z_world = pp_->project_multiple_world_points_with_z_world(points_in_world_frame);

    rg_->clear_buffer();
    rg_->add_points(points_in_image_plane_with_z_world);

    rg_->draw();
}

}  // namespace roshell_graphics

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string in_topic = "";
    int cam_x, cam_y, cam_z, cam_focal_distance;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_topic", in_topic);
    bad_params += !pnh.getParam("cam_x", cam_x);
    bad_params += !pnh.getParam("cam_y", cam_y);
    bad_params += !pnh.getParam("cam_z", cam_z);
    bad_params += !pnh.getParam("cam_focal_distance", cam_focal_distance);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    roshell_graphics::Pcl2VisualizerNode pvn(
        in_topic,
        cam_x,
        cam_y,
        cam_z,
        cam_focal_distance);

    ros::spin();
    return 0;
}