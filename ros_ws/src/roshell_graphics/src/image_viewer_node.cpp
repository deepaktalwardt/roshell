#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Uncomment if we want to measure time
// #include <chrono>  

#include <roshell_graphics/roshell_graphics.h>

namespace roshell_graphics
{

class ImageViewerNode
{
  public:
    ImageViewerNode(
        const std::string& in_topic,
        bool preserve_aspect = true);
    ~ImageViewerNode();

  private:
    std::shared_ptr<roshell_graphics::RoshellGraphics> rg_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::string in_topic_;
    bool preserve_aspect_;
    image_transport::Subscriber image_sub_;
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
};

ImageViewerNode::ImageViewerNode(
    const std::string& in_topic,
    bool preserve_aspect):
    in_topic_(in_topic),
    preserve_aspect_(preserve_aspect)
{
    ros::NodeHandle nh; 
    rg_ = std::make_shared<roshell_graphics::RoshellGraphics>();
    it_ = std::make_shared<image_transport::ImageTransport>(nh);
    image_sub_ = it_->subscribe(in_topic_, 1, &ImageViewerNode::image_callback, this);
}

ImageViewerNode::~ImageViewerNode()
{}

void ImageViewerNode::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert image using cv_bridge
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    rg_->clear_buffer();
    rg_->add_image(image, preserve_aspect_);
    rg_->draw();

    // If we want to measure time
    // auto start = std::chrono::steady_clock::now();
    // auto end = std::chrono::steady_clock::now();
    // std::cout << std::endl << std::endl;
    // std::cout <<  std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  
}

}   // namespace roshell_graphics 


int main(int argc, char **argv)
{
    std::ios::sync_with_stdio(false);
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic; // topic with image, e.g. "/wide_stereo/right/image_raw"
    bool preserve_aspect;
    int bad_params = 0;

    bad_params += !pnh.getParam("in_topic", topic);
    bad_params += !pnh.getParam("preserve_aspect", preserve_aspect);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    roshell_graphics::ImageViewerNode ivn(topic, preserve_aspect);  
    ros::spin();
}
