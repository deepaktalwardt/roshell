#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <chrono>

#include <roshell_graphics/roshell_graphics.h>

#define PESERVE_ASPECT 1

namespace roshell_graphics
{

class ImageViewerNode
{
  public:
    ImageViewerNode(std::string in_topic);
    ~ImageViewerNode();

  private:
    std::shared_ptr<roshell_graphics::RoshellGraphics> rg_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::string in_topic_;
    image_transport::Subscriber image_sub_;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

ImageViewerNode::ImageViewerNode(std::string in_topic)
{
  ros::NodeHandle nh; 
  in_topic_ = in_topic;
  rg_ = std::make_shared<roshell_graphics::RoshellGraphics>();
  it_ = std::make_shared<image_transport::ImageTransport>(nh);
  image_sub_ = it_->subscribe(in_topic_, 1, &ImageViewerNode::imageCallback, this);
}

ImageViewerNode::~ImageViewerNode()
{

}

void ImageViewerNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert image using cv_bridge
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

  struct winsize ws;
  ioctl(STDIN_FILENO, TIOCGWINSZ, &ws);

  cv::Size new_size;
  cv::Mat image_resized;

  if(PESERVE_ASPECT)
  {
    double s = std::min((double)ws.ws_row/image.rows, (double)ws.ws_col/image.cols);
    new_size = cv::Size(2 * image.cols * s, image.rows * s);
    image_resized = cv::Mat(new_size, CV_8UC3, cv::Scalar(0, 0, 0));
  }
  else // fulscreen
  {
    new_size = cv::Size(ws.ws_col, ws.ws_row);
    image_resized = cv::Mat(new_size, CV_8UC3, cv::Scalar(0, 0, 0));
  }

  cv::resize(image, image_resized, new_size);
  cv::imshow("debug_view", image);
  rg_->display_image(image_resized);

  // If we want to measure time
  // auto start = std::chrono::steady_clock::now();
  // auto end = std::chrono::steady_clock::now();
  // std::cout << std::endl << std::endl;
  // std::cout <<  std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  
}

}  // eof namespace


int main(int argc, char **argv)
{
  std::ios::sync_with_stdio(false);
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic; // topic with image, e.g. "/wide_stereo/right/image_raw"
  int bad_params = 0;
  bad_params += !pnh.getParam("in_topic", topic);
  if (bad_params > 0)
  {
    std::cout << "One or more parameters not set! Exiting." << std::endl;
    return 1;
  }

  roshell_graphics::ImageViewerNode ivn(topic);

  ros::spin();
}
