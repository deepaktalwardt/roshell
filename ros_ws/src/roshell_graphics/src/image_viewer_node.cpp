#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>

#define PESERVE_ASPECT 1

std::string rgb_to_ascii(cv::Vec3b pixel)
{
  // set color using ANSI escape sequences
  // Excelent explanation here:
  // https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences

  cv::Vec3b p = pixel;
  std::string b = std::to_string(p[0]);
  std::string g = std::to_string(p[1]);
  std::string r = std::to_string(p[2]);
  return "\033[38;2;" + r + ";" + g + ";" + b + "m" + "█" + "\033[0m";
}

void display_image(cv::Mat im)
{
  std::stringstream buf;
  for(int r = 0; r < im.rows; r++)
  {
    for(int c = 0; c < im.cols; c++)
    {
      cv::Vec3b pixel = im.at<cv::Vec3b>(r, c);
      buf << rgb_to_ascii(pixel);
    }
    buf << std::endl;
  }
  std::cout << buf.str();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
  display_image(image_resized);

  // If we want to measure time
  // auto start = std::chrono::steady_clock::now();
  // auto end = std::chrono::steady_clock::now();
  // std::cout << std::endl << std::endl;
  // std::cout <<  std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cv::namedWindow("debug_view");
  std::string topic; // topic with image, e.g. "/wide_stereo/right/image_raw"

  int bad_params = 0;

  bad_params += !pnh.getParam("in_topic", topic);
  
  if (bad_params > 0)
  {
    std::cout << "One or more parameters not set! Exiting." << std::endl;
    return 1;
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  // ROS_INFO("Listening to: " << topic << std::endl);

  ros::spin();
  cv::destroyWindow("debug_view");
}
