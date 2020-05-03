#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>

namespace roshell_graphics
{

class FloatVisualizer
{
    public:
        FloatVisualizer(
            const std::string& topic,
            const float& min_val,
            const float& max_val);
        
        ~FloatVisualizer();

        void callback(const std_msgs::Float32::ConstPtr& msg);
    
    private:
        ros::NodeHandle nh_;
        float min_val_;
        float max_val_;

        ros::Subscriber sub_;
};

FloatVisualizer::FloatVisualizer(
    const std::string& topic,
    const float& min_val,
    const float& max_val):
    nh_(),
    min_val_(min_val),
    max_val_(max_val)
{
    // TODO(Parshwa): Creating plotting object here

    sub_ = nh_.subscribe<std_msgs::Float32>
        (topic, 10, &FloatVisualizer::callback, this);
}

FloatVisualizer::~FloatVisualizer()
{
}

void FloatVisualizer::callback(const std_msgs::Float32::ConstPtr& msg)
{
    // TODO(Parshwa): Add plotting logic here
    std::cout << "Received: " << msg->data << std::endl;
}

}  // namespace roshell_graphics

int main(int argc, char** argv)
{
    ros::init(argc, argv, "float_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic = "";
    float max_val, min_val;

    int bad_params = 0;

    bad_params += !pnh.getParam("topic", topic);
    bad_params += !pnh.getParam("min_val", min_val);
    bad_params += !pnh.getParam("max_val", max_val);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    roshell_graphics::FloatVisualizer fv(
        topic,
        min_val,
        max_val);
    
    ros::spin();
    return 0;
}
