#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <roshell_graphics/roshell_graphics.h>
#include <roshell_graphics/perspective_projection.h>
#include <roshell_graphics/line_plotting.h>

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
        std::shared_ptr<roshell_graphics::PlotGraph> pg_;

        std::vector<float> points_;
};

FloatVisualizer::FloatVisualizer(
    const std::string& topic,
    const float& min_val,
    const float& max_val):
    nh_(),
    min_val_(min_val),
    max_val_(max_val)
{
    sub_ = nh_.subscribe<std_msgs::Float32>(topic, 10, &FloatVisualizer::callback, this);
    pg_ = std::make_shared<roshell_graphics::PlotGraph>();
}

FloatVisualizer::~FloatVisualizer()
{
}

void FloatVisualizer::callback(const std_msgs::Float32::ConstPtr& msg)
{
    pg_->clear_buffer();

    std::string ylabel = "Y-axis";

    points_.push_back(msg->data);

    pg_->plot_points(points_, min_val_, max_val_, ylabel);
    pg_->draw();
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
