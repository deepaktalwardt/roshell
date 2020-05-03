#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace roshell_graphics
{

class FloatPublisher
{
    public:
        FloatPublisher(
            const std::string& topic,
            const float& min_val,
            const float& max_val,
            const int& rate);

        ~FloatPublisher();

        void run();
    
    private:
        ros::NodeHandle nh_;
        float min_val_;
        float max_val_;
        ros::Rate rate_;

        ros::Publisher pub_;
};

/**
 * Constructor
*/
FloatPublisher::FloatPublisher(
    const std::string& topic,
    const float& min_val,
    const float& max_val,
    const int& rate):
    nh_(),
    min_val_(min_val),
    max_val_(max_val),
    rate_(rate)
{
    pub_ = nh_.advertise<std_msgs::Float32>(topic, 10);
    run();
}

/**
 * Destructor
*/
FloatPublisher::~FloatPublisher()
{
}

/**
 * Starts publishing random number
*/
void FloatPublisher::run()
{
    srand((unsigned) time(0));
    while (ros::ok())
    {
        std_msgs::Float32 msg;

        float num = min_val_ +
            static_cast <float>(rand()) / (static_cast<float>(RAND_MAX / (max_val_ - min_val_)));

        msg.data = num;
        pub_.publish(msg);

        rate_.sleep();
    }
}



}  // namespace roshell_graphics

int main(int argc, char** argv)
{
    ros::init(argc, argv, "float_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic = "";
    int rate = 1;
    float max_val, min_val;

    int bad_params = 0;

    bad_params += !pnh.getParam("topic", topic);
    bad_params += !pnh.getParam("min_val", min_val);
    bad_params += !pnh.getParam("max_val", max_val);
    bad_params += !pnh.getParam("rate", rate);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    roshell_graphics::FloatPublisher fp(
        topic,
        min_val,
        max_val,
        rate);
    
    return 0;
}