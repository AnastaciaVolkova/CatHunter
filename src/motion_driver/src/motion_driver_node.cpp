#include <stdio.h>
#include <pigpio.h>
#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

using geometry_msgs::Twist;
using ros::Subscriber;

void SigintHandler(int sig){
    ros::shutdown();
    ROS_INFO("ends");
    gpioHardwarePWM(18, 0, 0);
    gpioTerminate();
    exit(sig);
}

class MotionDriver{
private:
    static constexpr float kMaxDuty = 1e6;
public:
    MotionDriver(){
        if (gpioInitialise() < 0)
        {
            ROS_ERROR("Unable to initialize gpio");
            throw std::runtime_error("Unable to initialize gpio");
        }
        gpioHardwarePWM(18, 0, 0);
    };

    void SetVelocity(const Twist& velocity){
        gpioHardwarePWM(18, 1000, static_cast<unsigned int>(round(kMaxDuty*velocity.linear.x)));
    }
};

void HandleShutdown(std_msgs::Int8 msg){
    ROS_INFO("Node %s is shutting down", ros::this_node::getName().c_str());
    gpioHardwarePWM(18, 0, 0);
    ros::shutdown();
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_driver");
    ROS_INFO("Node %s starts", ros::this_node::getName().c_str());
    ros::NodeHandle n;

    //signal(SIGINT, SigintHandler);
    try {
        MotionDriver driver;
        Subscriber sub = n.subscribe("/teleop/cmd_vel", 4, &MotionDriver::SetVelocity, &driver);
        Subscriber sub_shutdown = n.subscribe("/joystick_server/shutdown", 1, &HandleShutdown);
        ros::spin();
        return 0;
    } catch(...){
        ROS_ERROR("Can not create Motion Driver");
        return -1;
    }

}
