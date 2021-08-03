#include <stdio.h>
#include <pigpio.h>
#include "ros/ros.h"
#include <signal.h>
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_driver");
    ros::NodeHandle n;

    //signal(SIGINT, SigintHandler);
    try {
        MotionDriver driver;
        Subscriber sub = n.subscribe("/velocity_controller/cmd_vel", 4, &MotionDriver::SetVelocity, &driver);
        ROS_INFO("starts");
        ros::spin();
        return 0;
    } catch(...){
        ROS_ERROR("Can not create Motion Driver");
        return -1;
    }

}
