#include <stdio.h>
#include <pigpio.h>
#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iomanip>
#include <iostream>

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
    static constexpr float kMaxDuty = (1<<12)-1;
    int i2c_id_;
    void GetLedRegValues(int duty, char& reg_l, char& reg_h){
        reg_l = static_cast<char>(duty & 0xff);
        reg_h = static_cast<char>((duty >> 8) & 0xff);
        ROS_INFO("%d %x %x", duty, reg_h, reg_l);
    }

    void ResetRegisters(){
        // Reset control registers.
        for (int reg = 0x6; reg < 0x45; reg++){
            i2cWriteByteData(i2c_id_, reg, 0);
        }
    }
public:
    MotionDriver(){
        if (gpioInitialise() < 0)
        {
            ROS_ERROR("Unable to initialize gpio");
            throw std::runtime_error("Unable to initialize gpio library");
        }
        i2c_id_ = i2cOpen(1, 0x40, 0);
        if (i2c_id_<0){
            ROS_ERROR("Unable to open i2c");
            throw std::runtime_error("Unable to open i2c");
        }
        ResetRegisters();
    };

    void SetVelocity(const Twist& velocity){
        char buffer[4];
        buffer[0] = 0;   // ON_L
        buffer[1] = 0;   // ON_H
        GetLedRegValues(static_cast<int>(round(kMaxDuty*velocity.linear.x)), buffer[2], buffer[3]);
        int channel = 12;
        int reg = 6 + (12 << 2);
        for (int i = 0; i < 4; i++){
            int stat = i2cWriteByteData(i2c_id_, reg, buffer[i]);
            if (stat < 0)
                switch(stat){
                    case PI_BAD_HANDLE:
                            ROS_ERROR("pi bad handle"); break;
                    case PI_BAD_PARAM:
                            ROS_ERROR("pi bad param"); break;
                    case PI_I2C_WRITE_FAILED:
                            ROS_ERROR("pi i2c write fails"); break;
                    default:
                            ROS_ERROR("Fail to write to i2c");
            }
            reg++;
        }
    };

    ~MotionDriver(){
        ResetRegisters();
        i2cClose(i2c_id_);
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
