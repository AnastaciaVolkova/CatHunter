#include <stdio.h>
#include <pigpio.h>
#include "ros/ros.h"
#include <signal.h>
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using geometry_msgs::Twist;
using ros::Subscriber;
using std::vector;

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
    static constexpr int kWheelsNum = 4;
    int i2c_id_;
    void GetLedRegValues(int duty, char& reg_l, char& reg_h){
        reg_l = static_cast<char>(duty & 0xff);
        reg_h = static_cast<char>((duty >> 8) & 0xff);
    }

    void ResetRegisters(){
        // Reset control registers.
        for (int reg = 0x6; reg <= 0x45; reg++){
            i2cWriteByteData(i2c_id_, reg, 0);
        }
    }

    // Set Led control register sequence (on_low, on_high, off_low, off_high)
    void SetLedRegister(int channel, char on_low, char on_high, char off_low, char off_high){
        char buffer[]{on_low, on_high, off_low, off_high};
        int reg = 6 + (channel<< 2);
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
    }

    void SetLedRegister(int channel, const char (&buffer)[4]){
        SetLedRegister(channel, buffer[0], buffer[1], buffer[2], buffer[3]);
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
        vector<int> channels={8, 10, 12, 14};// Forward left wheel, Forward right wheel, Back left wheel, Back right wheel
        vector<float> ch_vel(kWheelsNum, abs(velocity.linear.x));

        char buffer[4] = {0};
        for (auto& c: channels){
            SetLedRegister(c, buffer);
            SetLedRegister(c+1, buffer);
        }

        // Decrease side speed in case of angular speed.
        if (velocity.angular.z < 0){ // Rotate left, decrease right side speed.
            ch_vel[1] -= velocity.angular.z;
            ch_vel[3] -= velocity.angular.z;
        }else if (velocity.angular.z > 0){ // Rotate right, decrease left side speed.
            ch_vel[0] -= velocity.angular.z;
            ch_vel[2] -= velocity.angular.z;
        }

        for (int i = 0; i < channels.size(); i++){
            if (ch_vel[i] < 0){
                ch_vel[i] = -ch_vel[i];
                channels[i]++;
            }
        }

        buffer[0] = 0;   // ON_L
        buffer[1] = 0;   // ON_H
        for (int i = 0; i < kWheelsNum; i++){
            GetLedRegValues(static_cast<int>(round(kMaxDuty*ch_vel[i])), buffer[2], buffer[3]);
            SetLedRegister(channels[i], buffer[0], buffer[1], buffer[2], buffer[3]);
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
