#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"

using std_msgs::Float64;
using std_msgs::Int8;
using geometry_msgs::Twist;
using ros::NodeHandle;
using ros::Publisher;
using ros::Subscriber;

// kUp - increase linear speed
// kDown - decrease linear speed
// kLeft - increase angular speed
// kRight - decrease angular speed
// kQ - zero linear speed
// kA - zero angular speed
enum class Command : int8_t{kEscape = -1, kUp, kDown, kLeft, kRight, kQ, kA};

class VelocityController{
private:
    Twist twist_;
    Twist d_twist_;
    Publisher pub_;
public:
    VelocityController(float linear, float angular, float d_linear, float d_angular, Publisher pub){
        d_twist_.linear.x = linear;
        twist_.angular.z = angular;
        d_twist_.linear.x = d_linear;
        d_twist_.angular.z = d_angular;
        pub_ = pub;
    };

    void JoystickCallback(std_msgs::Int8 msg){
        Command cmd = static_cast<Command>(msg.data);
        switch (cmd){
            case Command::kUp:
                if (twist_.linear.x <= 1 - d_twist_.linear.x)
                    twist_.linear.x += d_twist_.linear.x;
            break;
            case Command::kDown:
                if (twist_.linear.x >= -1 + d_twist_.linear.x)
                    twist_.linear.x -= d_twist_.linear.x;
            break;
            case Command::kLeft:
                if (twist_.angular.z <= 1 - d_twist_.angular.z)
                    twist_.angular.z += d_twist_.angular.z;
            break;
            case Command::kRight:
                if (twist_.angular.z >= -1 + d_twist_.angular.z)
                    twist_.angular.z -= d_twist_.angular.z;
            break;
            case Command::kQ: {
                twist_.linear.x = 0;
            }
            break;
            case Command::kA: {
                twist_.angular.z = 0;
            }
            break;
            default:
                ROS_ERROR("Unknown error");
                break;
        }
        pub_.publish(twist_);
    };
};

void HandleShutdown(Int8 msg){
    ROS_INFO("Node %s is shutting down", ros::this_node::getName().c_str());
    ros::shutdown();
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle n;
    ROS_INFO("Node %s starts", ros::this_node::getName().c_str());
    Publisher pub = n.advertise<Twist>("/velocity_controller/cmd_vel", 4);
    VelocityController vel_ctrl(0.0, 0.0, 1.0/16.0, 1.0/8.0, pub);
    Subscriber sub = n.subscribe("/joystick_server/teleop", 4, &VelocityController::JoystickCallback, &vel_ctrl);
    Subscriber sub_shutdown = n.subscribe("/joystick_server/shutdown", 4, &HandleShutdown);
    ros::spin();
    return 0;
}
