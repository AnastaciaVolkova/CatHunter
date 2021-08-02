/*
UDP server receives user commands.
The node publishes user commands.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include <iostream>
#include <cstring>

using std::cerr;
using std::cout;
using std::endl;


//! UDP server class.
class Client{
private:
    int socket_id_;
    sockaddr_in serv_addr_;
    const int kPort;
    const int kMaxLen;
public:
    Client():kPort(8080), kMaxLen(1){
        socket_id_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_id_ < 0){
            ROS_ERROR("Socket creation fails");
            throw std::runtime_error("Socket creation failes");
        }

        memset(&serv_addr_, 0, sizeof(serv_addr_));
        serv_addr_.sin_family = AF_INET;
        serv_addr_.sin_addr.s_addr = INADDR_ANY;
        serv_addr_.sin_port = htons(kPort);

        if (bind(socket_id_, (const struct sockaddr*)&serv_addr_,sizeof(serv_addr_))<0){
            ROS_ERROR("Socket binding to address fails");
            throw std::runtime_error("Socket binding to address fails");
        }
    }

    //! Receive and return commands from user.
    std_msgs::Int8 Receive(){
        int n;
        std_msgs::Int8 buffer;
        n = recvfrom(socket_id_, &buffer.data, kMaxLen, MSG_WAITALL, NULL, NULL);
        if (n != 1){
            ROS_ERROR("Received more than 1 byte");
            throw std::runtime_error("Received more than 1 byte");
        }
        return buffer;
    }
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "joystick_server");
    ros::NodeHandle n;
    ROS_INFO("Node %s starts", ros::this_node::getName().c_str());
    ros::Publisher pub = n.advertise<std_msgs::Int8>("/joystick_server/teleop", 4);
    try{
        Client server;
        while(ros::ok()){
            try {
                std_msgs::Int8 buffer = server.Receive();
                pub.publish(buffer);
            } catch(...){
                ROS_ERROR("Command reception fails");
                return -1;
            }
        }
    } catch (...){
        ROS_ERROR("Can not create UDP server");
        return -1;
    }
    return 0;
}
