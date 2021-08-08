/*
Node sends via UDP to remote machine camera images.
*/

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Int8.h"
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>
#include <iostream>

using namespace cv;
using std::string;
using std::vector;
using std::cout;
using ros::Subscriber;

//! Camera Streamer class
/*
    Reads images from camera, codes them and send to remote machine via UDP
*/
class CameraStreamer {
public:
    int socket_id_;
    sockaddr_in client_addr_;
    VideoCapture cap_;
    const int kPort;
    const string kIp;

public:
    CameraStreamer():kPort(8081), kIp("192.168.50.200"){
        // Create capture for /dev/video0
        cap_.open(0 + CAP_V4L2);
        socket_id_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_id_ < 0){
            ROS_ERROR("Socket creation fails");
            throw std::runtime_error("Socket creation fails");
        }

        memset(&client_addr_, 0, sizeof(client_addr_));
        client_addr_.sin_family = AF_INET;
        client_addr_.sin_port = htons(kPort);
        client_addr_.sin_addr.s_addr = inet_addr(kIp.c_str());
    }

    void Send(){
        Mat frame;

        // Read from camera
        cap_.read(frame);

        vector<uchar> buffer;

        // Code image to jpeg
        imencode(".jpg", frame, buffer, {IMWRITE_JPEG_QUALITY, 30});

        // Send coded image to remote machine via udp
        sendto(socket_id_, buffer.data(), buffer.size()*sizeof(uchar), 0, (const struct sockaddr*)&client_addr_, sizeof(client_addr_));
    }

    ~CameraStreamer(){
        cap_.release();
    }
};

void HandleShutdown(std_msgs::Int8 msg){
    ROS_INFO("Node %s is shutting down", ros::this_node::getName().c_str());
    ros::shutdown();
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "camera_streamer"); 
    ros::NodeHandle node;
    ROS_INFO("Node %s starts", ros::this_node::getName().c_str());
    try {
        CameraStreamer camera_streamer;
        Subscriber sub_shutdown = node.subscribe("/joystick_server/shutdown", 1, &HandleShutdown);
        while(ros::ok()){
            camera_streamer.Send();
            ros::spinOnce();
        }
        return 0;
    } catch (...){
        ROS_ERROR("Can not create CameraStreamer");
        return -1;
    }
}
