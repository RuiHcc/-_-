#include <iostream>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <chrono>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#pragma pack(1)

struct HEADER
{
    int16_t mFrameHeader;
    int16_t mFrameType;
    uint8_t mFrameLength;
};

struct UdpControlCommand
{
    HEADER mHeader;
    int16_t vel;
    int16_t cur;
    int16_t pitch;
    uint8_t checkSum;
};

struct UdpRecvCommand
{
    HEADER mHeader;
    int16_t vel;
    int16_t cur;
    int16_t wheelVelLeft;
    int16_t wheelVelRight;
    int16_t reserve;
    uint8_t checkSum;
};

#pragma pack()
uint8_t sendBuffer[32];
uint8_t recvBuffer[32];

int send_port = 8021;
int receive_port = 40021;
std::string server_ip = "192.2.2.21";
std::string client_ip;

geometry_msgs::Twist cmd_vel_cur;

// // 发送数据的线程函数
// void sendThread(int sockfd, int16_t vel, int16_t cur, int16_t pitch, const sockaddr_in &serverAddr)
// {
//     // struct sockaddr_in serverAddr;
//     // serverAddr.sin_family = AF_INET;
//     // serverAddr.sin_port = htons(SEND_PORT);
//     // serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

//     while (true)
//     {

//         UdpControlCommand dataToSend;
//         dataToSend.mHeader.mFrameHeader = 0xffcc;
//         dataToSend.mHeader.mFrameType = 0x11;
//         dataToSend.mHeader.mFrameLength = 16;

//         dataToSend.vel = vel;
//         dataToSend.cur = cur;
//         dataToSend.pitch = pitch;

//         uint16_t checkSum = 0;

//         memset(&sendBuffer[0], 0, sizeof(sendBuffer));
//         memcpy(&sendBuffer[0], &dataToSend, 11);

//         for (int i = 0; i < 11; i++)
//         {
//             checkSum += sendBuffer[i];
//         }

//         sendBuffer[11] = checkSum & 0xff;

//         sendto(sockfd, &sendBuffer[0], 12, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

//         // 延迟100毫秒
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

// // 接收数据的线程函数
// void receiveThread(int sockfd)
// {
//     while (true)
//     {
//         struct sockaddr_in clientAddr;
//         socklen_t addr_len = sizeof(clientAddr);

//         memset(&recvBuffer[0], 0, sizeof(recvBuffer));
//         // 接收数据
//         // std::cout << "/* message */" << std::endl;
//         recvfrom(sockfd, &recvBuffer[0], 16, 0, (struct sockaddr *)&clientAddr, &addr_len);
//         // std::cout << "/* message!!! */" << std::endl;

//         uint16_t checkSum = 0;
//         for (int i = 0; i < 15; i++)
//         {
//             checkSum += recvBuffer[i];
//         }
//         checkSum=checkSum&0xff;
//         std::cout << "/* message */" <<checkSum<< std::endl;
//         if ((int)recvBuffer[15] == (int)(checkSum ))
//         {

//             UdpRecvCommand receivedData;
//             memcpy(&receivedData, &recvBuffer[0], 16);
//             std::cout << "Received Data->vel: " << receivedData.vel << " cur: " << receivedData.wheelVelLeft << std::endl;
//         }
//     }
// }

void SpeedHandle(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel_)
{
    // 接受速度，转化成字符串发送
    cmd_vel_cur = cmd_vel_->twist;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "UdpVehicle");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("send_port", send_port);
    nhPrivate.getParam("receive_port", receive_port);
    nhPrivate.getParam("server_ip", server_ip);
    nhPrivate.getParam("client_ip", client_ip);

    nhPrivate.getParam("send_port_vehicle", send_port);
    nhPrivate.getParam("receive_port_vehicle", receive_port);
    nhPrivate.getParam("server_ip_vehicle", server_ip);
    nhPrivate.getParam("client_ip_vehicle", client_ip);

    const int SEND_PORT = send_port;         // 发送数据的UDP端口
    const int RECEIVE_PORT = receive_port;      // 接收数据的UDP端口   
    const char *SERVER_IP = server_ip.c_str(); // 目标服务器IP地址
    const char *CLIENT_IP = client_ip.c_str(); // 自身IP地址

    ros::Subscriber subSpeed = nh.subscribe("cmd_vel", 5, SpeedHandle);

    int sendSockfd, receiveSockfd;
    struct sockaddr_in serverAddr, receiveAddr,serverAddress1;

    // 创建UDP socket用于发送数据
    sendSockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sendSockfd < 0)
    {
        perror("Error in socket creation for sending");
        exit(1);
    }

    // 创建UDP socket用于接收数据
    receiveSockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (receiveSockfd < 0)
    {
        perror("Error in socket creation for receiving");
        exit(1);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SEND_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);


    receiveAddr.sin_family = AF_INET;
    receiveAddr.sin_port = htons(RECEIVE_PORT);
    receiveAddr.sin_addr.s_addr = inet_addr(CLIENT_IP);
    // receiveAddr.sin_addr.s_addr = INADDR_ANY;

    //绑定接收端口
    if (bind(sendSockfd, (struct sockaddr *)&receiveAddr, sizeof(receiveAddr)) < 0)
    {
        perror("Error in binding for sending");
        exit(1);
    }


    int16_t vel = 0;
    int16_t cur = 0;
    int16_t pitch = 0;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        /*********************************最新速度发送***********************************/ 
        UdpControlCommand dataToSend;
        dataToSend.mHeader.mFrameHeader = 0xffcc;
        dataToSend.mHeader.mFrameType = 0x11;
        dataToSend.mHeader.mFrameLength = 12;
        
        if(cmd_vel_cur.linear.x < 0.05)
        {
            dataToSend.vel = 0.0;
            dataToSend.cur = (int16_t) (cmd_vel_cur.angular.z * 1000.0);
        }
        else
        {
            dataToSend.vel = (int16_t) (cmd_vel_cur.linear.x * 100.0);
            dataToSend.cur = (int16_t) (cmd_vel_cur.angular.z * 1.8 / cmd_vel_cur.linear.x * 1000.0);
        }
        dataToSend.pitch = pitch;

        // int16_t vel_hex;
        // std::memcpy(&vel_hex, &cmd_vel_cur.linear.x, sizeof(int16_t));
        // int16_t cur_hex;
        // std::memcpy(&cur_hex, &cmd_vel_cur.angular.z, sizeof(int16_t));
        // dataToSend.vel = vel_hex;
        // dataToSend.cur = cur_hex;
        // dataToSend.pitch = pitch;

        uint16_t checkSum = 0;

        memset(&sendBuffer[0], 0, sizeof(sendBuffer));
        memcpy(&sendBuffer[0], &dataToSend, 11);

        for (int i = 0; i < 11; i++)
        {
            checkSum += sendBuffer[i];
        }

        sendBuffer[11] = checkSum & 0xff;

        sendto(sendSockfd, &sendBuffer[0], 12, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

                
        /*******************************最新返回指令解析***********************************/ 
        struct sockaddr_in clientAddr;
        socklen_t addr_len = sizeof(clientAddr);

        memset(&recvBuffer[0], 0, sizeof(recvBuffer));
        // 接收数据
        // std::cout << "/* message */" << std::endl;
        recvfrom(sendSockfd, &recvBuffer[0], 16, 0, (struct sockaddr *)&clientAddr, &addr_len);
        // std::cout << "/* message!!! */" << std::endl;

        checkSum = 0;
        for (int i = 0; i < 15; i++)
        {
            checkSum += recvBuffer[i];
        }
        checkSum=checkSum&0xff;
        std::cout << "/* message */" <<checkSum<< std::endl;
        if ((int)recvBuffer[15] == (int)(checkSum ))
        {

            UdpRecvCommand receivedData;
            memcpy(&receivedData, &recvBuffer[0], 16);
            std::cout << "Received Data->vel: " << receivedData.vel << " cur: " << receivedData.wheelVelLeft << std::endl;

        //     UdpRecvCommand receivedData;
        //     memcpy(&receivedData, &recvBuffer[0], 16);
        // tank_sdk::TankFeedback fb_msg;
        // fb_msg.velocity = (int)(receivedData.vel) * 0.1;
        // if (fabs(fb_msg.velocity) < 0.05)
        // {
        //     fb_msg.yaw_velocity = (int)(receivedData.cur);
        // }
        // else
        // {
        //     fb_msg.yaw_velocity = fb_msg.velocity * (int)(receivedData.cur) * 1e-3;
        // }
        // fb_msg.wheel_speed_left = (int)(receivedData.wheelVelLeft);
        // fb_msg.wheel_speed_right = (int)(receivedData.wheelVelRight);
        // feedback_pub_.publish(fb_msg);
        // std::cout << "Received Data->vel: " << receivedData.vel << " cur: " << receivedData.wheelVelLeft << std::endl;
        }

    }
    

    // std::thread sender(sendThread, sendSockfd, vel, cur, pitch, serverAddr);

    // std::thread receiver(receiveThread, sendSockfd);

    // sender.join();
    // receiver.join();

    close(sendSockfd);
    close(receiveSockfd);

    return 0;
}
