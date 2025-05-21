#include <iostream>
#include <thread>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <chrono>
#include <unistd.h>
#include <ros/ros.h>

#pragma pack(1)

// 此节点无需修改,作用是模拟操纵端向底盘持续发送程控模式指令,以使底盘能够响应上位机发送的速度指令

struct HEADER
{
    uint16_t  mFrameHeader;
    uint8_t mFrameType;
    uint8_t mFrameLength;
};

struct UdpControlCommand
{
    HEADER mHeader;
    uint8_t driveGear;
    uint8_t driveMode;
    uint8_t carMaxSpeed;
    uint8_t armMaxSpeed;
    int8_t chassisControlArmHorizontal;
    int8_t chassisControlArmVertical;
    uint8_t chassisPower;
    int16_t frontArmAngle;
    int16_t behindArmAngle;
    uint8_t dataDestruction;
    uint16_t reserved;
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


int send_port;
int receive_port;
std::string server_ip;
std::string client_ip;



// 发送数据的线程函数
void sendThread(int sockfd,/* int8_t driveGear, int8_t driveMode, int8_t carMaxSpeed, int8_t chassisControlArmHorizontal, 
                                    //int8_t chassisControlArmVertical, uint8_t chassisPower, uint16_t frontArmAngle, uint16_t behindArmAngle, */const sockaddr_in &serverAddr)
{
    // struct sockaddr_in serverAddr;
    // serverAddr.sin_family = AF_INET;
    // serverAddr.sin_port = htons(SEND_PORT);
    // serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    while (true)
    {

        UdpControlCommand dataToSend;
        dataToSend.mHeader.mFrameHeader = 0xffaa;
        dataToSend.mHeader.mFrameType = 0x0a;
        dataToSend.mHeader.mFrameLength = 19;

        dataToSend.driveGear = 0b11;
        dataToSend.driveMode = 0b1;
        dataToSend.carMaxSpeed = 20;
        dataToSend.armMaxSpeed = 0;
        dataToSend.chassisControlArmHorizontal = 0;
        dataToSend.chassisControlArmVertical = 0;
        dataToSend.chassisPower = 0b1010001;
        dataToSend.frontArmAngle = 0;
        dataToSend.behindArmAngle = 0;
        dataToSend.dataDestruction = 0;
        dataToSend.reserved = 0;


        // dataToSend.vel = vel;
        // dataToSend.cur = cur;
        // dataToSend.pitch = pitch;


        uint16_t checkSum = 0;

        memset(&sendBuffer[0], 0, sizeof(sendBuffer));
        memcpy(&sendBuffer[0], &dataToSend, 18);

        for (int i = 0; i < 18; i++)
        {
            checkSum += sendBuffer[i];
        }

        sendBuffer[18] = checkSum & 0xff;

        sendto(sockfd, &sendBuffer[0], 19, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

        // 延迟100毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(1000*1));
    }
}

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "UdpRemote");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("send_port", send_port);
    nhPrivate.getParam("receive_port", receive_port);
    nhPrivate.getParam("server_ip", server_ip);
    nhPrivate.getParam("client_ip", client_ip);

    nhPrivate.getParam("send_port_Remote", send_port);
    nhPrivate.getParam("receive_port_Remote", receive_port);
    nhPrivate.getParam("server_ip_Remote", server_ip);
    nhPrivate.getParam("client_ip_Remote", client_ip);

    const int SEND_PORT = send_port;         // 发送数据的UDP端口
    const int RECEIVE_PORT = receive_port;      // 接收数据的UDP端口   
    const char *SERVER_IP = server_ip.c_str(); // 目标服务器IP地址
    const char *CLIENT_IP = client_ip.c_str(); // 自身IP地址

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

    //绑定接收端口
    

    if (bind(sendSockfd, (struct sockaddr *)&receiveAddr, sizeof(receiveAddr)) < 0)
    {
        perror("Error in binding for sending");
        exit(1);
    }


    int16_t vel = 0x64;
    int16_t cur = 0;
    int16_t pitch = 0;

    // uint8_t driveGear;
    // uint8_t driveMode;
    // uint8_t carMaxSpeed;
    // uint8_t armMaxSpeed;
    // uint8_t chassisControlArmHorizontal;
    // uint8_t chassisControlArmVertical;
    // uint8_t chassisPower;
    // uint16_t frontArmAngle;
    // uint16_t behindArmAngle;

    // std::thread sender(sendThread, sendSockfd, vel, cur, pitch, serverAddr);
    std::thread sender(sendThread, sendSockfd, /*driveGear, driveMode, carMaxSpeed, armMaxSpeed, chassisControlArmHorizontal, chassisControlArmVertical, chassisPower, frontArmAngle, behindArmAngle,*/ serverAddr);

    // std::thread receiver(receiveThread, sendSockfd);

    sender.join();
    // receiver.join();

    close(sendSockfd);
    // close(receiveSockfd);

    return 0;
}
