/*

本节点用来作为广播发送节点

*/


#include "ros/ros.h"
#include "broadcast_types.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "broadcast_sender");

    /*初始化socket 用来在局域网内发送广播*/
    int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if(socket_fd == -1){
        ROS_ERROR("socket fail\n");
        return -1;
    }
    int optval = 1;//这个值一定要设置，否则可能导致sendto()失败
    setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST | SO_REUSEADDR, &optval, sizeof(int));
    struct sockaddr_in theirAddr;
    memset(&theirAddr, 0, sizeof(struct sockaddr_in));
    theirAddr.sin_family = AF_INET;
    theirAddr.sin_port = htons(8001);//设置广播端口
    struct ifreq ifr;
    bzero(&ifr, sizeof(ifr));
    strcpy(ifr.ifr_name, "wlp15s0");//获取指定网卡的广播地址
    if(ioctl(socket_fd, SIOCGIFBRDADDR, &ifr) < 0)
    theirAddr.sin_addr.s_addr = inet_addr(inet_ntoa(((struct sockaddr_in*)&(ifr.ifr_addr))->sin_addr));



    int robot_number;
    std::string robot_team;
	ros::param::get("robot_number", robot_number);
	ros::param::get("robot_team", robot_team);

    RobotInformation robotInfo;//加载robotinfo信息
    strncpy(robotInfo.robotTeam,robot_team.c_str(),robot_team.size());
    robotInfo.number = robot_number;

    /*系统启动时，发送本机器人的信息*/
    int data_type = RobotInfo;
    sendto(socket_fd, (char*)&data_type, sizeof(int), 0,(struct sockaddr*)&theirAddr, sizeof(struct sockaddr));
    sendto(socket_fd, &robotInfo, sizeof(RobotInformation), 0,(struct sockaddr*)&theirAddr, sizeof(struct sockaddr));
    ROS_INFO("Send Robot Info\n");


    while (ros::ok())
    {
        //定时发送机器人位置信息及小球的坐标信息
        ros::spin();
    }
    close(socket_fd);
    return 0;
}


