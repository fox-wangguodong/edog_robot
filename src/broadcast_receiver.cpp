/*

本节点用来作为广播接收节点

*/

#include "ros/ros.h"
#include "broadcast_types.h"
#include "edog_robot/command.h"
#include "edog_robot/status.h"
#include "command_builder.h"

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
#include <string>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "broadcast_receiver");


    ros::NodeHandle n;
    ros::Publisher status_publisher = n.advertise<edog_robot::status>("status_control", 10);//发布状态变更消息


    int robot_number;
    std::string robot_team;
	ros::param::get("robot_number", robot_number);
	ros::param::get("robot_team", robot_team);


    int sockListen = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockListen == -1)
    {
        printf("socket fail\n");
        return -1;
    }

    int set = 1;
    setsockopt(sockListen, SOL_SOCKET, SO_REUSEADDR, &set, sizeof(int));

    struct sockaddr_in recvAddr;
    memset(&recvAddr, 0, sizeof(struct sockaddr_in));
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(8001);
    recvAddr.sin_addr.s_addr = INADDR_ANY;

    // 必须绑定，否则无法监听
    if(bind(sockListen, (struct sockaddr*)&recvAddr, sizeof(struct sockaddr)) == -1)
    {
        printf("bind fail\n");
        return -1;
    }

    int addrLen = sizeof(struct sockaddr_in);
    while (ros::ok())
    {
        int type;
        int recvbytes = recvfrom(sockListen, (char*)&type, 4, 0,(struct sockaddr*)&recvAddr, (socklen_t*)&addrLen); // 接收包长度字段
        switch(type)
        {
            case StartSignal_TYPE:
            {
                //比赛开始
                ROS_INFO("StartSignal_TYPE\n");

                edog_robot::status status_msg;
                status_msg.status = (unsigned int)type;
                status_publisher.publish(status_msg);//发布消息
            }
            break;
            case StopSignal_TYPE:
            {
                //比赛结束
                ROS_INFO("StopSignal_TYPE\n");

                edog_robot::status status_msg;
                status_msg.status = (unsigned int)type;
                status_publisher.publish(status_msg);//发布消息
            }
            break;
            case OutSignal_TYPE:
            {
                RobotInformation robotInfo;
                recvbytes = recvfrom(sockListen, (char*)&robotInfo, sizeof(RobotInformation), 0,(struct sockaddr*)&recvAddr, (socklen_t*)&addrLen); // 接收包长度字段
                
                ROS_INFO("OutSignal_TYPE ````%d\t%s\n",robotInfo.number,robotInfo.robotTeam);

                //判断自己是否应该出局,若是自己则停止运动，如果不是自己则继续比赛
                if(robotInfo.number == robot_number && strcmp(robot_team.c_str(),robotInfo.robotTeam) == 0)
                {
                    edog_robot::status status_msg;
                    status_msg.status = (unsigned int)type;
                    status_publisher.publish(status_msg);//发布消息

                    //发送罚下场地信号给自己
                    ROS_INFO("被罚出场地\n");
                }

            }
            break;
            case InSignal_TYPE:
            {
                RobotInformation robotInfo;
                recvbytes = recvfrom(sockListen, (char*)&robotInfo, sizeof(RobotInformation), 0,(struct sockaddr*)&recvAddr, (socklen_t*)&addrLen); // 接收包长度字段
                
                ROS_INFO("InSignal_TYPE ````%d\t%s\n",robotInfo.number,robotInfo.robotTeam);

                //判断自己是否应该重新进入比赛,若是自己则继续比赛1
                if(robotInfo.number == robot_number && strcmp(robot_team.c_str(),robotInfo.robotTeam) == 0)
                {
                    edog_robot::status status_msg;
                    status_msg.status = (unsigned int)type;
                    status_publisher.publish(status_msg);//发布消息

                    //发送重新入场信号给自己
                    ROS_INFO("重新进入场地\n");
                }
            }
            break;
        }
    }
    close(sockListen);
    return 0;
}
