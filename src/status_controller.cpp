/*

本节点用来接收APP端的状态数据,然后发送给决策节点
2019年07月24日 16:44

*/


#include "ros/ros.h"
#include "edog_robot/command.h"
#include "edog_robot/status.h"
#include "command_builder.h"

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <malloc.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <linux/ip.h>
#include <errno.h>

int getIP(std::string net_name,std::string &strIP)
{
#define BUF_SIZE 1024

   int sock_fd;
   struct ifconf conf;
   struct ifreq *ifr;
   char buff[BUF_SIZE] = {0};
   int num;
   int i;

   sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
   if ( sock_fd < 0 )
       return -1;

   conf.ifc_len = BUF_SIZE;
   conf.ifc_buf = buff;

   if ( ioctl(sock_fd, SIOCGIFCONF, &conf) < 0 )
   {
       close(sock_fd);
       return -1;
   }

   num = conf.ifc_len / sizeof(struct ifreq);
   ifr = conf.ifc_req;

   for(i = 0; i < num; i++)
   {
       struct sockaddr_in *sin = (struct sockaddr_in *)(&ifr->ifr_addr);

       if ( ioctl(sock_fd, SIOCGIFFLAGS, ifr) < 0 )
       {
               close(sock_fd);
               return -1;
       }

       if ( (ifr->ifr_flags & IFF_UP) && strcmp(net_name.c_str(),ifr->ifr_name) == 0 )
       {
               strIP = inet_ntoa(sin->sin_addr);
               close(sock_fd);

               return 0;
       }

       ifr++;
   }
   close(sock_fd);
   return -1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "status_controller");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<edog_robot::status>("status_control", 10);


    std::string ip_address;
    getIP("wlp15s0",ip_address); // 获取主机IP地址
    if(ip_address.size() == 0){
        ROS_INFO("wlan0 getIP failed!");
        return 0;
    }

    //1、创建socket
    int server_sockfd = socket(AF_INET,SOCK_STREAM,0);
    //2、创建服务器地址及协议
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(30012);
    //server_address.sin_addr.s_addr = inet_addr(ip_address.c_str());
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);//监听主机任意IP
    //3、绑定socket到指定地址
    bind(server_sockfd,(struct sockaddr*)&server_address,sizeof(server_address));
    //4、监听socket,队列长度为1
    listen(server_sockfd,1);

    ROS_INFO("wait for client...");

    while(ros::ok())
    {
        //5、accept接受客户端链接
        socklen_t address_len;//保存客户端地址长度
        struct sockaddr_in client_address;//保存客户端地址族
        int client_sockfd = accept(server_sockfd,(struct sockaddr*)&client_address,&address_len);//接收客户端链接并保存socket_fd

        ROS_INFO("client connected !");
        struct timeval timeout = {4,0};
        setsockopt(client_sockfd,SOL_SOCKET,SO_RCVTIMEO,(char*)&timeout,sizeof(struct timeval));//设置读取超时时间

        while(ros::ok())
        {

            char buffer[4];
            int len = recv(client_sockfd,buffer,sizeof(buffer),0);//接收数据
            if(len > 0) // 正常读取
            {
                int status = buffer[0]<<24 | buffer[1]<<16 | buffer[2]<<8 | buffer[3];

                if(status != -1) // status为-1则表示为心跳帧,若收到status不是-1则回传正常指令
                {
                    ROS_INFO("recv status:%d",status);

                    edog_robot::status status_msg;
                    status_msg.status = (unsigned int)status;
                    //发布消息
                    chatter_pub.publish(status_msg);
                    ros::spinOnce();//若通过回调函数订阅了消息，调用该语句会处理回调函数，否则不会调用
                    send(client_sockfd,"OK",2,0);//发送回执
                }
            }
            else // 若len<=0,表示客户端是否断开了链接
            {
                close(client_sockfd);
                ROS_INFO("client disconnected !");
                break;
            }
        }
    }
    //6、关闭socket
    close(server_sockfd);
    return 0;
}
