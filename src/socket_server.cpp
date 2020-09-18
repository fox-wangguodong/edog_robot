/*

本节点用来作为一对一TCP通信,作为服务端

*/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "std_msgs/String.h"

using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "server_node"); //节点初始化
  ros::NodeHandle nh;  //创建节点句柄
  ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages/", 1000);
  //Testing package is working fine , 创建publisher, 发布名为/server_messages/ 的topic

  int sockfd, newsockfd, portno; //Socket文件描述符, 端口号
  socklen_t clilen; //object clilen of type socklen_t
  char buffer[256]; //buffer array of size 256
  

  struct sockaddr_in serv_addr, cli_addr; ///设置socket地址结构 serv_addr, cli_addr
  std_msgs::String message;  //初始化std_msgs::String类型的消息
  std::stringstream ss;
  int n;

  ros::Duration d(0.01); // 100Hz

  portno = 1024;
  cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
      error("ERROR opening socket");
  int enable = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
      error("setsockopt(SO_REUSEADDR) failed");

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0)
            error("ERROR on binding");

  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd,
              (struct sockaddr *) &cli_addr,
              &clilen);

  if (newsockfd < 0)
       error("ERROR on accept");

  while(ros::ok()) {
     ss.str(std::string()); //清除string流
     bzero(buffer,256);
     n = read(newsockfd,buffer,255); //读取socket缓存长度

     if (n < 0) error("ERROR reading from socket");
     // printf("Here is the message: %s\n",buffer);

     ss << buffer;
     message.data = ss.str();

     //发布消息
     ROS_INFO("%s", message.data.c_str());
     server_pub.publish(message);

     n = write(newsockfd,"I got your message",18);
     if (n < 0) error("ERROR writing to socket");
     //close(newsockfd);
     //close(sockfd);
     //ros::spinOnce();
     //d.sleep();
  }
  return 0;
}
