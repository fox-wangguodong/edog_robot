#ifndef SERIALPORT_H
#define SERIALPORT_H


/**
 * @brief openSerialPort  打开串口设备
 * @param dev 串口设备文件(/dev/ttyUSB0)
 * @return 返回串口设备fd
 */
int openSerialPort(const char *dev);



/**
 * @brief closeSerialPort  关闭串口设备
 * @param fd   串口设备fd
 */
void closeSerialPort(int fd);


/**
 * @brief SendData  发送数据包
 * @param fd    串口设备
 * @param data   待发送的数据
 * @param length   数据长度
 * @return  0：成功   -1：失败
 */
int SendData(int fd,char* data,unsigned int length);



/**
 * @brief RecvData   接收数据包
 * @param fd        串口设备
 * @param data  待接收的数据缓冲区
 * @param length  待接收的数据长度
 * @return  0:成功   -1：失败
 */
int RecvData(int fd,char* data,unsigned int length);


#endif
