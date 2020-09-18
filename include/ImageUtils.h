#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>

class ImageUtils
{
public:
    bool loadData(std::string filename);
    bool CheckBlueDoor(cv::Mat srcImage,cv::Rect &door);
    bool CheckYellowDoor(cv::Mat srcImage,cv::Rect &door);
    bool CheckBlackDog(cv::Mat srcImage,std::vector<cv::Rect> &dogs);
    bool judgeDog(cv::Mat srcImage,cv::Rect dog,std::string team);
    bool CheckRedBall(cv::Mat srcImage,cv::Rect &ball);

private:
    int blue_door_hmin = 80;//色相
    int blue_door_hmax = 114;
    int blue_door_smin = 44;  //饱和度
    int blue_door_smax = 255;
    int blue_door_vmin = 0;    //亮度
    int blue_door_vmax = 255;
    int blue_door_MORPH_OPEN_ksize = 7;//开运算核大小
    int blue_door_area_size = 500;//轮廓面积


    int yellow_door_hmin = 0;//色相
    int yellow_door_hmax = 25;
    int yellow_door_smin = 30;  //饱和度
    int yellow_door_smax = 255;
    int yellow_door_vmin = 82;    //亮度
    int yellow_door_vmax = 255;
    int yellow_door_MORPH_OPEN_ksize = 7;//开运算核大小
    int yellow_door_area_size = 500;//轮廓面积


    int black_dog_BINARY_INV_threashold = 57;
    int black_dog_erode_ksize = 3;
    int black_dog_dilate_ksize = 9;
    int black_dog_area_size = 1024;
    int blue_leg_hmin = 90;//色相
    int blue_leg_hmax = 109;
    int blue_leg_smin = 53;  //饱和度
    int blue_leg_smax = 255;
    int blue_leg_vmin = 100;    //亮度
    int blue_leg_vmax = 255;
    int blue_leg_medianblur_ksize = 5;//开运算核大小
    int blue_leg_area_size = 100;//轮廓面积
    int yellow_leg_hmin = 10;//色相
    int yellow_leg_hmax = 28;
    int yellow_leg_smin = 64;  //饱和度
    int yellow_leg_smax = 255;
    int yellow_leg_vmin = 98;    //亮度
    int yellow_leg_vmax = 255;
    int yellow_leg_medianblur_ksize = 3;//开运算核大小
    int yellow_leg_area_size = 100;//轮廓面积


    int red_ball_BlueChannel = 10;//蓝色通道增强系数
    int red_ball_GreenChannel = 10;//绿色通道增强系数
    int red_ball_RedChannel = 14;//红色通道增强系数
    int red_ball_BINARY_threashold = 100;//二值化阈值
    int red_ball_erode_ksize = 3;//腐蚀核大小
    int red_ball_dilate_ksize = 3;//膨胀核大小


    int background_bChannel = 3; // B通道增强系数
    int background_rChannel = 6; // R通道增强系数
    int background_gChannel = 6; // G通道增强系数
    int background_threshold = 120; // 二值化阈值
    int background_erode_ksize = 3; // 腐蚀核大小
    int background_dilate_ksize = 3; // 膨胀核大小
    int background_medianblur_ksize = 25; // 中值滤波核大小
};

#endif
