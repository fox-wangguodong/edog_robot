/*

目标识别算法封装类，从文件中读取配置参数

*/


#include "ImageUtils.h"


bool ImageUtils::loadData(std::string filename)
{
    std::fstream file(filename,std::ios::in);
    if(!file.is_open()){
        return false;
    }

    file >> blue_door_hmin ;
    file >> blue_door_hmax ;
    file >> blue_door_smin ;
    file >> blue_door_smax ;
    file >> blue_door_vmin ;
    file >> blue_door_vmax ;
    file >> blue_door_MORPH_OPEN_ksize ;
    file >> blue_door_area_size ;

    file >> yellow_door_hmin ;
    file >> yellow_door_hmax ;
    file >> yellow_door_smin ;
    file >> yellow_door_smax ;
    file >> yellow_door_vmin ;
    file >> yellow_door_vmax ;
    file >> yellow_door_MORPH_OPEN_ksize ;
    file >> yellow_door_area_size ;

    file >> black_dog_BINARY_INV_threashold ;
    file >> black_dog_erode_ksize ;
    file >> black_dog_dilate_ksize ;
    file >> black_dog_area_size ;

    file >> blue_leg_hmin;
    file >> blue_leg_hmax;
    file >> blue_leg_smin;
    file >> blue_leg_smax;
    file >> blue_leg_vmin;
    file >> blue_leg_vmax;
    file >> blue_leg_medianblur_ksize;
    file >> blue_leg_area_size;
    file >> yellow_leg_hmin;
    file >> yellow_leg_hmax;
    file >> yellow_leg_smin;
    file >> yellow_leg_smax;
    file >> yellow_leg_vmin;
    file >> yellow_leg_vmax;
    file >> yellow_leg_medianblur_ksize;
    file >> yellow_leg_area_size;

    file >> red_ball_BlueChannel ;
    file >> red_ball_GreenChannel ;
    file >> red_ball_RedChannel ;
    file >> red_ball_BINARY_threashold ;
    file >> red_ball_erode_ksize ;
    file >> red_ball_dilate_ksize ;

    file >> background_bChannel ;
    file >> background_gChannel ;
    file >> background_rChannel ;
    file >> background_threshold ;
    file >> background_erode_ksize ;
    file >> background_dilate_ksize ;
    file >> background_medianblur_ksize ;

    file.close();
    return true;
}

bool ImageUtils::CheckBlueDoor(cv::Mat srcImage,cv::Rect &door)
{
    //转HSV图像
    cv::Mat imgHSV;
    cv::cvtColor(srcImage, imgHSV, cv::COLOR_BGR2HSV);

    //提取指定颜色区域到Mask中
    cv::Mat mask_blue;
    cv::inRange(imgHSV,
                cv::Scalar(blue_door_hmin, blue_door_smin, blue_door_vmin),
                cv::Scalar(blue_door_hmax, blue_door_smax, blue_door_vmax),
                mask_blue);

    // 形态学变换,开运算,去掉白点
    cv::morphologyEx(mask_blue,
                        mask_blue,
                        cv::MORPH_OPEN,
                        cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(blue_door_MORPH_OPEN_ksize,blue_door_MORPH_OPEN_ksize)));
    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_blue,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);

    if(contours.size() == 0)
    {
        return false;
    }
    else
    {
        //查找最大轮廓
        int maxArea_index = 0;
        for (size_t index = 0; index < contours.size(); index++)
        {
            if (cv::contourArea(contours[index]) > cv::contourArea(contours[maxArea_index]))
            {
                maxArea_index = index; // 取最大轮廓索引号
            }
        }

        std::vector<cv::Point> roi_point_approx;
        cv::approxPolyDP(contours[maxArea_index],roi_point_approx,15,1);
        if(roi_point_approx.size() >= 4
                && roi_point_approx.size() <= 8
                && cv::contourArea(contours[maxArea_index]) > blue_door_area_size)
        {
            door = cv::boundingRect(contours[maxArea_index]);
            return true;
        }
        else
        {
            return false;
        }
    }
}


bool ImageUtils::CheckYellowDoor(cv::Mat srcImage,cv::Rect &door)
{
    //转HSV图像
    cv::Mat imgHSV;
    cv::cvtColor(srcImage, imgHSV, cv::COLOR_BGR2HSV);

    //提取颜色区域
    cv::Mat mask_yellow;
    cv::inRange(imgHSV,
                cv::Scalar(yellow_door_hmin, yellow_door_smin, yellow_door_vmin),
                cv::Scalar(yellow_door_hmax, yellow_door_smax, yellow_door_vmax),
                mask_yellow);

    //形态学变换,开运算,去掉白点
    cv::morphologyEx(mask_yellow,
                        mask_yellow,
                        cv::MORPH_OPEN,
                        cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(yellow_door_MORPH_OPEN_ksize,yellow_door_MORPH_OPEN_ksize)));

    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_yellow,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);

    if(contours.size() == 0)
    {
        return false;
    }
    else
    {
        //查找最大轮廓
        int maxArea_index = 0;
        for (size_t index = 0; index < contours.size(); index++)
        {
            if (cv::contourArea(contours[index]) > cv::contourArea(contours[maxArea_index]))
            {
                maxArea_index = index;
            }
        }

        std::vector<cv::Point> roi_point_approx;
        cv::approxPolyDP(contours[maxArea_index],roi_point_approx,15,1);
        if(roi_point_approx.size() >= 4
                && roi_point_approx.size() <= 8
                && cv::contourArea(contours[maxArea_index]) > yellow_door_area_size)
        {
            door = cv::boundingRect(contours[maxArea_index]);
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool ImageUtils::CheckBlackDog(cv::Mat srcImage,std::vector<cv::Rect> &dogs)
{
    //提取红色通道
    cv::Mat channel[3];
    cv::split(srcImage,channel);
    cv::Mat grayImage = channel[2];

    //反二值化,提取黑色部分
    cv::threshold(channel[2],channel[2],black_dog_BINARY_INV_threashold,255,cv::THRESH_BINARY_INV);

    //腐蚀
    cv::morphologyEx(grayImage,
                    grayImage,
                    cv::MORPH_ERODE,
                    cv::getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(black_dog_erode_ksize,black_dog_erode_ksize)));
    //膨胀
    cv::morphologyEx(grayImage,
                    grayImage,
                    cv::MORPH_DILATE,
                    cv::getStructuringElement(cv::MORPH_RECT,
                                            cv::Size(black_dog_dilate_ksize,black_dog_dilate_ksize)));
    //提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(grayImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);


    if(contours.size() == 0)
    {
        return false;
    }
    else
    {
        for(size_t i=0; i<contours.size(); i++)
        {
            if(cv::contourArea(contours[i]) > black_dog_area_size)
            {
                cv::Rect dog_rect = cv::boundingRect(contours[i]);
                dogs.push_back(dog_rect);
            }
        }
        if(dogs.size() > 0){
            return true;
        } else {
            return false;
        }
    }
}


bool ImageUtils::judgeDog(cv::Mat srcImage,cv::Rect dog_rect,std::string team)
{
    //提取狗的区域
    cv::Mat dogImage = srcImage(dog_rect);
    //转HSV图像
    cv::Mat imgHSV;
    cv::cvtColor(dogImage, imgHSV, cv::COLOR_BGR2HSV);

    // 黄色区域检测
    do {
        //提取颜色区域
        cv::Mat mask_yellow;
        cv::inRange(imgHSV,
                    cv::Scalar(yellow_leg_hmin, yellow_leg_smin, yellow_leg_vmin),
                    cv::Scalar(yellow_leg_hmax, yellow_leg_smax, yellow_leg_vmax),
                    mask_yellow);
        //中值滤波去噪点
        cv::medianBlur(mask_yellow,mask_yellow,yellow_leg_medianblur_ksize);

        //提取轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask_yellow,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);

        if(contours.size() == 0)
        {
            break;
        }
        else
        {
            //查找最大轮廓
            int maxArea_index = 0;
            for (size_t index = 0; index < contours.size(); index++)
            {
                if (cv::contourArea(contours[index]) > cv::contourArea(contours[maxArea_index]))
                {
                    maxArea_index = index;
                }
            }
            if(cv::contourArea(contours[maxArea_index]) > yellow_leg_area_size && team == "yellow")
            {
                return true;
            }
        }
    }while(true);


    // 蓝色区域检测
    do {
        //提取颜色区域
        cv::Mat mask_blue;
        cv::inRange(imgHSV,
                    cv::Scalar(blue_leg_hmin, blue_leg_smin, blue_leg_vmin),
                    cv::Scalar(blue_leg_hmax, blue_leg_smax, blue_leg_vmax),
                    mask_blue);
        cv::medianBlur(mask_blue,mask_blue,blue_leg_medianblur_ksize);

        //提取轮廓
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask_blue,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_NONE);

        if(contours.size() == 0)
        {
            break;
        }
        else
        {
            //查找最大轮廓
            int maxArea_index = 0;
            for (size_t index = 0; index < contours.size(); index++)
            {
                if (cv::contourArea(contours[index]) > cv::contourArea(contours[maxArea_index]))
                {
                    maxArea_index = index;
                }
            }
            if(cv::contourArea(contours[maxArea_index]) > blue_leg_area_size && team == "blue")
            {
                return true;
            }
        }
    }while(true);
    return false;
}


bool ImageUtils::CheckRedBall(cv::Mat srcImage,cv::Rect &ball)
{
    // 提取红色通道图像
    cv::Mat channel[3];
    cv::split(srcImage, channel);
    channel[0] = channel[0].mul(0.1*red_ball_BlueChannel); // B
    channel[1] = channel[1].mul(0.1*red_ball_GreenChannel); // G
    channel[2] = ( channel[2] - channel[0] - channel[1] ) * red_ball_RedChannel; // R
    cv::Mat mask_ball = channel[2];

    // 二值化
    cv::threshold(mask_ball,mask_ball,red_ball_BINARY_threashold,255,cv::THRESH_BINARY);

    // 腐蚀
    cv::morphologyEx(mask_ball,
                        mask_ball,
                        cv::MORPH_ERODE,
                        cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(red_ball_erode_ksize,red_ball_erode_ksize)));
    // 膨胀
    cv::morphologyEx(mask_ball,
                        mask_ball,
                        cv::MORPH_DILATE,
                        cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(red_ball_dilate_ksize,red_ball_dilate_ksize)));
    // 提取轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_ball, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    if(contours.size() == 0)
    {
        return false;
    }
    else
    {
        // 查找最大轮廓
        int maxArea_index = 0;
        for (size_t index = 0; index < contours.size(); index++)
        {
            if (cv::contourArea(contours[index]) > cv::contourArea(contours[maxArea_index]))
            {
                maxArea_index = index;
            }
        }
        if(cv::contourArea(contours[maxArea_index]) > 50){
            ball = cv::boundingRect(contours[maxArea_index]);
            return true;
        }else{
            return false;
        }
    }
}

