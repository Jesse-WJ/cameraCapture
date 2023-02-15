/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:43:52
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:14:38
 */
#ifndef FRAME_H_
#define FRAME_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>
#include "locker.h"


class frame
{
public:
    frame();
    frame(cv::Mat img,int i,long long time);
    ~frame();

    frame(const frame& a);
    frame& operator=(const frame& a);

    bool empty();
    
    cv::Mat imgData;
    int cameraID;
    long long timeStamp;
};



#endif