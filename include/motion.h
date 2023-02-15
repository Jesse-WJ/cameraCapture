/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-07 19:43:09
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:12:31
 */
#ifndef MOTION_H_
#define MOTION_H_

#include <opencv2/opencv.hpp>


class motion{

public:
    motion();
    ~motion();

    cv::Point getmoveDot(int actionID);
    cv::Point action1();
    cv::Point action2();
    bool isFinish();
    void clear();
    int getcalibPointID();
private:
    int index;
    bool isFinished;
    int calibPointID;
};
#endif