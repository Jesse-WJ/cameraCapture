/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-07 19:42:59
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-15 17:53:48
 */
#include "motion.h"
#include <opencv2/opencv.hpp>
#include <vector>

motion::motion()
{
    index =0;
    isFinished = false;
}

motion::~motion()
{

}

cv::Point motion::getmoveDot(int actionID)
{
    switch(actionID)
    {
        case 1:
            return action1();
        case 2:
            return action2();
        default:
            return {-1,-1};
    }
    
}

bool motion::isFinish()
{
    return isFinished;
}

void motion::clear()
{
    index=0;
    isFinished=false;
}

cv::Point motion::action1()
{
    int x,y;
    if(index<=17)
    {
        x = (index)*100+100;
        y = 100;
    }
    else if(index<=34)
    {
        x = 1800 - (index-17)*100;
        y = 100 + (index-17)*50;
    }
    else if(index<=51)
    {
        x = (index-34)*100+100;
        y = 100 + 17*50;
    }
    else
    {
        isFinished = true;
    }
    index++;
    return {x,y};
}

cv::Point motion::action2()
{
    static std::vector<cv::Point> calibPoint={{0,0},{500,300},{950,300},{1450,300},
                                {500,550},{950,550},{1450,550},
                                {500,800},{950,800},{1450,800}};
    calibPointID = index++/10;
    if(calibPointID<calibPoint.size())
    {
        return calibPoint[calibPointID];
    }
    else
    {
        isFinished = true;
        return {-1,-1};
    }
}

int motion::getcalibPointID()
{
    return calibPointID;
}