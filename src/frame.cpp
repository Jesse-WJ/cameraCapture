/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:43:55
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:22:02
 */
#include "frame.h"
#include "glass.h"
#include <cmath>
#include <algorithm>

frame::frame()
{
    
}

frame::frame(cv::Mat img,int i,long long time):cameraID(i),timeStamp(time)
{
    imgData = img.clone();
}

frame::~frame()
{
    imgData.release();
}

frame::frame(const frame& a)
{
	if(this!=&a)
    {
        this->imgData.release();
		this->imgData = a.imgData.clone();
		this->cameraID = a.cameraID;
		this->timeStamp = a.timeStamp;
	}
}

frame& frame::operator=(const frame& a)
{
    if(this!=&a)
    {
        this->imgData.release();
        this->imgData = a.imgData.clone();
        this->cameraID = a.cameraID;
        this->timeStamp = a.timeStamp;
    }
    return *this;
}

bool frame::empty()
{
    return this->imgData.empty();
}