/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:37:41
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:15:07
 */
#ifndef GLASS_H_
#define GLASS_H_

#include <queue>
// #include <QApplication>

#include "aSeeGlassesUserApi.h"
#include <opencv2/opencv.hpp>
#include "locker.h"
#include "threadpool.h"
#include "frame.h"
#include "cameraCapture.h"
#include <fstream>
#include "cameraCapture.h"

using namespace API_ASEEGLASSES_7INVENSUN;

class glass{
public:
    static glass *Instance();

    bool Init();

    bool Start();

    bool Stop();

    void Release();

    static void _7INVENSUN_CALL SceneImageCallback(uint8_t* image, int32_t size, int32_t width, int32_t height, int64_t timestamp);

	static void _7INVENSUN_CALL EyeImageCallback(int32_t flag, uint8_t* image, int32_t size, int32_t width, int32_t height, int64_t timestamp);

private:
    class CGarbo
    {
    public:
        ~CGarbo()
        {   
            if(glass::p)
            {
                delete p;
                p=nullptr;
                printf("delete glass::p.\n");
            }
            
        }
    };
    static CGarbo Garbo; 

private:
    glass(){};
    ~glass(){};
    glass(glass &a);
    glass& operator=(glass &a);
    static glass* p;
    bool Isinited;
    cameraCapture w;
};

#endif