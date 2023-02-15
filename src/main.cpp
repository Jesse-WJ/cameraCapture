/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:09:27
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:25:05
 */
#include "cameraCapture.h"
#include "glass.h"
#include <QApplication>
#include <QOpenGLWidget>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // cameraCapture w;
    // w.show();

    glass *myglass = glass::Instance();
    myglass->Init();
    
    return a.exec();
}