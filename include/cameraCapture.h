/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:09:27
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-15 13:56:21
 */
#pragma once
#include "ui_cameraCapture.h"
#include <QMainWindow>
#include "frame.h"
#include "subwin.h"
#include <opencv2/opencv.hpp>
#include "eyetracking.h"
#include "locker.h"
#include "mygl.h"
// #include <QOpenGLWidget>

class cameraCapture : public QMainWindow {
    Q_OBJECT
    
public:
    cameraCapture(QWidget* parent = nullptr);
    ~cameraCapture();
    void showImageAPI(frame img);
    enum FRAMEID {SCENE=0,LEFT,RIGHT};
protected:
    bool eventFilter(QObject *obj, QEvent *event);
    
private slots:
    void ClickOpenCameraButton();
    void ClickCloseCameraButton();
    void ClickCaptureButton();
    void ClicktestglButton();
    void saveImage(int id);

private:
    Ui_cameraCapture* ui;
    tracker myTracker;
    locker mylock;
    subwin capture;
    MyGLWidget mygl;
    frame sceneImage,leftImage,rightImage;
    int myTimer;
    bool Enalbeshow;
    void showImage(frame img);
    void timerEvent(QTimerEvent *event);
    void listFiles(std::string dir);
};