/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-06 21:38:39
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 14:12:15
 */
#pragma once
#include "ui_subwin.h"
#include <QMainWindow>
#include <QPalette>
#include <opencv2/opencv.hpp>
#include "motion.h"


class subwin : public QMainWindow {
    Q_OBJECT
    
public:
    subwin(QWidget* parent = nullptr);
    ~subwin();

private:
    Ui_subwin* ui;
    QPalette palette;
    motion m;
    cv::Mat screen;
    cv::Point moveDot;
    int myTimer;

    void reflush();
    void resizeEvent(QResizeEvent * e);
    void timerEvent(QTimerEvent *event);

private slots:
    void on_subwin_customContextMenuRequested(const QPoint &pos);
    void maxScreen();
    void track();
public slots:
    void showPOR(cv::Point POR);
signals:
    void sendcalibPointID(int calibPointID);

};