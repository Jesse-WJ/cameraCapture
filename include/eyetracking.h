/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-09 15:28:19
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-15 13:46:27
 */
#ifndef EYETRACKING_H_
#define EYETRACKING_H_
#include <QObject>
#include <opencv2/opencv.hpp>
#include "optimization.h"
#include <vector>
#include <fstream>


class tracker : public QObject {
    Q_OBJECT
    
public:
    tracker(QObject* parent = nullptr);
    ~tracker();
    void getPOR(cv::Mat img);
private:
    optimization* myopt;
    cv::RotatedRect Ellipse(cv::Mat input);
    std::vector<cv::Point2d> L_SegmentSpotCalib(cv::Mat pInput, cv::Point2f pupilcenter);
    cv::Point2d ImageCenter;
    cv::Point2f solve_pud();
    // int solve_ud(void *p, int m, int n, const double *X, double *Errorparameter, int iflag);
    
public:
    cv::Point2d ocImg2d;
    cv::RotatedRect eillparam;
    std::ofstream file;
    int index;

signals:
    void sendPOR(cv::Point POR);
};

#endif