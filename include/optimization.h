/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-14 15:29:34
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-15 13:39:42
 */
#ifndef OPTIMIZATION_H_
#define OPTIMIZATION_H_

#include <opencv2/opencv.hpp>

class optimization
{
public:
    optimization(double r_t,double ce,double f,double n_0,double n_1,cv::Point2d ic);
    ~optimization();
    cv::Point3d process_oc(cv::Point2d p_img_up,cv::Point2d p_img_down,cv::Point3d C_t,double r_true);
    cv::Point3d Trans_img_to_cam(cv::Point2d p);
    
private:
    double R_t;
    double CE;
    double F;
    double n0;
    double n1;
    cv::Point2d ImgCenter;

    
    cv::Point3d Check(cv::Point3d p);
    cv::Point3d Correct_op(cv::Point3d oc, cv::Point3d op, cv::Point3d opu, cv::Point3d opd);
    cv::Point3d incident_solve(cv::Point3d out, cv::Point3d g, cv::Point3d cornea, double n0, double n1);
};


#endif