/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-14 17:01:06
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 18:29:25
 */
#ifndef CORNEA_CALIB_H_
#define CORNEA_CALIB_H_

#include <opencv2/opencv.hpp>

using namespace cv;



class cornea_calib
{

public:
    cornea_calib(Point3d l1,Point3d l2,Point3d l3,Point3d l4,
                Point3d g1img,Point3d g2img,Point3d g3img,Point3d g4img);
    cornea_calib(Point3d l3,Point3d l4,
                Point3d g3img,Point3d g4img);
    ~cornea_calib();

    Point3d getCorneaCenter();
    double getCorneaRadius();

    bool solve_lsq_cor();

private:
    Point3d corneaCenter;
    double  corneaRadius;

    Point3d EOC;			
	Point3d G1img;			
	Point3d G2img;			
	Point3d L1;			
	Point3d L2;			
	Point3d G3img;			
	Point3d G4img;			
	Point3d L3;			
	Point3d L4;		

    int mod;

    void Init4();
    void Init2();

    static int solve_cornea4(void *p, int m, int n, const double *X, double *Errorparameter, int iflag);
    static int solve_cornea2(void *p, int m, int n, const double *X, double *Errorparameter, int iflag);

};



#endif