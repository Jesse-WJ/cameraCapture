/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-11-24 15:43:22
 * @LastEditors: Jesse
 * @LastEditTime: 2022-12-14 16:09:44
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/opencv.hpp>
#include <cmath>
using namespace cv;
using namespace std;

double getRandData(int min, int max);

Point3d LinePlaneIntersection(Point3d p1, Point3d v1, Point3d p2, Point3d v2);

Point3d LineLineIntersection(Point3d p1,Point3d v1,Point3d p2,Point3d v2);

double Get_angle(Point3d v1, Point3d v2);

Point3d line_sphere_intersection(Point3d line_p, Point3d line_v, Point3d center, double r);

pair<Mat,Mat> getRotateMatrix(Point3d xs,Point3d ys,Point3d zs,Point3d xd={1,0,0},Point3d yd={0,1,0},Point3d zd={0,0,1});

class get_R
{
public:
    Mat operator()(Point3d x1,Point3d y1,Point3d z1,Point3d x2,Point3d y2,Point3d z2);
private:
    Mat xs,ys,zs,xd,yd,zd;

    static int fcn(void *p, int m, int n, const double *X, double *Errorparameter, int iflag);
};
#endif