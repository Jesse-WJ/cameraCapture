/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-14 15:29:24
 * @LastEditors: Jesse
 * @LastEditTime: 2023-01-10 15:55:08
 */
#include "optimization.h"
#include <math.h>
#include <cstdio>
#include "utils.h"
#include <opencv2/opencv.hpp>

optimization::optimization(double r_t,double ce,double f,double n_0,double n_1,cv::Point2d ic):
                            R_t(r_t),CE(ce),n0(n_0),n1(n_1),F(f),ImgCenter(ic)
{

}

optimization::~optimization()
{

}

cv::Point3d optimization::process_oc(cv::Point2d p_img_up,cv::Point2d p_img_down,cv::Point3d C,double r_true)
{
    //将图像坐标系转化为相机坐标系
	// 设定初值
	cv::Point2d p_img_middle = (p_img_up + p_img_down) / 2;										// 图像ocp切瞳孔中点
	cv::Point3d oc = C / cv::norm(C);															// oc单位方向向量
	//图像坐标系->相机坐标系
	cv::Point3d piu = Trans_img_to_cam(p_img_up);													//ocp切瞳孔上边缘点
	cv::Point3d pid = Trans_img_to_cam(p_img_down);													//ocp切瞳孔下边缘点
	cv::Point3d pim = Trans_img_to_cam(p_img_middle);												//ocp切瞳孔中点

	//单位化
	cv::Point3d opiu = piu / cv::norm(piu);
	cv::Point3d opim = pim / cv::norm(pim);
	cv::Point3d opid = pid / cv::norm(pid);

    // cv::Point3d opiu = {0.115912,0.0146294,-1.65};
	// cv::Point3d opid = {0.0849074,-0.172349,-1.65};
	// cv::Point3d opim = (opiu+opid)/2;
    
	//方向统一，单位向量均朝向用户
	opiu = Check(opiu);
	opim = Check(opim);
	opid = Check(opid);
	//op修正
	// cv::Point3d op = Correct_op(oc, opim, piu, pid);
	cv::Point3d op = opim;
	op = Check(op);
	//寻找最优角膜半径
	r_true = R_t - 0.05*R_t;
	int range_k = 60;																	//步长约为0.01mm 总范围+—0.05*R
	double min_error = 2;
	double new_ce = CE;
	cv::Point3d opt = {0,0,0};
    cv::Point3d O = {0,0,0};
    cv::Point3d g_op;
    cv::Point3d g1,g2,g_up,g_dowm;
    double r;
    double t_r_range;
	cv::Point3d pppup;
	for(int k=0;k<60;++k)
    {
        r = R_t -1 + k/30;
		// r= r_true;
		// 求折射点
		g_up = line_sphere_intersection(O, opiu, C, r); 									//上边缘点角膜表面折射点
		g_dowm = line_sphere_intersection(O, opid, C, r);  								// 下边缘点角膜表面折射点
		g_op = line_sphere_intersection(O, op, C, r);  									// 近似瞳孔中心角膜表面折射点
        // 求入射光线
		cv::Point3d incident_up = incident_solve(opiu, g_up, C, n0, n1);
		cv::Point3d incident_op = incident_solve(op, g_op, C, n0, n1);
		cv::Point3d incident_dowm = incident_solve(opid, g_dowm, C, n0, n1);
		//寻找最佳CE 范围3.5-4.5
		int range_i = 50;
        
		for(int i=0;i<range_i;++i)
        {
            double r_range = CE - 1 + 0.04 * i;
			g1 = line_sphere_intersection(g_up, incident_up, C, r_range);
			g2 = line_sphere_intersection(g_dowm, incident_dowm, C, r_range);
			if (g1.z != 0 && g2.z != 0)
			{	
                cv::Point3d gp = (g1 + g2) / 2;
				cv::Point3d g1g2 = Check((g2 - g1) / cv::norm(g2 - g1));
				cv::Point3d cgp = Check((C - gp) / cv::norm(C - gp));
				cv::Point3d pup1 = LineLineIntersection(g1,g1-g2,g_op,incident_op);
				double error_temp = cv::norm(pup1-gp);
                // if(g1g2.z==0 || cgp.z==0 || error==0)
                // {
                // printf("error_in:%f\n",error_temp);
                // }
				// printf("error_in:%f\n",error);
                // printf("gp:%f,%f,%f\n",gp.x,gp.y,gp.z);
                // printf("g1g2:%f,%f,%f\n",g1g2.x,g1g2.y,g1g2.z);
                // printf("cgp:%f,%f,%f\n",cgp.x,cgp.y,cgp.z);
                // std::cout<<g1g2.x*cgp.x<<std::endl;
				// std::cout<<g1g2.y*cgp.y<<std::endl;
				// std::cout<<g1g2.z*cgp.z<<std::endl;
				// std::cout<<g1g2.x*cgp.x+g1g2.y*cgp.y<<std::endl;
                // cv::waitKey(0);
				if (error_temp <= min_error)
				{	
                    min_error = error_temp;
                    t_r_range = r_range;
					new_ce = cv::norm(C - gp);
					// r_true = r;
					opt = cgp;
					pppup = gp;
                }
            }
        }
			
    }
    g_up = line_sphere_intersection(O, opiu, C, r_true); 									//上边缘点角膜表面折射点
	g_dowm = line_sphere_intersection(O, opid, C, r_true); 
    cv::Point3d incident_up = incident_solve(opiu, g_up, C, n0, n1);
    cv::Point3d incident_dowm = incident_solve(opid, g_dowm, C, n0, n1);
    g1 = line_sphere_intersection(g_up, incident_up, C, t_r_range);
	g2 = line_sphere_intersection(g_dowm, incident_dowm, C, t_r_range);
	// printf("c:%f,%f,%f.pppup:%f,%f,%f.\n",C.x,C.y,C.z,pppup.x,pppup.y,pppup.z);
	// printf("g1:%f,%f,%f.g2:%f,%f,%f.\n",g1.x,g1.y,g1.z,g2.x,g2.y,g2.z);

	// printf("min_error:%f\n",min_error);
	// printf("CE:%f\n",new_ce);
	// printf("r:%f\n",r_true);
	if (opt.z == 0)
    {
        cv::Point3d pp = line_sphere_intersection(g_op, op, C, r);
		opt = Check((C - pp) / cv::norm(C - pp));
    }
	// CE = (new_ce+CE)/2;
    // R_t = r_true;
	return opt;
}

cv::Point3d optimization::Trans_img_to_cam(cv::Point2d p)
{
    return {-(p.x-ImgCenter.x)*0.003,-(p.y-ImgCenter.y)*0.003,-F};
}

cv::Point3d optimization::Check(cv::Point3d p)
{
    if(p.z<0)
    {
        return -p;
    }
    else{
        return p;
    }
}

cv::Point3d optimization::Correct_op(cv::Point3d oc, cv::Point3d op, cv::Point3d opu, cv::Point3d opd)
{
    double angle = acos(oc.dot(op));
    if(angle>1)
    {
        printf("error:,%f.\n",angle);
    }
    else
    {
        // printf("angle:%f\n",angle);
        op = opu+((1-angle)*(opd-opu))/2;
		op = op / cv::norm(op);
    }
    return op;
}

cv::Point3d optimization::incident_solve(cv::Point3d out, cv::Point3d g, cv::Point3d cornea, double n0, double n1)
{
    cv::Point3d N = cornea - g;																		//法向量
	N = N / cv::norm(N);														//法向量单位向量

	cv::Point3d incident = out + N * (
				sqrt((pow(n1, 2) - pow(n0, 2)) + pow(N.dot(out), 2)) - N.dot(out));  // 求入射光线方向向量
	incident = incident / cv::norm(incident);
	// print('反射点坐标：',g)
	//print('入射光线单位向量：', incident)
	// print('法线单位向量：',N)
	// print('折射光线单位向量：',-out)
	return incident;
}