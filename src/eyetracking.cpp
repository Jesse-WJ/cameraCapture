/*
 * @Description: 
 * @Author: Jesse
 * @Date: 2022-12-09 15:28:28
 * @LastEditors: Jesse
 * @LastEditTime: 2023-01-10 16:01:21
 */
#include "eyetracking.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "cornea_calib.h"
#include "utils.h"
#include "cminpack.h"
#include <cstdio>
#include <qdatetime.h>
#include <vector>

using namespace cv;

tracker::tracker(QObject* parent) : QObject(parent)
{
    ImageCenter={320.168430300339, 245.003736613060};
    myopt = new optimization(8,4,1.65,1,1.34,{320.168430300339, 245.003736613060});
	file.open("../bin/data.csv");
}

tracker::~tracker()
{

}

cv::Point3d solve_OE(Mat OC, Mat P)
{
	Mat A = Mat::zeros(2, 2, CV_64FC1);
	Mat B = Mat::zeros(2, 1, CV_64FC1);
	for (int i = 0; i < 2; i++)
	{
		Mat oc_tmp = OC.rowRange(i, i + 1).clone();
		double temp = oc_tmp.at<double>(0, 2) / (-1.65);
		Mat oc_temp = oc_tmp / temp;
		A.at<double>(i, 0) = (P.at<double>(i, 1) - oc_temp.at<double>(0, 1)) / (P.at<double>(i, 0) - oc_temp.at<double>(0, 0));
		A.at<double>(i, 1) = -1;
		B.at<double>(i, 0) = (P.at<double>(i, 1) - oc_temp.at<double>(0, 1)) / (P.at<double>(i, 0) - oc_temp.at<double>(0, 0))*P.at<double>(i, 0) - P.at<double>(i, 1);
		//cout << A.at<double>(i, 0) << ',' << A.at<double>(i, 1) << ',' << B.at<double>(i, 0) << endl;
	}

	Mat ATA = A.t()*A;
	invert(ATA, ATA);
	Mat x = ATA * A.t()*B;
	Point3d point1(x.at<double>(0, 0), x.at<double>(1, 0), -1.65);
	return point1;
}

Point3d Intersection(Mat line_vec, Mat line_point)
{
	Mat I = Mat::eye(3, 3, CV_64FC1);
	Mat q = Mat::zeros(3, 1, CV_64FC1);
	Mat Msum = Mat::zeros(3, 3, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		Mat lineD_temp = line_vec.rowRange(i, i + 1).clone();
		normalize(lineD_temp, lineD_temp);
		Mat lineD = lineD_temp;
		Mat viviT = lineD.t()*lineD;
		Mat P = line_point.rowRange(i, i + 1).clone();
		Mat M = I - viviT;
		Msum = Msum + M;
		q = q + M * P.t();
	}
	invert(Msum, Msum);
	Mat x = Msum * q;
	Point3d eye(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
	return eye;
}

void tracker::getPOR(cv::Mat img)
{
	static std::vector<cv::Point> PORlist;
	static int porX,porY;
	static QDateTime lastTime = QDateTime::currentDateTime();
    QDateTime curTime = QDateTime::currentDateTime();
    qint64 intervalTimeMS = lastTime.msecsTo(curTime);
    lastTime = curTime;

    
    

    eillparam = Ellipse(img);
	if(eillparam.size.width==0)
	{
		return ;
	}
    std::vector<cv::Point2d> SpotParameter = L_SegmentSpotCalib(img,eillparam.center);
    if(SpotParameter[0].x ==0)
	{
		return;
	}
	std::cout<<intervalTimeMS<<"ms"<<std::endl;
    cv::cvtColor(img,img,cv::COLOR_GRAY2RGB);
    cv::ellipse(img,eillparam,cv::Scalar(0,255,0),-1);
    for(auto p :SpotParameter )
    {
        cv::circle(img,cv::Point(p),5,cv::Scalar(0,0,255),-1);
    }



    Point3d L1 = {-14.8662,-28.1115,29.018};
    Point3d L2 = {11.8974, -27.926, 26.5082};
    Point3d L3 = {-30.262151,-22.784739,21.988585};
    Point3d L4 = {24.123169,-21.318642,16.792984};
    Point3d L5 = {-30.200880625182418, -13.593387838082322 , 8.134705225021412};
	Point3d L6 = {20.86860584395294 , -8.095281115842548 , 10.22757801621854};
    Point3d L7 = {-16.913920709351583 , -3.5426354308919064 , 1.6359649600150157};
    Point3d L8 = {8.364206671324332 , -4.476015699053317 , -1.0948872031393384};
    Point3d G1img = {-(SpotParameter[0].x - ImageCenter.x)*0.003, -(SpotParameter[0].y - ImageCenter.y)*0.003, -1.65};
    Point3d G2img = {-(SpotParameter[1].x - ImageCenter.x)*0.003, -(SpotParameter[1].y - ImageCenter.y)*0.003, -1.65};
    Point3d G3img = {-(SpotParameter[2].x - ImageCenter.x)*0.003, -(SpotParameter[2].y - ImageCenter.y)*0.003, -1.65};
    Point3d G4img = {-(SpotParameter[3].x - ImageCenter.x)*0.003, -(SpotParameter[3].y - ImageCenter.y)*0.003, -1.65};
				

    cornea_calib cor = cornea_calib(L1,L2,L3,L4,G1img,G2img,G3img,G4img);
    if(cor.solve_lsq_cor())
    {
        cv::Point3d cornea = cor.getCorneaCenter();
        cv::Point3d ocImg3d = LinePlaneIntersection(cornea,cornea,{0,0,-1.65},{0,0,-1.65});

		// cv::Point3d t1 = cornea/cv::norm(cornea);
		// cv::Point3d t2 = ocImg3d/cv::norm(ocImg3d);

        ocImg2d = {-ocImg3d.x/0.003+ImageCenter.x,-ocImg3d.y/0.003+ImageCenter.y};
        cv::Point2f p_img_up = solve_pud();
        cv::Point2f p_img_down = p_img_up+ (eillparam.center-p_img_up)*2;

        cv::circle(img,cv::Point(p_img_up),5,cv::Scalar(255,255,255),-1);
        cv::circle(img,cv::Point(p_img_down),5,cv::Scalar(255,255,255),-1);
        cv::circle(img,cv::Point(ocImg2d),5,cv::Scalar(255,0,0),-1);
        
        // cv::imshow("",img);
        cv::Point3d pup = myopt->process_oc(p_img_up,p_img_down,cornea,cor.getCorneaRadius());
        printf("cornea:%f,%f,%f. pup:%f,%f,%f.\n",cornea.x,cornea.y,cornea.z,pup.x,pup.y,pup.z);
		if(index>0 && index<10)
		{
			file<<index<<','<<cornea.x<<','<<cornea.y<<','<<cornea.z<<','<<cor.getCorneaRadius()<<','<<pup.x<<','<<pup.y<<','<<pup.z<<std::endl;
		}

		static Point3d lastC={0,0,0};
		static Point3d lastP={0,0,0};
		static Point2d lastPimg={0,0};
		static Point3d lasteye = {0,0,0};
		if(lastC.x!=0 && lastC.y!=0 && lastC.z!=0)
		{
			cv::Mat OC = (Mat_<double>(2, 3) << lastC.x, lastC.y, lastC.z, cornea.x, cornea.y, cornea.z);
			Point3d lp = myopt->Trans_img_to_cam(lastPimg);
			Point3d np = myopt->Trans_img_to_cam(eillparam.center);
			cv::Mat P = (Mat_<double>(2, 3) << lp.x, lp.y, -1.65, np.x, np.y, -1.65);
			cv::Point3d OE = solve_OE(OC, P);
			//std::cout << "OE:" << OE.x << ',' << OE.y << ',' << OE.z << std::endl;

			cv::Mat line_vec = (Mat_<double>(3, 3) << pup.x, pup.y, pup.z,
			lastP.x, lastP.y, lastP.z, 0 - OE.x, 0 - OE.y, 0 - OE.z);
			cv::Mat line_ponit = (Mat_<double>(3, 3) << cornea.x, cornea.y, cornea.z, lastC.x, lastC.y, lastC.z,
			OE.x, OE.y, OE.z);
			cv::Point3d Eyecenter = Intersection(line_vec, line_ponit);
			
			if(lasteye.x==0)
			{
				lasteye = Eyecenter;
			}
			else
			{
				if(norm(Eyecenter-lasteye)<2)
				{
					Eyecenter = (Eyecenter+lasteye)/2;
					lasteye = Eyecenter;
				}
				else
				{
					Eyecenter=lasteye;
				}
			}
			printf("OE:%f,%f,%f. EYE:%f,%f,%f.\n",OE.x,OE.y,OE.z,Eyecenter.x,Eyecenter.y,Eyecenter.z);
			//std::cout << "眼球中心：" << Eyecenter.x << ',' << Eyecenter.y << ',' << Eyecenter.z << std::endl;
			
			cv::Point3d plane = (L1-L2).cross(L1-L7);
			cv::Point3d POINT2 = {6.32368049657640	,-15.3862885090551,	13.9399569910769};
			cv::Point3d POINT1 = {-9.20045723722834,	-15.1069332989318,	16.6047393814898};
			double dst = 1350/cv::norm(POINT2-POINT1);

			cv::Point3d POR = LinePlaneIntersection(Eyecenter,pup,POINT1,plane);

			cv::Point por = {int(abs(POR.x-POINT1.x)*dst)+250,int(sqrt(pow(cv::norm(POR-POINT1),2)-pow(POR.x-POINT1.x,2))*dst*1.7)};
			// std::cout<<sqrt(pow(cv::norm(POR-POINT1),2)-pow(POR.x-POINT1.x,2))*dst<<','<<dst<<std::endl;
			// cv::imshow("",img);
			// cv::waitKey(1);
			PORlist.push_back(por);
			if(PORlist.size()>5)
			{
				PORlist.erase(PORlist.begin());
			}
			cv::Point res = {0,0};
			for(auto ppo:PORlist)
			{
				res+=ppo/5;
			}

			printf("POR:%d,%d.\n\n",res.x,res.y);
			emit sendPOR(res);
		}
		else
		{
			lastC = cornea;
			lastPimg = eillparam.center;
			lastP = pup;
		}
						
		
		
	}
    
}

cv::RotatedRect tracker::Ellipse(cv::Mat input)
{

	cv::Mat temp = input.clone();

	cv::threshold(temp, temp, 20, 255, cv::THRESH_BINARY);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20, 20));
	cv::morphologyEx(temp, temp, cv::MORPH_OPEN, kernel);

	cv::Mat Image = temp.clone();
	cv::cvtColor(Image,Image,cv::COLOR_GRAY2RGB);
	// imshow("3", temp);
	// waitKey(1);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(temp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	int num = contours.size();

	if (num < 1)
	{
		return {{0,0},{0,0},0};
	}

	cv::RotatedRect res={{0,0},{0,0},0};
	for (size_t i = 0; i < contours.size(); i++)
	{
		size_t count = contours[i].size();
		//drawContours(input, contours, i, Scalar(255,0,0));
		//imshow("7", input);
		/*个数必须大于6，这是cvFitEllipse_32f的要求*/
		if (count < 6)
		{
			continue;
		}


		cv::RotatedRect Box = cv::fitEllipse(contours[i]);

		if ((Box.size.width*0.5) / (Box.size.height*0.5) < 1.3 && (Box.size.width*0.5) / (Box.size.height*0.5) > 0.7)
		{
			if (Box.size.width > 50 && Box.size.width < 100)
			{
				res=Box;
			}
		}
	}
	if(res.size.width==0)
		return {{0,0},{0,0},0};
	return res;
}

//////////////////////////////左眼光斑分割//////////////////////////
std::vector<cv::Point2d> tracker::L_SegmentSpotCalib(cv::Mat pInput, cv::Point2f pupilcenter)
{
	std::vector<cv::Point2d> res(8,cv::Point2d(0,0));
	cv::Mat pTemp = pInput.clone();
	cv::Mat img2 = pTemp.clone();
	cv::threshold(pTemp, pTemp, 170, 255, cv::THRESH_BINARY);//此处阈值选择200的原因是因为白色目标很突出，固定阈值可以节省处理时间
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
	cv::morphologyEx(pTemp, pTemp, cv::MORPH_OPEN, kernel);

	cv::Mat Image = pTemp.clone();
	cv::cvtColor(Image,Image,cv::COLOR_GRAY2RGB);

	// imshow("4", pTemp);
	// waitKey(1);
	//提取轮廓  
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(pTemp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<int> number;
	std::vector<cv::Point2f>  SpotCenter;
	std::vector<cv::Point>  SpotCenter_2_hull;
	//此处为光斑个数大于2才做处理
	if (contours.size() >= 2)
	{
		/*这一过程主要是为了得到所有轮廓的中心坐标*/
		for (int i = 0; i < contours.size(); i++)
		{
			if (contours[i].size() > 3)
			{

				cv::Rect  tempRect = { 0,0,0,0 };
				cv::Point2f  tempCenter = { 0,0 };
				double M_00 = 0, M_01 = 0, M_10 = 0;
				int x, y;
				tempRect = cv::boundingRect(contours[i]);
				//for(int i=0;i<)
				//cout << tempRect.x << ',' << tempRect.y << endl;
				for (int ii = 0; ii < tempRect.width; ii++)
				{
					for (int jj = 0; jj < tempRect.height; jj++)
					{
						x = tempRect.x + ii;
						y = tempRect.y + jj;
						M_00 = M_00 + pTemp.at<uchar>(y, x);
						M_01 = M_01 + x * pTemp.at<uchar>(y, x);
						M_10 = M_10 + y * pTemp.at<uchar>(y, x);
					}
				}
				tempCenter.x = M_01 / M_00;
				tempCenter.y = M_10 / M_00;
				if (fabs(tempRect.x - pupilcenter.x) <= 100 && fabs(tempRect.y - pupilcenter.y) <= 60)
				{
					SpotCenter.push_back(tempCenter);
					SpotCenter_2_hull.push_back(tempCenter);
				}

			}
		}
		// cout<<SpotCenter_2_hull.size()<<endl;
		if (SpotCenter_2_hull.size() < 6)
		{
			// std::cout<<SpotCenter_2_hull.size()<<std::endl;
			return {{0,0},{0,0}};
		}
		//通过凸包进行椭圆拟合
		std::vector<cv::Point> hull;
		cv::RotatedRect Box;

		cv::convexHull(SpotCenter_2_hull, hull);
		if (hull.size() < 6)
		{
			// std::cout<<hull.size()<<std::endl;
			return {{0,0},{0,0}};
		}
		Box = cv::fitEllipse(hull);

		



		//cout << "Box.angle=" << Box.angle << endl;
		for (int i = 0; i < SpotCenter.size(); i++)
		{
			double xita = atan((Box.center.y - SpotCenter[i].y) / (Box.center.x - SpotCenter[i].x)) * 180 / 3.1415;

			//cout << xita << ',';
			if (Box.angle <= 90)
			{
				xita = abs(xita) - (90 - Box.angle);
			}
			else
			{
				xita = abs(xita) + (Box.angle - 90);
			}
			//cout << xita << ',';

			if (Box.center.y >= SpotCenter[i].y && Box.center.x <= SpotCenter[i].x)//第一象限
			{

				xita = 90 - xita;
				//cout << "第一象限:" << xita << endl;
			}
			if (Box.center.y > SpotCenter[i].y && Box.center.x > SpotCenter[i].x)//第二象限
			{

				xita = xita + 270;
				//cout << "第二象限:" << xita << endl;
			}
			if (Box.center.y < SpotCenter[i].y && Box.center.x > SpotCenter[i].x)//第三象限
			{

				xita = 270 - xita;
				//cout << "第三象限:" << xita << endl;
			}
			if (Box.center.y <= SpotCenter[i].y && Box.center.x <= SpotCenter[i].x)//第四象限
			{

				xita = xita + 90;
				//cout << "第四象限:" << xita << endl;
			}


			if (xita < 360 && xita >= 296)
			{
				res[0]=SpotCenter[i];
			}
			if (xita < 71 && xita >= 0)
			{
				res[1]=SpotCenter[i];
			}
			if (xita < 296 && xita >= 265)
			{
				res[2]=SpotCenter[i];
			}
			if (xita < 99 && xita >= 71)
			{
				res[3]=SpotCenter[i];
			}
			if (xita < 265 && xita >= 238)
			{
				res[4]=SpotCenter[i];
			}
			if (xita < 125 && xita >= 99)
			{
				res[5]=SpotCenter[i];
			}
			if (xita < 238 && xita >= 181)
			{
				res[6]=SpotCenter[i];
			}
			if (xita < 181 && xita >= 125)
			{
				res[7]=SpotCenter[i];
			}
			
			// for (int j=0; j < number.size(); j++)
			// {
			// 	cout << number[j] <<' ';
			// }
			// cout << endl;
			// circle(img2, SpotCenter[i], 5, (255, 255, 255));
			// line(img2, Point(Box.center.x - 40, Box.center.y), Point(Box.center.x + 40, Box.center.y), (255, 255, 255));
			// imshow("2", img2);
			// waitKey(0);
		}
	}

	return res;
}

int solve_ud(void *p, int m, int n, const double *X, double *Errorparameter, int iflag)
{
    tracker * me = (tracker *)p;

    cv::Point2d op = me->eillparam.center;
    cv::Point2d c = me->ocImg2d;

    double xpos, ypos, radm, radn, an;
	double co, si;
	double tempang, tempx, tempy;
	xpos = me->eillparam.center.x;//要修改
	ypos = me->eillparam.center.y;//假设相机分辨率为640*480
	radm = me->eillparam.size.width/2;
	radn = me->eillparam.size.height/2;
	an = me->eillparam.angle/180*CV_PI;
	co = cos(an);
	si = sin(an);

    tempang = X[0];
    tempx = radm * cos(tempang)*co - si * radn*sin(tempang) + xpos;
    tempy = radm * cos(tempang)*si + co * radn*sin(tempang) + ypos;

    cv::Point2d p1 = {tempx,tempy};
    cv::Point2d v1 = op-c;
    cv::Point2d v2 = p1-c;
    cv::Point2d v3 = p1-op;
    v1=v1/cv::norm(v1);
    v2=v2/cv::norm(v2);
    v3=v3/cv::norm(v3);

    Errorparameter[0] =abs(v1.dot(v2))-1;
    Errorparameter[1] =abs(v1.dot(v3))-1;

    return 0;
}

cv::Point2f tracker::solve_pud()
{
    int m=2, n=1;
    int info, lwa= m*n+5*n+m+1, iwa[n];	
    double  x0[n], Errorparameter[m];
    double tol=1e-15, wa[lwa];

    while(1)
    {
        x0[0] = getRandData(-1,1);//0~600的随机数

        info=__cminpack_func__(lmdif1)(solve_ud, this, m, n, x0, Errorparameter, tol, iwa, wa, lwa);

        double xpos, ypos, radm, radn, an;
        double co, si;
        double tempang, tempx, tempy;
        xpos = eillparam.center.x;//要修改
        ypos = eillparam.center.y;//假设相机分辨率为640*480
        radm = eillparam.size.width/2;
        radn = eillparam.size.height/2;
        an = eillparam.angle/180*CV_PI;
        co = cos(an);
        si = sin(an);

        tempang = x0[0];
        tempx = radm * cos(tempang)*co - si * radn*sin(tempang) + xpos;
        tempy = radm * cos(tempang)*si + co * radn*sin(tempang) + ypos;

        cv::Point2d p1 = {tempx,tempy};
        cv::Point2d v1 = eillparam.center-cv::Point2f(ocImg2d);
        cv::Point2d v2 = p1-ocImg2d;
        cv::Point2d v3 = p1-cv::Point2d(eillparam.center);
        v1=v1/cv::norm(v1);
        v2=v2/cv::norm(v2);
        v3=v3/cv::norm(v3);

        double error =(abs(abs(v1.dot(v2))-1) + abs(abs(v1.dot(v3))-1))/2;
        if(error<1e-5)
        {
            printf("error1:%f,error2:%f.\n",abs(v1.dot(v2))-1,abs(v1.dot(v3))-1);
            return {tempx,tempy};
        }
        
    }
    
    return {0,0};
}