#include "utils.h"
#include "cminpack.h"
#include <cstdlib>


double getRandData(int min, int max)
{
	double m1 = (double)(rand() % 101) / 101;               //计算 0，1之间的随机小数,得到的值域近似为(0,1)
	min++;												    //将 区间变为(min+1,max),
	double m2 = (double)((rand() % (max - min + 1)) + min); //计算 min+1,max 之间的随机整数，得到的值域为[min+1,max]
	m2 = m2 - 1;											//令值域为[min,max-1]
	return m1 + m2;											//返回值域为(min,max),为所求随机浮点数
}

Point3d LinePlaneIntersection(Point3d p1, Point3d v1, Point3d p2, Point3d v2)
{    /*
    求空间直线和空间平面的交点
    p1和v1为空间直线经过的点和方向向量
    p2和v2为空间平面经过的点和法向量
    */
    double t = (p2.dot(v2)-p1.dot(v2))/v1.dot(v2);
    return p1+t*v1;
}

Point3d LineLineIntersection(Point3d p1,Point3d v1,Point3d p2,Point3d v2)
{    /*
    求空间两直线的交点
    v1和v2是方向向量
    p1和p2是直线上一点
    */
    v1 = v1/norm(v1);
    v2 = v2/norm(v2);
    Point3d startPointSeg = p2-p1;
    Point3d vecS1 = v1.cross(v2);
    Point3d vecS2 = startPointSeg.cross(v2);
    double num = vecS2.dot(vecS1)/pow(norm(vecS1),2);

    return p1+v1*num;
}

double Get_angle(Point3d v1, Point3d v2)
{
    v1 = v1/norm(v1);
    v2 = v2/norm(v2);
    return acos(v1.dot(v2))/CV_PI*180;
}


Point3d line_sphere_intersection(Point3d line_p, Point3d line_v, Point3d center, double r)
{
	Point3d point1, point2;
	double  a = (pow(line_v.x, 2) + pow(line_v.y, 2) + pow(line_v.z, 2)) / pow(line_v.x, 2);
	double	b = 2 * (-center.x + (line_p.y - line_v.y / line_v.x * line_p.x)* line_v.y / line_v.x- center.y * line_v.y / line_v.x 
				+ (line_p.z - line_v.z / line_v.x * line_p.x) * line_v.z / line_v.x - center.z * line_v.z / line_v.x);
	double	c = pow(center.x, 2) + pow(line_p.y - line_v.y / line_v.x * line_p.x, 2) - 2 * center.y * (line_p.y - line_v.y / line_v.x * line_p.x) + pow(center.y, 2) + \
				pow(line_p.z - line_v.z / line_v.x * line_p.x, 2) - 2 * center.z * (line_p.z - line_v.z / line_v.x * line_p.x) + pow(center.z, 2) - pow(r, 2);
	double	deta = pow(b, 2) - 4 * a * c;
	if (deta < 0)
	{
		point1={0,0,0};
		return point1;
	}
	else
	{
		double	x1 = (-b + sqrt(deta)) / 2 / a;
		double	x2 = (-b - sqrt(deta)) / 2 / a;
		double	y1 = line_v.y * x1 / line_v.x + line_p.y - line_v.y / line_v.x * line_p.x;
		double	y2 = line_v.y * x2 / line_v.x + line_p.y - line_v.y / line_v.x * line_p.x;
		double	z1 = line_v.z * x1 / line_v.x + line_p.z - line_v.z / line_v.x * line_p.x;
		double	z2 = line_v.z * x2 / line_v.x + line_p.z - line_v.z / line_v.x * line_p.x;
		if (sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) < sqrt(pow(x2, 2) + pow(y2, 2) + pow(z2, 2)))
		{
			point1={x1, y1, z1};
			point2={x2, y2, z2};
		}
		else
		{
			point2={x1, y1, z1};
			point1={x2, y2, z2};
		}	
	}
	return point1;
}

int get_R::fcn(void *p, int m, int n, const double *X, double *Errorparameter, int iflag)
{
	get_R * ptr = (get_R*)p;
	double a=X[0];
	double b=X[1];
	double c=X[2];

	Mat Rx=(Mat_<double>(3,3)<< 1,0,0,0,cos(a),sin(a),0,-sin(a),cos(a));
	Mat Ry=(Mat_<double>(3,3)<<cos(b),0,-sin(b),0,1,0,sin(b),0,cos(b));
	Mat Rz=(Mat_<double>(3,3)<<cos(c),sin(c),0,-sin(c),cos(c),0,0,0,1);

	Mat R = Rx*Ry*Rz;


	Mat error1 = R*ptr->xs-ptr->xd;
	Mat error2 = R*ptr->ys-ptr->yd;
	Mat error3 = R*ptr->zs-ptr->zd;

	int i=0;
	for(;i<3;++i)
		Errorparameter[i]=error1.at<double>(i%3);
	for(;i<6;++i)
		Errorparameter[i]=error2.at<double>(i%3);
	for(;i<9;++i)
		Errorparameter[i]=error3.at<double>(i%3);

	return 0;
}



Mat get_R::operator()(Point3d x1,Point3d y1,Point3d z1,Point3d x2,Point3d y2,Point3d z2)
{
	xs=(Mat_<double>(3,1)<<x1.x,x1.y,x1.z);
	ys=(Mat_<double>(3,1)<<y1.x,y1.y,y1.z);
	zs=(Mat_<double>(3,1)<<z1.x,z1.y,z1.z);
	xd=(Mat_<double>(3,1)<<x2.x,x2.y,x2.z);
	yd=(Mat_<double>(3,1)<<y2.x,y2.y,y2.z);
	zd=(Mat_<double>(3,1)<<z2.x,z2.y,z2.z);

	int m=9, n=3;
    int info, lwa= m*n+5*n+m+1, iwa[n];	
	double Errorparameter[m];
    double tol=1e-8, wa[lwa];

	double x0[n] = {getRandData(0,CV_2PI),getRandData(0,CV_2PI),getRandData(0,CV_2PI)};

	info=__cminpack_func__(lmdif1)(fcn, this, m, n, x0, Errorparameter, tol, iwa, wa, lwa);

	double a=x0[0],b=x0[1],c=x0[2];

    Mat Rx=(Mat_<double>(3,3)<< 1,0,0,0,cos(a),sin(a),0,-sin(a),cos(a));
	Mat Ry=(Mat_<double>(3,3)<<cos(b),0,-sin(b),0,1,0,sin(b),0,cos(b));
	Mat Rz=(Mat_<double>(3,3)<<cos(c),sin(c),0,-sin(c),cos(c),0,0,0,1);

	Mat R = Rx*Ry*Rz;

	return R;
}

pair<Mat,Mat> getRotateMatrix(Point3d xs,Point3d ys,Point3d zs,Point3d xd,Point3d yd,Point3d zd)
{
    xs = xs / norm(xs);
    ys = ys / norm(ys);
    zs = ((zs.z>0)?-zs:zs) / norm(zs);

    if(ys.dot(zs.cross(xs))<0)
        ys = -ys;

    get_R f;
    Mat R1 = f(xs,ys,zs,xd,yd,zd);
    Mat R2 = f(xd,yd,zd,xs,ys,zs);

	// Mat x1 = (Mat_<double>(3,1)<< zd.x,zd.y,zd.z);
	// Mat temp = R2*x1;
	// cout<<temp.at<double>(0)<<','<<temp.at<double>(1)<<','<<temp.at<double>(2)<<endl;
    
    return {R1,R2};
}

vector<Point3d> Equal_circle(vector<Point2d>& pupilEdge,Point2d ImageCenter, double Foc)
{
	vector<Point3d> res;
	for(auto pos :pupilEdge)
	{
		Point3d temp = {{-(pos.x - ImageCenter.x)*0.003, -(pos.y - ImageCenter.y)*0.003, -Foc}};
		res.emplace_back(temp);
	}
	return res;
}

pair<RotatedRect,Rect> Ellipse(Mat &input)
{
	Mat temp = input.clone();

	threshold(temp, temp, 20, 255, THRESH_BINARY);
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(20, 20));
	morphologyEx(temp, temp, MORPH_OPEN, kernel);
	
	// imshow("3", temp);
	// waitKey(1);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(temp, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	int num = contours.size();

	if (num < 1)
	{
		return {  };
	}

	RotatedRect res1={};
	Rect res2={};
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
		RotatedRect tempres = fitEllipse(contours[i]);	
		if( tempres.size.width>=20 && tempres.size.height>=20 && tempres.size.width/tempres.size.height>=0.8 && tempres.size.width/tempres.size.height<=1.2)
		{
			res1 = tempres;
			res2 = boundingRect(contours[i]);
		}
		
	};

	return {res1,res2};
}

size_t Point2dhash(const Point2d& p) 
{
	size_t h1 = hash<double>()(p.x);
	size_t h2 = hash<double>()(p.y);
	return h1 ^ (h2 << 1);
}