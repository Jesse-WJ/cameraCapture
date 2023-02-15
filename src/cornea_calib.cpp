#include "cornea_calib.h"
#include <cmath>
#include "cminpack.h"
#include "utils.h"

cornea_calib::cornea_calib(Point3d l1,Point3d l2,Point3d l3,Point3d l4,
    Point3d g1img,Point3d g2img,Point3d g3img,Point3d g4img):L1(l1),L2(l2),L3(l3),L4(l4),
    G1img(g1img),G2img(g2img),G3img(g3img),G4img(g4img)
{
	mod=4;
}

cornea_calib::cornea_calib(Point3d l3,Point3d l4,Point3d g3img,Point3d g4img):L3(l3),L4(l4),G3img(g3img),G4img(g4img)
{
	mod=2;
}

cornea_calib::~cornea_calib()
{

}

void cornea_calib::Init4()
{
    G1img = ((G1img.z<0)?(-G1img):G1img)/norm(G1img);
    G2img = ((G2img.z<0)?(-G2img):G2img)/norm(G2img);
    G3img = ((G3img.z<0)?(-G3img):G3img)/norm(G3img);
    G4img = ((G4img.z<0)?(-G4img):G4img)/norm(G4img);

    corneaCenter = {0,0,0};
    corneaRadius = 0;

    Point3d OL1G1img = G1img.cross(L1);
    Point3d OL2G2img = G2img.cross(L2);
    Point3d OL3G3img = G3img.cross(L3);
    Point3d OL4G4img = G4img.cross(L4);

    Point3d EOC_temp[4];
    EOC_temp[0] = OL1G1img.cross(OL2G2img); 
	EOC_temp[1] = OL1G1img.cross(OL4G4img); 
	EOC_temp[2] = OL2G2img.cross(OL3G3img); 
	EOC_temp[3] = OL3G3img.cross(OL4G4img); 

    EOC = (EOC_temp[0]+EOC_temp[1]+EOC_temp[2]+EOC_temp[3])/4;
    EOC = ((EOC.z<0)?(-EOC):EOC)/norm(EOC);
	// cout<<EOC.x<<','<<EOC.y<<','<<EOC.z<<endl;
}

void cornea_calib::Init2()
{
    G3img = ((G3img.z<0)?(-G3img):G3img)/norm(G3img);
    G4img = ((G4img.z<0)?(-G4img):G4img)/norm(G4img);

    corneaCenter = {0,0,0};
    corneaRadius = 0;

    Point3d OL3G3img = G3img.cross(L3);
    Point3d OL4G4img = G4img.cross(L4);

	EOC = OL3G3img.cross(OL4G4img); 

    EOC = ((EOC.z<0)?(-EOC):EOC)/norm(EOC);
	// cout<<EOC.x<<','<<EOC.y<<','<<EOC.z<<endl;
}


int cornea_calib::solve_cornea4(void *p, int m, int n, const double *X, double *Errorparameter, int iflag)
{
    cornea_calib* args = (cornea_calib*)p;

    double k = X[0];
	double h = X[1];

    Point3d C = k * args->EOC;
	Point3d G1 = line_sphere_intersection({0,0,0}, args->G1img, C, h);
	Point3d G2 = line_sphere_intersection({0,0,0}, args->G2img, C, h);
	Point3d G3 = line_sphere_intersection({0,0,0}, args->G3img, C, h);
	Point3d G4 = line_sphere_intersection({0,0,0}, args->G4img, C, h);

    /*方程r1 + l1 = 2(n1*l1)*n1的参数*/
	Point3d r1 = -args->G1img;
	Point3d G1L1 = args->L1 - G1;
	double LenG1L1 = norm(G1L1);
	Point3d CG1 = G1 - C;
	double LenCG1 = norm(CG1);

	/*方程r2 + l2 = 2(n2*l2)*n2的参数*/
	Point3d r2 = -args->G2img;
	Point3d G2L2 = args->L2 - G2;
	double LenG2L2 = norm(G2L2);
	Point3d CG2 = G2 - C;
	double LenCG2 = norm(CG2);

	/*方程r2 + l2 = 2(n2*l2)*n2的参数*/
	Point3d r3 = -args->G3img;
	Point3d G3L3 = args->L3 - G3;
	double LenG3L3 = norm(G3L3);
	Point3d CG3 = G3 - C;
	double LenCG3 = norm(CG3);

	/*方程r2 + l2 = 2(n2*l2)*n2的参数*/
	Point3d r4 = -args->G4img;
	Point3d G4L4 = args->L4 - G4;
	double LenG4L4 = norm(G4L4);
	Point3d CG4 = G4 - C;
	double LenCG4 = norm(CG4);

    Point3d Equation1 = r1 + G1L1 / LenG1L1 - 2 * r1.dot(CG1 / LenCG1) * (CG1 / LenCG1);  //r1 + l1 - 2(n1*l1)*n1 = 0
	Point3d Equation2 = r2 + G2L2 / LenG2L2 - 2 * r2.dot(CG2 / LenCG2) * (CG2 / LenCG2);  // r2 + l2 - 2(n2*l2)*n2 = 0
	Point3d Equation3 = r3 + G3L3 / LenG3L3 - 2 * r3.dot(CG3 / LenCG3) * (CG3 / LenCG3);  //r1 + l1 - 2(n1*l1)*n1 = 0
	Point3d Equation4 = r4 + G4L4 / LenG4L4 - 2 * r4.dot(CG4 / LenCG4) * (CG4 / LenCG4);  // r2 + l2 - 2(n2*l2)*n2 = 0

	Errorparameter[0] = abs(Equation1.x); Errorparameter[1] = abs(Equation1.y); Errorparameter[2] = abs(Equation1.z);
	Errorparameter[3] = abs(Equation2.x); Errorparameter[4] = abs(Equation2.y); Errorparameter[5] = abs(Equation2.z);
	Errorparameter[6] = abs(Equation3.x); Errorparameter[7] = abs(Equation3.y); Errorparameter[8] = abs(Equation3.z);
	Errorparameter[9] = abs(Equation4.x); Errorparameter[10] = abs(Equation4.y); Errorparameter[11] = abs(Equation4.z);
	Errorparameter[12] = abs(LenCG1 - h);  Errorparameter[13] = abs(LenCG2 - h);
	Errorparameter[14] = abs(LenCG3 - h);  Errorparameter[15] = abs(LenCG4 - h);
	Errorparameter[16] = Errorparameter[0] + Errorparameter[1] + Errorparameter[2] + Errorparameter[3] + Errorparameter[4] + Errorparameter[5] + Errorparameter[6] + Errorparameter[7] +
		Errorparameter[8] + Errorparameter[9] + Errorparameter[10] + Errorparameter[11] + Errorparameter[12] + Errorparameter[13] + Errorparameter[14] + Errorparameter[15];
		

	return 0;
}

int cornea_calib::solve_cornea2(void *p, int m, int n, const double *X, double *Errorparameter, int iflag)
{
    cornea_calib* args = (cornea_calib*)p;

    double k = X[0];
	double h = X[1];

    Point3d C = k * args->EOC;
	Point3d G3 = line_sphere_intersection({0,0,0}, args->G3img, C, h);
	Point3d G4 = line_sphere_intersection({0,0,0}, args->G4img, C, h);


	/*方程r2 + l2 = 2(n2*l2)*n2的参数*/
	Point3d r3 = -args->G3img;
	Point3d G3L3 = args->L3 - G3;
	double LenG3L3 = norm(G3L3);
	Point3d CG3 = G3 - C;
	double LenCG3 = norm(CG3);

	/*方程r2 + l2 = 2(n2*l2)*n2的参数*/
	Point3d r4 = -args->G4img;
	Point3d G4L4 = args->L4 - G4;
	double LenG4L4 = norm(G4L4);
	Point3d CG4 = G4 - C;
	double LenCG4 = norm(CG4);

	Point3d Equation3 = r3 + G3L3 / LenG3L3 - 2 * r3.dot(CG3 / LenCG3) * (CG3 / LenCG3);  //r1 + l1 - 2(n1*l1)*n1 = 0
	Point3d Equation4 = r4 + G4L4 / LenG4L4 - 2 * r4.dot(CG4 / LenCG4) * (CG4 / LenCG4);  // r2 + l2 - 2(n2*l2)*n2 = 0

	Errorparameter[0] = abs(Equation3.x); Errorparameter[1] = abs(Equation3.y); Errorparameter[2] = abs(Equation3.z);
	Errorparameter[3] = abs(Equation4.x); Errorparameter[4] = abs(Equation4.y); Errorparameter[5] = abs(Equation4.z);
	Errorparameter[6] = abs(LenCG3 - h);  Errorparameter[7] = abs(LenCG4 - h);
	Errorparameter[8] = Errorparameter[0] + Errorparameter[1] + Errorparameter[2] + Errorparameter[3] + 
                Errorparameter[4] + Errorparameter[5] + Errorparameter[6] + Errorparameter[7];
		

	return 0;
}

bool cornea_calib::solve_lsq_cor()
{
	if(mod==4)
	{
		int m=17, n=2;
		int info, lwa= m*n+5*n+m+1, iwa[n];	
		double  x0[n], Errorparameter[m];
		double tol=1e-15, wa[lwa];

		Init4();

		x0[0] = getRandData(30,50);//0~600的随机数
		x0[1] = 7.8;

		info=__cminpack_func__(lmdif1)(solve_cornea4, this, m, n, x0, Errorparameter, tol, iwa, wa, lwa);

		if(x0[0]<35 ||  x0[0]>50)
			return false;
		
		corneaCenter = x0[0] * EOC;
		corneaRadius = x0[1];
		return true;
	}
	else if(mod==2)
	{
		int m=9, n=2;
		int info, lwa= m*n+5*n+m+1, iwa[n];	
		double  x0[n], Errorparameter[m];
		double tol=1e-5, wa[lwa];

		Init2();

		x0[0] = getRandData(0,50);//0~600的随机数
		x0[1] = 7.8;

		info=__cminpack_func__(lmdif1)(solve_cornea2, this, m, n, x0, Errorparameter, tol, iwa, wa, lwa);

		if(x0[0]<30 ||  x0[0]>50)
			return false;
		
		corneaCenter = x0[0] * EOC;
		corneaRadius = x0[1];
		return true;
	}
    else
	{
		return false;
	}
}

Point3d cornea_calib::getCorneaCenter()
{
	return corneaCenter;
}

double cornea_calib::getCorneaRadius()
{
	return corneaRadius;
}