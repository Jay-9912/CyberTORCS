/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "class_Visualization.h"
#include "driver_cruise.h"
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include <ostream>
#include <fstream>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_cruise";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberCruise" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}


/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
//******************************************************//

//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum=0;//sum of direction error(Integration)      //
// Speed Control Variables								     //
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum=0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta=20;	
int delta0 = 10;
//
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag=true;	
bool IsSerialCorner = 0;														//
double offset=0;										//
double Tmp = 0;
double dist = 0;
double tmps = 0;
double serr = 0;
char dataOut[100];
clock_t start, finish;
double duration;
double err=0;
double bl = 4.9;
double bw = 1.92;
double theta = atan2(bw, bl);
double diag = sqrt(bl * bl + bw * bw) / 2;
double L = 0;
double terr = 0;
double wheelbase = 2.74;
double ld = 0;
double el = 0;
double alpha = 0;
double backx = 0.795;
double backy = -1.198;
double miderr = 0;
double cnt = 0;
double dmid = 0;
double lastmid = 0;
double period = 0;
int state = 0;
bool flag2 = true;

//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear);													//
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary,double input);		//
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
//******************************************************************************//

//******************** Global Variables for OpenCV Visualization *******************//
// Shiyi Wu 2020.03.17																//
// Under Development																//
cls_VISUAL cls_visual;																//
int nKey = 0;																		//
char cKeyName[512];																	//
//**********************************************************************************//
static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}
static double min(double x, double y, double z)
{
	if (x <= y && x <= z)
		return x;
	else if( y <= z)
		return y;
	else return z;
}


static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	if(parameterSet==false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-  
		*/
		//startPoint = constrain(0,200,_speed * 0.445-5);
		startPoint = _speed * _speed * 0.0013+constrain(0,80,2*(_speed-275))+c.r/50;//10
		int sp_far = 2*startPoint;
		dist = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
		L = diag * sin(theta + abs(_yaw)) + dist + bw / 2;
		terr=(L - bw);
		err += terr/bl;
		c = getR(_midline[startPoint][0],_midline[startPoint][1],_midline[startPoint+delta][0],_midline[startPoint+delta][1],_midline[startPoint+2*delta][0],_midline[startPoint+2*delta][1]);
		if (c.r < 120)
		{
			delta = 12;
		}
		c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
		//中距离点
		circle c1;
		c1= getR(_midline[sp_far][0], _midline[sp_far][1], _midline[sp_far + delta][0], _midline[sp_far + delta][1], _midline[sp_far + 2 * delta][0], _midline[sp_far + 2 * delta][1]);
		//远点
		circle c0;
		c0= getR(_midline[0][0], _midline[0][1], _midline[delta][0], _midline[delta][1], _midline[2 * delta][0], _midline[2 * delta][1]);
		//近点
		double min_r = min(c0.r, c1.r, c.r);
		double km_d = 60;
		double km_p = 3;
		double km_i = 0.17;
		double kb_s = 0.02;
		bool flag = true;
		double kp_y = 0.2;
		kp_d = 3;//4
		ki_d = 0;//0.05
		kd_d = 0.01;//0.01
		if (c0.r <= 25)//25
		{

			kp_d = 1.5;//1.5
			kp_y = 1;//1
			km_p = 11.0;
			km_d = 80;
		/*
		}
		if (min_r <= 25)
		{*/
			expectedSpeed = constrain(30, 50, 40 / 25.0 * (c.r - 0) + 0);//c.r*c.r*(-0.046)+c.r*5.3-59.66);//30,50
		}
		else if (min_r <= 120)
		{
			expectedSpeed = constrain(60,100, (40)/95.0*(c.r-25)+60);
			//expectedSpeed = constrain(95, 140, c.r * c.r * (-0.046) + c.r * 5.3 - 59.66);
		}
		
		else if (min_r <= 200)//150
		{
			expectedSpeed = constrain(100, 145 , (c.r-120)/80.0*45+100); //110,230 1.5+10 link speed with D_err
		}
		else 
		{
			expectedSpeed = constrain(145, 175, (c.r-200)/300.0*45+145);//180 300 3.5
		}

		curSpeedErr = expectedSpeed - _speed;
		serr = curSpeedErr - tmps;// add a defferential term
		speedErrSum = 0.1 * speedErrSum + curSpeedErr; // weaken the influence of integral term
		tmps = curSpeedErr;

		if (abs(_midline[0][0]) < 0.005 && abs(_yaw) < 0.005&&abs(_yawrate)<0.1)
		{
			km_p = 1;
			km_d = 5;
		}
		if (_speed > 150 && c1.r == 500 && c.r==500 && abs(_yaw)<0.05)
		{
			*cmdAcc = 0.7;
			*cmdBrake = 0.0;
			km_p = 1;
			km_d = 5;
			flag = false;
		}
		if (curSpeedErr > 0 && flag==true)
		{
                                                  
			if (_speed < 15)						//起步优化
			{
				*cmdAcc = 1;
				*cmdBrake = 0;
			}
			else if (_speed < 40 &&cnt<300)
			{
				*cmdAcc = 0.7;
				*cmdBrake = 0.0;
			}
			else if (abs(_yawrate) > 0.7)
			{
				*cmdAcc = 0.3-0.25 * abs(*cmdSteer) * abs(*cmdSteer);
				*cmdBrake = 0;

			}
			
			else if (abs(*cmdSteer)<0.1) // help starting
			{
				*cmdAcc = constrain(0.0,1.0,kp_s * curSpeedErr + ki_s * speedErrSum + offset + kd_s * serr); // PID of speed
				*cmdBrake = 0;
			}
			
			
			else if (abs(*cmdSteer)>0.1)
			{
				*cmdAcc = 0.3 - 0.25*abs(*cmdSteer)*abs(*cmdSteer);
				*cmdBrake = 0;
			}
//			else
//			{
//				*cmdAcc = 0.11 + offset;
//				*cmdBrake = 0;
//			}
		
		}
		else if (curSpeedErr < 0 &&flag==true)
		{
			if (abs(*cmdSteer) > 0.1 || abs(_yawrate)>1)
			{
				*cmdAcc = 0;
				*cmdBrake = 0.1;
			}
			else
			{ 
				*cmdBrake = constrain(0.0,1.0,-kb_s *curSpeedErr/3 - offset/3+0.12 );//4 0.02
				*cmdAcc = 0;
			}

		}

		updateGear(cmdGear);
		
		/******************************************Modified by Yuan Wei********************************************/
		/*
		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
		Once you choose the error model, you can rectify the value of PID to improve your control performance.
		Enjoy  -_-  
		*/
		// Direction Control		
		//set the param of PID controller


		//std::fstream file;
		//FILE *fp;
		/*
		FILE* pFile = fopen("data.txt", "a");
		finish = clock();
		//duration = (double)(finish - start) / CLOCKS_PER_SEC;
		sprintf(dataOut, "%f,%f,%f,%f\n",*cmdAcc, _acc, c.r,*cmdBrake);

		fputs(dataOut, pFile);    // 写入一个字符串到文件
		fclose(pFile);     // 关闭文件
		*/
		//get the error 
		//int obj =0.7 * startPoint + 3*_width-30;// take current speed into account

		int obj = 0.00008* _speed * _speed +8; //12*_width-133;//13
		D_err = -atan2(_midline[obj][0],_midline[obj][1]) + 0.25 * _yaw;//only track the aiming point on the middle line
		int aim = obj;
		if (cnt<500)
			cnt++;
		//the differential and integral operation 
		D_errDiff = D_err - Tmp;
		D_errSum = 0.2*D_errSum + D_err;
		Tmp = D_err;
		if (cnt > 300||abs(_midline[0][0])<0.05)
		{
			miderr += _midline[0][0];
		}
		dmid = _midline[0][0] - lastmid;
		lastmid = _midline[0][0];
		el = _midline[aim][0];
		ld = sqrt(el * el + pow(_midline[aim][1] - backy, 2));
		alpha = asin(el / ld);
		double del = atan2(2.0 * wheelbase * sin(alpha)/ld, 1.0);//Pure Pursuit

		//set the error and get the cmdSteer
	
		//*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff - 1* _midline[0][0] + 0.2 * _yaw);//1.1 0.01 associate steer with yaw.   PID of D_err
		if (cnt > 300)
		{
				
			*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff + kp_y * _yaw - 0.3 * del - km_p * _midline[0][0] - km_i * miderr - km_d * dmid-10*constrain(0,0.1,(abs(_yawrate)-0.9))*_yawrate);// -0.1 * miderr - 40 * dmid);//1 0.1 40
		}
		else
		{
			if (flag2)
			{
				if (abs(_midline[0][0]) < 3)
					state = 1;
				else if (abs(_midline[0][0]) < 4.5)
					state = 2;
				else state = 3;
				flag2 = false;
			}
			if (state == 1)
				*cmdSteer = constrain(-1.0, 1.0, 5.0 / 3.0 * kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff + 0.2 * _yaw - 1 * del - 1.3 * _midline[0][0] - 0.13 * miderr - 4 * dmid);
			else if (state == 2)
				*cmdSteer = constrain(-1.0, 1.0, 5.0 / 3.0 * kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff + 0.2 * _yaw - 1 * del - 1.4 * _midline[0][0] - 0.13 * miderr - 4.2 * dmid);
			else if (state == 3)
				*cmdSteer = constrain(-1.0, 1.0, 5.0 / 3.0 * kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff + 0.2 * _yaw - 1 * del - 1.3 * _midline[0][0] - 0.13 * miderr - 8 * dmid);//1 8
		}

#pragma region Wu
		cv::Mat im1Src = cv::Mat::zeros(cv::Size(400, 400), CV_8UC1);
		
		for (int i = 0; i < 200; i++)
			cv::circle(im1Src, cv::Point(200 + _midline[i][0] * 2, 400 - _midline[i][1] * 2), 2, cv::Scalar(100, 100, 100));
		sprintf_s(cKeyName, "Key: %c is pushed", nKey);
		cv::putText(im1Src, cKeyName, cv::Point(20, 50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 255));
		cv::imshow("Path", im1Src);
		cls_visual.Fig1Y(5, -10, 10, 10, "CurrentV", _speed/20.0, "expect", expectedSpeed/20.0, "miderr", miderr);
		cls_visual.Fig2Y(3, -1, 1, -1, 1, 10, "yaw", _yaw, "midline[0][0]", _midline[0][0], "terr", terr);

		int tempKey = cv::waitKey(1);
		if (tempKey != -1)
			nKey = tempKey;
#pragma endregion

		//print some useful info on the terminal
		if (cnt==1)
			printf("err : %f \n", _midline[0][0]);
		/*
		printf("err : %f \n", err);
		printf("terr: %f \n", terr);
		printf("cmdSteer %f \n", *cmdSteer);
		printf("radius: %f \n", c.r);
		printf("del: %f \n", del);
		printf("objspeed: %f \n", expectedSpeed);
		printf("acc: %f \n", *cmdAcc);
		printf("speed: %f \n", _speed);
		printf("dist: %f %f %f\n", _midline[0][0],_midline[0][1],dist);
		printf("err: %f \n", err);
		printf("yawrate %f \n",_yawrate );
		*/
		/******************************************End by Yuan Wei********************************************/
	}
}

void PIDParamSetter()
{
		start = clock();
		kp_s=0.02;//0.02
		ki_s=0;//0.02
		kd_s=0.02;//0.02
		kp_d=1.35;
		ki_d=0.151;
		kd_d=0.10;
		parameterSet = true;
	
}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >=105 && topGear >2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear >3)
		{
			*cmdGear = 4;
		}
		else 
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear >4)
		{
			*cmdGear = 5;
		}
		else 
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear >5)
		{
			*cmdGear = 6;
		}
		else 
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else 
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary,double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
/*bool IsSerialCorner()
{
	int i = 10;
	circle tmp0= getR(_midline[i][0], _midline[i][1], _midline[i + delta][0], _midline[i + delta][1], _midline[i + 2 * delta][0], _midline[i + 2 * delta][1]);
	circle tmp;
	for (i = 10;i < 100;++i)
	{
		tmp = getR(_midline[i][0], _midline[i][1], _midline[i + delta][0], _midline[i + delta][1], _midline[i + 2 * delta][0], _midline[i + 2 * delta][1]);
		if (tmp.sign != tmp0.sign)
		{
			return 1;
		}
		tmp0 = tmp;

	}
	return 0;

}
*/



circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a,b,c,d,e,f;
    double r,x,y;
	
	a=2*(x2-x1);
    b=2*(y2-y1);
    c=x2*x2+y2*y2-x1*x1-y1*y1;
    d=2*(x3-x2);
    e=2*(y3-y2);
    f=x3*x3+y3*y3-x2*x2-y2*y2;
    x=(b*f-e*c)/(b*d-e*a);
    y=(d*c-a*f)/(b*d-e*a);
    r=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
	x=constrain(-1000.0,1000.0,x);
	y=constrain(-1000.0,1000.0,y);
	r=constrain(1.0,500.0,r);
	int sign = (x>0)?1:-1;
	circle tmp = {r,sign};
	return tmp;
}
//circle getpointR(int Point)
//{
//	return getR(_midline[Point][0], _midline[Point][1], _midline[Point + delta][0], _midline[Point + delta][1], _midline[Point + 2 * delta][0], _midline[Point + 2 * delta][1]);
//}
