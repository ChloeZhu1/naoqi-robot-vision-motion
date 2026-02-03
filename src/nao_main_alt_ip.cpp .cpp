/**
* Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
*
* \file movehead.cpp
* \brief Move NAO's head.
*
* A simple example showing how to move NAO's head by using ALMotionProxy.
* This example will make NAO turn its head left and right slowly.
* We use here a specialized proxy to ALMotion.
*/
#include <alvalue/alvalue.h>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <althread/alcriticalsection.h>
#include <boost/shared_ptr.hpp>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <althread/almutex.h>
#include <alproxies/altexttospeechproxy.h>
#include <althread/almutex.h>
#include <alerror/alerror.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <qi/log.hpp>

////
#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alredballdetectionproxy.h>
//#include <alproxies/alnotificationmanagerproxy.h>
#include <process.h>

#include <alproxies/alvisualcompassproxy.h>

#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alvision/alimage.h>


#include <alproxies/allandmarkdetectionproxy.h>

#include <almath/types/altransform.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <cmath>
#include<vector>

using namespace AL::Math;
using namespace std;
using namespace AL;
using namespace cv;

#define pi 3.1415926
#define dx 0.20  //the distance of hitting
#define  robotIP  "192.168.1.103"

AL::ALValue MovConfig, MovConfig_triangle ;
float allballdata[7]={0};//save all the data of redball detection
float landmark[5] ={0}; //save data of landMark detection
float YellowAngle=0; 

int ROBOT_STATE=1;
#define SEARCH_RED_BALL	  1
#define FIND_LANDMARK     2
#define FIND_YELLOW       3
#define CREATE_TRIANGLE	  4
#define YELLOW_TRIANGLE   5
#define HIT_BALL		  6
#define NO_LANDMARK_YELLOW 7


AL::ALMemoryProxy memoryProxy(robotIP, 9559);    
AL::ALMotionProxy motionProxy(robotIP, 9559);    
AL::ALRobotPostureProxy postureProxy(robotIP, 9559);
AL::ALTextToSpeechProxy tts(robotIP, 9559);
AL::ALVideoDeviceProxy cameraProxy(robotIP, 9559);
AL::ALVisualCompassProxy compassProxy(robotIP,9559);
AL::ALLandMarkDetectionProxy landmarkProxy(robotIP, 9559);

//declare subfunctions
void onTrackbar_changed(int, void*);
void redball_detect( float*);
void VerifyRedBallPresentBeforeHitting( float*);
//void Go_to_Redball(AL::ALMotionProxy,  AL::ALVideoDeviceProxy, float*, float&, AL::ALValue, float&);
void MovetowardsRedBallBeforeHitting( float*);
bool landmark_Find( float*, float*);
void TriangleCalculation(float*, float*, float*);
void AdjustPosition( float*, float*, float*);
void openhand();
void closehand();
void begin_puthand();
void openclose();
void  HitBall( float , float);
void InitRobotParams(AL::ALValue*,  AL::ALValue *);   //InitRobotParams( &MovConfig,&MovConfig_triangle);
int  PrepareRobot();
void Kick_off( );
void DirectHit();
void TRIANGLE_MAKING(float *allballdata,  float *landmark );
void CloseRedBallSearching( float *allballdata);
void LandMarkSearching(float *allballdata );
float YellowStickDetect_SunUp( );
void STICK_RED_Triangle(float *allballdata  );
float YellowStickSearching();
void  Task_two();
void  Tak_one();
namespace
{
    // windows and trackbars name
    const std::string windowName = "GREEN";
	const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string accumulatorThresholdTrackbarName = "Accumulator Threshold";
    const std::string usage = "Usage : tutorial_HoughCircle_Demo <path_to_input_image>\n";
    const std::string window_name = "BLUE ";
    const std::string window_red = "RED";
    // initial and max values of the parameters of interests.
    const int cannyThresholdInitialValue = 200;
    const int accumulatorThresholdInitialValue = 16;
    const int maxAccumulatorThreshold = 200;
    const int maxCannyThreshold = 255;	
   

	
}
//Global variable for hsv color wheel plot
//RNG rng(12345);//黄杆
int max_hue_range = 179;
int max_step = 3; //nuber of pixel for each hue color
int wheel_width = max_hue_range*max_step;
int wheel_hight = 50;
int wheel_x = 50; //x-position of wheel
int wheel_y = 5;//y-position of wheel

Mat HSV;
#define HUEMAX 179
#define SATMAX 255
#define VALMAX 255
int MAX_H = 179;
int MAX_S = 255;
int MAX_V = 255;
int mouse_x = 0;
//Global variable plot for satuarion-value plot
int S_V_Width = MAX_S;
int S_V_Height = MAX_S;
int S_V_x = 10;
int S_V_y = wheel_y + wheel_hight + 20;
//Global variable for HSV ploat
int HSV_Width = 150;
int HSV_Height = 150;
int HSV_x = S_V_x + S_V_Width + 30;
int HSV_y = S_V_y + 50;
void onTrackbar_changed(int, void*){
 //Plot color wheel.
 int hue_range = 0;
 int step = 1;
 for (int i = wheel_y; i < wheel_hight + wheel_y; i++){
  hue_range = 0;
  for (int j = wheel_x; j < wheel_width + wheel_x; j++){
   if (hue_range >= max_hue_range) hue_range = 0;
   if (step++ == max_step){
    hue_range++;
    step = 1;
   }
Vec3b pix;
   pix.val[0] = hue_range;
   pix.val[1] = 255;
   pix.val[2] = 255;

   //HSV.at<Vec3b>(i, j) = pix;
  }
 }
}



int main(int argc, char* argv[]) {
	  
	 float headTouchedButtonFlag=0;
	  while( headTouchedButtonFlag==0)
	  {
          headTouchedButtonFlag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value");	
	  }
	// initialize robot gait params.
	InitRobotParams( &MovConfig,&MovConfig_triangle);
	
 	


try{

	  float headTouchedButtonFlag=0;
	  while( headTouchedButtonFlag==0)
	  {
          headTouchedButtonFlag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value");	
	  }
	postureProxy.goToPosture("StandInit", 0.3);
	motionProxy.moveInit();
	Sleep(1000);
	motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
	//motionProxy.angleInterpolationWithSpeed("HeadPitch",0.373, 0.2f);//保持低头21.4度:0.373
	Sleep(1000);
	redball_detect(allballdata);
	/*if(*(allballdata+4)==0)
		{
			CloseRedBallSearching(allballdata);//近处红球寻找球
			if(*(allballdata+4)==0)
				VerifyRedBallPresentBeforeHitting( allballdata);
			//state=three; 
		 	//break;
		}
	else
		tts.say("seen ");
	//CloseRedBallSearching(allballdata);*/
	VerifyRedBallPresentBeforeHitting( allballdata);

	// }

  //   float Angle=0;
//	Angle= YellowStickSearching();
 //   motionProxy.setMoveArmsEnabled(false, false);
//	std::cout<<"Angle:"<<Angle;
//	motionProxy.moveTo(0,0,Angle,MovConfig_triangle);	
//	 STICK_RED_Triangle(allballdata );
//	 CloseRedBallSearching(allballdata);
//	VerifyRedBallPresentBeforeHitting( allballdata);
	 
	  while(1);

   
}
catch (const AL::ALError& e) {
	std::cerr << "Caught exception: " << e.what() << std::endl;
	exit(1);
	}
	exit(0);

/***********************************/
/*****************/
  if (PrepareRobot()){

	   //  Kick_off();//start to hit the ball
		// Task_0ne();
		// Task_two();
	    // Task_three();
		try {
   

			  //Init the robot
			  //Wake up
			  //Take the club
			  //Kick off
			  //Verify the that the ball is there before hitting
			  //While the ball is not there, search for the ball.
			  //go to Loop
	
			  //After hitting the ball
			  //Loop
			  //Search for the ball
			  //Head to the ball
			  //Search for the landmark
			  //1 Landmark found
			  //2 landmark not found
			  //case 1
			  // if the robot goes towards the ball and position in order to hit the ball
			  //verify that the ball is there before hitting
			  // case 2
			  // estimation of the robot's position
			  //derive the landmark position
			  //The robot goes to th red ball
			  //Hit the ball according to the landmark estimation  
			  //Go to loop
	  
			  //Kick_off();

	

				//End initialize the robot
				bool landmark_Found =false;
				bool END_GAME=false;
			  
		
				while(!END_GAME){
					switch(ROBOT_STATE){
						case SEARCH_RED_BALL:
							VerifyRedBallPresentBeforeHitting( allballdata);
							ROBOT_STATE++;
							break;
						case FIND_LANDMARK:
					        landmark_Found = landmark_Find( landmark, allballdata);
							if(landmark_Found == true)
								ROBOT_STATE=CREATE_TRIANGLE;
							else  
							    ROBOT_STATE=FIND_YELLOW;
							break;
				
						case FIND_YELLOW:
						
							YellowAngle=YellowStickSearching();
							if(YellowAngle == 0)
							{  
							    ROBOT_STATE=NO_LANDMARK_YELLOW;
								break;
							}
							else
								 ROBOT_STATE=YELLOW_TRIANGLE;
							break;


						case CREATE_TRIANGLE:
						   TRIANGLE_MAKING(allballdata,  landmark );
						   ROBOT_STATE=HIT_BALL;
							break;

						case YELLOW_TRIANGLE:
							STICK_RED_Triangle( allballdata );
							ROBOT_STATE=HIT_BALL;
							break;

						case HIT_BALL:
							HitBall( 2.5, 0.25f);
							motionProxy.setMoveArmsEnabled(true, false);
							motionProxy.moveTo(0,0,1.57f,MovConfig);
							motionProxy.moveTo(0.35,0,0,MovConfig);
							motionProxy.moveTo(0.35,0,0,MovConfig);
							ROBOT_STATE=SEARCH_RED_BALL;
							break;

						case NO_LANDMARK_YELLOW:
							MovetowardsRedBallBeforeHitting(allballdata);
							ROBOT_STATE=HIT_BALL;
							break;

					}


				}

		  }

		  catch (const AL::ALError& e) {
		   std::cerr << "Caught exception: " << e.what() << std::endl;
			exit(1);
		  }
			exit(0);
	
	}
/***********************************/

}


//define subfunctions
/****Initialize Robot Gait Params*****/
void InitRobotParams(AL::ALValue *MovConfig,AL::ALValue *MovConfig_triangle)
{
	
	//set gait parameters
	
    AL::ALValue one  = AL::ALValue::array("MaxStepX", 0.037); //0.001~0.080, Default:0.040
	//MaxStepX: Better use 0.080 meters only when walking on flat hard floors.
	AL::ALValue two  = AL::ALValue::array("MaxStepY", 0.14); //0.101~0.160, Default:0.140
	AL::ALValue three= AL::ALValue::array("MaxStepTheta", 0.1); //0.001~0.524, Default:0.349
	AL::ALValue four = AL::ALValue::array("MaxStepFrequency", 0.85);//0~1, Default:1
    AL::ALValue five = AL::ALValue::array("StepHeight",0.013); //0.005~0.040, Default:0.020
	AL::ALValue six  = AL::ALValue::array("TorsoWx",0.00); //-0.122~0.122, Default:0.000
	AL::ALValue seven  = AL::ALValue::array("TorsoWy",0.00); //-0.122~0.122, Default:0.000
	
	AL::ALValue moveConfig=AL::ALValue::array(one,two,three,four,five,six,seven);
	*MovConfig=moveConfig;

	 one  = AL::ALValue::array("MaxStepX", 0.028); //0.001~0.080, Default:0.040
	//MaxStepX: Better use 0.080 meters only when walking on flat hard floors.
	 two  = AL::ALValue::array("MaxStepY", 0.12); //0.101~0.160, Default:0.140
	 three= AL::ALValue::array("MaxStepTheta", 0.10); //50.001~0.524, Default:0.349  //0.12
	 four = AL::ALValue::array("MaxStepFrequency", 0.85);//0~1, Default:1
     five = AL::ALValue::array("StepHeight",0.01); //0.005~0.040, Default:0.020
	 six  = AL::ALValue::array("TorsoWx",0.00); //-0.122~0.122, Default:0.000
	 seven  = AL::ALValue::array("TorsoWy",0.00); //-0.122~0.122, Default:0.000

	AL::ALValue moveConfig_triangle=AL::ALValue::array(one,two,three,four,five,six,seven);

	*MovConfig_triangle=moveConfig_triangle;

}
/***********************************/
int PrepareRobot()
{
	try {
		//Initialize the robot
		motionProxy.wakeUp();
		postureProxy.goToPosture("StandInit", 0.2);
		Sleep(2000);
		motionProxy.moveInit();

		//openhand();//ask for club
		//closehand();//hold the club
		//openclose();//open hand
		//begin_puthand();//put right hand down

		return 1;
    }

	catch (const AL::ALError& e) {
		std::cerr << "Caught exception: " << e.what() << std::endl;
		exit(1);
	}
}
/*******************/
void Kick_off()
{
	
		//HitBall( 2.5, 0.25f);
		float angle;
		//motionProxy.setMoveArmsEnabled(true, false);
		//motionProxy.moveTo(0.0, 0.0, 1.57, MovConfig);
		//motionProxy.moveTo(0.4, 0.0, 0.0, MovConfig);
		//motionProxy.moveTo(0.4, 0.0, 0.0, MovConfig);

       redball_detect(allballdata); //callback "redball_detect" function
		if(*(allballdata+4)==0)
			{
				//motionProxy.moveTo(0.3, 0.0, 0.0, MovConfig);
				VerifyRedBallPresentBeforeHitting( allballdata);  //检测红球
		    }
		else
		{
			tts.say("I see the ball");
			angle=*allballdata;
			motionProxy.moveTo(0,0,angle,MovConfig);
			motionProxy.moveTo(*(allballdata+6)-0.30,0,0,MovConfig);

		}
		
		
		
}

/********Detect red_ball*****/
void redball_detect( float *allballdata)
{
      
//use to test
	std::string names  = "Head";
    float stiffnessLists  = 1.0f;
    float timeLists  = 1.0f;
    motionProxy.stiffnessInterpolation(names, stiffnessLists, timeLists); 

 Mat src, src_gray, frameHSV, channel[3], frameHSV_BW, channel_HSV[3];
 Mat pass[3];
 Mat frame, frameCopy, image;
	 int cameraId = 1;//0:camera_up，1:camrea_bottom
	 cameraProxy.setActiveCamera(cameraId);
	 std::string clientName = "test";
	 clientName = cameraProxy.subscribe(clientName, kVGA, kBGRColorSpace, 1);//fps=5

	 cv::Mat imgHeader = cv::Mat(cv::Size(640, 480),CV_8UC3); 
	 

 int Hm = 7;//白天6 晚上7
 
 //declare and initialize both parameters that are subjects to change
 int cannyThreshold = cannyThresholdInitialValue;
 int accumulatorThreshold = accumulatorThresholdInitialValue;
 // create the main window, and attach the trackbars
// namedWindow(windowName, WINDOW_AUTOSIZE);
// createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold, maxCannyThreshold);
 //createTrackbar(accumulatorThresholdTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);
// namedWindow(window_name);
 //createTrackbar("Hue Min", window_name, &Hm, HUEMAX, onTrackbar_changed);
 //HSV.create(640, 480, CV_8UC3); //Mat to store clock image
// HSV.setTo(Scalar(200, 0, 200));
 onTrackbar_changed(0, 0); //initialoze window
 
 int redballFlag = 0;//the flag of having detected red ball
 int   loseBallTimes = 0;
 float m_center_x= 0;
 float m_center_y= 0;
 float redballradius=0.02f;
 float theta1=0;
 float theta2=0;
 float d1_a=0;
float d2_a=0;
float d1=0;
float d2=0;
float Xw2=0;
float Yw2=0;
float Zw=0;
float Dis_move=0;

//float fx=558.8f; //437.2f 
//float fy=561.7f; //414.7f
float CameraHeight=0;
float CameraX=0;
float CameraY=0;
 for(int i=0; i<3; i++)
  {	  
	  std::vector<float>result = motionProxy.getPosition("CameraBottom",2,true);//bottom
	   CameraHeight = result.at(2);
	   CameraX = result.at(0);
	   CameraY = result.at(1);
	  std::vector<float> Rotation1 = motionProxy.getAngles("HeadPitch", true);
	  float pitch=Rotation1.at(0);
	   std::vector<float> Rotation2 = motionProxy.getAngles("HeadYaw", true);
	  float yaw=Rotation2.at(0);

    AL::ALValue img= cameraProxy.getImageRemote(clientName);
	while(!img.getSize())
	{  
		std::cout<<"img is empty"<<std::endl;
		//cameraProxy.unsubscribe(clientName);//cancle subscribing to camera
		//clientName = cameraProxy.subscribe(clientName, kVGA, kBGRColorSpace, 5);//fps=5,resubscribe
	   img= cameraProxy.getImageRemote(clientName);
	}
    imgHeader.data = (uchar*)img[6].GetBinary(); //error  
     Mat frame = imgHeader.clone();  //Mat
	
  
   //split the channels in order to manrobotIPulate them
   split(frame, channel);
   //threshold(channel[2], channel[2], 130, 255, 0);
   
   //To get the RED only
   channel[0] = channel[0].mul(.1*Hm); //B
   channel[1] = channel[1].mul(.1*Hm);//G
   channel[2] = channel[2] - channel[0] - channel[1]; //R
    //channel[2] =3* channel[2] ;
 /*  

   //To get the BLUE only
  channel[1] = channel[1].mul(.1*Hm);
   channel[2] = channel[2].mul(.1*Hm);
   channel[0] = channel[0] - channel[1] - channel[2]; 
   */
  double minVal, maxVal;
  // int x, y;
   minMaxLoc(channel[0], &minVal, &maxVal);
   for (int i = 0; i < channel[1].cols; i++)
   {
    for (int j = 0; j < channel[1].rows; j++){
     if (channel[2].at<unsigned char>(j, i)  < 0)
      channel[2].at<unsigned char>(j, i) = 0;
    }
   }
   
   //printf("%f", maxVal);
  // imshow(window_red, channel[0]);
 // imshow(windowName, channel[1]);
 // imshow(window_name, channel[2]);

  
    cannyThreshold = std::max(cannyThreshold, 1);
   accumulatorThreshold = std::max(accumulatorThreshold, 1);
        std::vector<Vec3f> circles;
        // runs the actual detection
        HoughCircles( channel[2], circles, CV_HOUGH_GRADIENT, 1, frame.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );


     redballFlag = circles.size();//flag of finding the ball
	 
	 redballradius=0.025f;
     for( size_t i = 0; i < circles.size(); i++ )
        {
           Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
           int radius = cvRound(circles[i][2]);
            circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
           circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
			//std::cout<<"Center_X:"<<circles[i][0]<<std::endl;
			//std::cout<<"Center_Y:"<<circles[i][1]<<std::endl;
			 m_center_x= circles[i][0];
	         m_center_y= circles[i][1];
             

			d1_a=(m_center_y-240.0f)*47.64f/480.0f*0.01744f;
			d1=(CameraHeight-redballradius)/tan(0.6929f+pitch+d1_a);//对比
			d2_a=(320.0f-m_center_x)*60.92f/640.0f*0.01744f;
			d2=d1/cos(d2_a);
			Xw2=d2*cos(d2_a+yaw)+CameraX;

			Yw2=d2*sin(d2_a+yaw)+CameraY;
			//Yw2=((CameraHeight-redballradius)*theta1)/(sin(0.6929f+pitch-atan(theta2))*std::sqrt(1+theta2*theta2))+CameraY;
			Zw=redballradius;
			Dis_move=sqrt(Xw2*Xw2+Yw2*Yw2);
		//   float ballToRobot=std::sqrt( Xw*Xw + Yw*Yw +Zw*Zw);
	          
        }
	 if(redballFlag==1)
		 break;   
        //imshow("original image: " , frame);
  }
    
    cameraProxy.unsubscribe(clientName);//cancle subscribing to camera device

    *allballdata = d2_a; //centerX
	*(allballdata+1) = d1_a; //centerY
	*(allballdata+2) = Xw2;  //composition distance in X & Y axis of robot
	*(allballdata+3) = Yw2;  //distance in X axis of robot
	*(allballdata+4) = redballFlag;
	*(allballdata+5) = CameraHeight;
	*(allballdata+6) = Dis_move;

//display for test:
	std::cout<< "...... "<<std::endl;
	std::cout<< "以下为redball检测数据："<<std::endl;
	std::cout<< "wzCamera: "<<d1_a<<std::endl;
	std::cout<< "wyCamera: "<<d2_a<<std::endl;
	std::cout<< "Distance_X: "<<Xw2<<std::endl;
	std::cout<< "Distance_Y: "<<Yw2<<std::endl;
	std::cout<< "Distance_move: "<<Dis_move<<std::endl;
	std::cout<< "redball_Flag: "<<redballFlag<<std::endl;
	std::cout<< "camera_Height: "<<CameraHeight<<std::endl;
	std::cout<< "...... "<<std::endl;
}


/********************寻找红球&走向红球*****************************/
void VerifyRedBallPresentBeforeHitting( float *allballdata)
{
	
	tts.say("red ball searching!");
	motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.00, 0.3f);//头下低21.4度:0.373
	motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.00, 0.2f);
	
	
	float head_yaw =0;
	int redball_Flag = 0;

int SEARCH_STATE=1;
#define STRAIGHT_FORWARD 1
#define STRAIGHT_LEFT    2
#define STRAIGHT_RIGHT   3
#define DOWN_RIGHT       4
#define DOWN_FORWARD     5
#define DOWN_LEFT        6
#define MOVE             7
	
int MOVE_STATE  =0;
#define MOVE_FORWARD     1
#define MOVE_TO_RIGHT    2
#define MOVE_TO_LEFT     3
#define MOVE_TO_RF       4

redball_detect(allballdata);

while(*(allballdata+4) != 1)
{


	switch(SEARCH_STATE)
	{
	     case STRAIGHT_FORWARD:
				//look straight                           往正前方看
				std::cout<<"look straight forward..."<<std::endl;
				motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.00f, 0.3f);//Interpolation插补 up-down,speed 
				motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.00f, 0.2f);//left-right
				//start to detect redball
				redball_detect(allballdata); //callback "redball_detect" function
				head_yaw = 0.00;  
				if(*(allballdata+4) != 1){
					SEARCH_STATE++;
				}
				break;

		 case STRAIGHT_LEFT:                      
			      //look at left                        往左方看（头部向左偏转45度--偏航角yaw）
				 std::cout<<"look straight on left..."<<std::endl;
				 motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.00f, 0.3f);
				 motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.785f, 0.2f);// pi/4, assuming counterclockwise is postive
				 //start to detect redball
				 redball_detect(allballdata); //callback "redball_detect" function                                            
				 head_yaw = 0.785;                     //输出偏航角45度
				 if(*(allballdata+4) !=1){
					SEARCH_STATE++;
				}
				break;

		 case STRAIGHT_RIGHT: 
			     //look at right                     往右方看（头部向右偏转45度--偏航角yaw）
				std::cout<<"look straight on right..."<<std::endl;
				motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.00f, 0.3f);
				motionProxy.angleInterpolationWithSpeed("HeadYaw", -0.785f, 0.2f);
				//start to detect redball
				redball_detect( allballdata); //callback "redball_detect" function
				head_yaw = -0.785;
				if(*(allballdata+4) != 1){
					SEARCH_STATE++;
				}
				break;

		 case DOWN_RIGHT:
			    //look down right                    往右下方看（头部向右偏转45度--偏航角yaw，向下偏转20度pitch）
				std::cout<<"look down right...:"<<std::endl;
				motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.31f, 0.3f);
				motionProxy.angleInterpolationWithSpeed("HeadYaw", -0.785f, 0.2f);
				//start to detect redball
				redball_detect(allballdata); //callback "redball_detect" function
				head_yaw = -0.785;  
				if(*(allballdata+4) != 1){
					SEARCH_STATE++;
				}
				break;

		 case DOWN_FORWARD:
			    //look down forward                    往正下方看（向下偏转20度pitch）
				std::cout<<"look down forward..."<<std::endl;
				motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.31f, 0.3f);
				motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.00f, 0.2f);
				//start to detect redball
				redball_detect(allballdata); //callback "redball_detect" function
				head_yaw = 0.00;  
				if(*(allballdata+4) != 1){
					SEARCH_STATE++;
				}
				break;

		 case DOWN_LEFT:     
			    //look down left                    往左下方看（头部向右偏转45度--偏航角yaw，向下偏转20度pitch）
				std::cout<<"look down left..."<<std::endl;
				motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.31f, 0.3f);
				motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.785f, 0.2f);
				//start to detect redball
				redball_detect(allballdata); //callback "redball_detect" function
				head_yaw = 0.785;  
				if(*(allballdata+4) != 1){
					tts.say("red ball is not insight.");
					std::cout<<"there is no redball insight..."<<std::endl;
					SEARCH_STATE++;
					MOVE_STATE++;
				}
				break;
         
		 case MOVE:
			 //there is no redball insight, robot change its position
			 switch(MOVE_STATE){
			      case MOVE_FORWARD:
					    //move ahead
						std::cout<<"move ahead..."<<std::endl;
						motionProxy.moveTo(0.2, 0.0, 0.0,MovConfig);
						SEARCH_STATE=1;
						break;

				  case MOVE_TO_RIGHT:
					   //move to the right
						std::cout<<"move to the right..."<<std::endl;
						motionProxy.moveTo(0.0, 0, -0.785f,MovConfig);
						SEARCH_STATE=1;
						break;

				  case MOVE_TO_LEFT:
					  //move to the left
						std::cout<<"move to the left..."<<std::endl;
						motionProxy.moveTo(0.0, 0.0, 0.785f,MovConfig);
						SEARCH_STATE=1;
						break;

				  case MOVE_TO_RF:
					  //move to the right front
					    motionProxy.moveTo(0.0, 0.0, -0.53f,MovConfig);
						std::cout<<"move to the right front..."<<std::endl;
						motionProxy.moveTo(0.524, -0.524, 0.0,MovConfig);
						SEARCH_STATE=1;
						break;

				  default:
					    SEARCH_STATE=1;
					    MOVE_STATE=0;

			 
			 }
			 break;

		 default:
			    
				std::cout<<"red ball search error..."<<std::endl;				
	
	}

}
//give the report of redball direction.

    if (head_yaw == 0){
			tts.say("redball is in front !");  //front         	
		    }
    if (head_yaw > 0){
			tts.say("redball is on the left !");  //left   	
		    }
    if (head_yaw < 0){
			tts.say("redball is on the right !");  //right      
		    } 
    //Display all ball data!
    std::cout<< "************"<<std::endl;
    std::cout<< "Redball Data Shown as Follow: "<<std::endl;
	std::cout<< "wzCamera: "<<*allballdata<<std::endl;
	std::cout<< "wyCamera: "<<*(allballdata+1)<<std::endl;
	std::cout<< "Distance_X: "<<*(allballdata+2)<<std::endl;
	std::cout<< "Distance_Y: "<<*(allballdata+3)<<std::endl;
	std::cout<< "Dis_move: "<<*(allballdata+6)<<std::endl;
	std::cout<< "redball_Flag: "<<*(allballdata+4)<<std::endl;
	std::cout<< "camera_Height: "<<*(allballdata+5)<<std::endl;
	std::cout<< "head_yaw: "<<head_yaw<<std::endl;
	std::cout<< "************"<<std::endl;
	

/////////移动至红球///////////
//第一次，机器人转到正对红球的方向
    float theta = head_yaw + *allballdata;
	//angle = theta;//第一个转角保存在angle中

	//机器人头部调正
    motionProxy.setMoveArmsEnabled(true, false);
    motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5);
	motionProxy.angleInterpolationWithSpeed("HeadPitch",0.00, 0.35f);//保持低头21.4度:0.373
	//turn around
	motionProxy.moveTo(0, 0, theta, MovConfig);

    motionProxy.setMoveArmsEnabled(true, false);
//第二次，机器人移动至距离红球40cm距离
	if(*(allballdata+6)>0.4){
	 motionProxy.moveTo(*(allballdata+6)-0.4, 0, 0, MovConfig);}// x,y,theta,confirmation 

//第三次，机器人低头再次检测红球，调整角度
	motionProxy.setMoveArmsEnabled(true, false);
    motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
	motionProxy.angleInterpolationWithSpeed("HeadPitch",0.00, 0.3f);//保持低头21.4度:0.373
	//Sleep(1000);
	motionProxy.waitUntilMoveIsFinished();
	redball_detect(allballdata); //callback "redball_detect" function
    head_yaw = 0.00;
    
	theta = head_yaw + *allballdata;
	//angle += theta;//第二个转角保存在angle中
	motionProxy.setMoveArmsEnabled(true, false);
	motionProxy.moveTo(0, 0, theta, MovConfig);
    //compassProxy.moveStraightTo(*(allballdata+2)-0.3); //向前向后距离矫正，距红球30cm
	motionProxy.moveTo(*(allballdata+6)-0.3, 0, 0,MovConfig); //向前向后距离矫正，距红球30cm
	motionProxy.moveTo(0, 0, 1.57, MovConfig);//pi/2
    motionProxy.moveTo(0.5, 0, 0, MovConfig);
	motionProxy.moveTo(0, 0, -1.57, MovConfig);
	motionProxy.moveTo(0.5, 0, 0, MovConfig);
}

/*********************************/

/********************找不到杆的情况下****************************/
void MovetowardsRedBallBeforeHitting( float *allballdata)
{
	
       motionProxy.setMoveArmsEnabled(true, false);
		tts.say("the landmark is in front");
		float angle=0;
		for(int j=0;j<2 ;j++ )
		{     
			float Distance =0.3f;
			float WALKING_X= Distance - Distance*cos(0.69f);
			float WALKING_Y= Distance*sin(0.69f);
			motionProxy.moveTo(WALKING_X,WALKING_Y,-0.69f,MovConfig_triangle);													
		}
		motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1f);
		motionProxy.angleInterpolationWithSpeed("HeadPitch",0.40, 0.1f);//保持低头21.4度:0.373	
		redball_detect( allballdata); //callback "redball_detect" function	
		if(*(allballdata+4)==0)
			 CloseRedBallSearching(allballdata);
			               
		angle =  *allballdata; 
		motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
		redball_detect( allballdata); //callback "redball_detect" function		
		motionProxy.moveTo(*(allballdata+6)-0.16,0,0,MovConfig_triangle);	
		//redball_detect( allballdata); //callback "redball_detect" function	
		//angle =  *allballdata; 
		//motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	

		motionProxy.moveTo(0,0.07,0,MovConfig_triangle);//侧移	
		
}

/************NaoMark定位 + 机器人与NaoMak距离计算**********************************/

bool landmark_Find( float *landmark, float *allballdata)
 {
	 motionProxy.angleInterpolationWithSpeed("HeadYaw",0.0f, 0.1f);	
	 motionProxy.angleInterpolationWithSpeed("HeadPitch",0.0f, 0.1f);	
   // landmark[]={0};
	float  headYawAngle= 2.0;
//	bool landmarkflag=0;
	int camera_ID = 0;//0:top camrea, 1:bottom
	 cameraProxy.setActiveCamera(camera_ID);

    memoryProxy.subscribeToEvent("LandmarkDetected","ALModule","callback");//subscribe to LandmarkDetected event
	 float wzCamera = 0, wyCamera =0, angularSize =0, distanceFromCameraToLandmark =0;
	 float landmarkTheoreticalSize = 0.10; // in meters
	 float mark_Flag = 0.0f;//0:landMark not detected, 1:landMark detected

			 while( headYawAngle>-1.9 )
			 {
				  Sleep(1000);
				   AL::ALValue markData=memoryProxy.getData( "LandmarkDetected");// This can often be cast transparently into the original type.//getData()
				 //   AL::ALValue markDate=memoryProxy.getData( "LandmarkDetected");
				   float landmarkTheoreticalSize = 0.093f;
	  
	  
				   if( markData.getSize()>=2)
				   {
					   
					//   markData=memoryProxy.getData( "LandmarkDetected");
					// std::cout<<"no"<<std::endl;
					 wzCamera = markData[1][0][0][1]; 
					 wyCamera =  markData[1][0][0][2];
				     angularSize =  markData[1][0][0][3];// Retrieve landmark angular size in radians.

					 distanceFromCameraToLandmark = landmarkTheoreticalSize / ( 2 * tan( angularSize / 2));

					 std::vector<float> sensorAngles = motionProxy.getAngles("HeadYaw", true);
					 std::vector<float>::iterator it;
					  it= sensorAngles.begin();

					  headYawAngle = wzCamera +(*it);//修正摄像头焦点至mark中心

				 //   std::cout<<"distance:"<<distanceFromCameraToLandmark<<std::endl<<"angle:"<<headYawAngle<<std::endl<<"wzCamera:"<<wzCamera<<std::endl;

					// std::cout <<"wzCamera:"<<wzCamera<<std::endl;
					//  std::cout << "Sensor angles: " << std::endl << sensorAngles << std::endl;
					 // landmarkflag=1;
					  tts.say("landmark, i see you!");
					  mark_Flag = 1.0f;
				     break;
				   }
				   headYawAngle-=0.5f;
				   motionProxy.angleInterpolationWithSpeed("HeadYaw",headYawAngle, 0.1f);
				 	
					Sleep(1000);
					tts.say("landmark searching!");
					mark_Flag = 0.0f;
					
			 }
/********************/
//找不到LandMark直接击球（写的比较粗糙）
if(mark_Flag == 0){

	return 0;

}

/************************************/			 
			
	AL::ALValue  currentCamera="CameraTop"; //or CameraBottom
	//Get current camera position in NAO space
	int frame=2;//FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
	bool useSensorValues= true;
	std::vector<float> transform1= motionProxy.getTransform(currentCamera, frame, useSensorValues); //Vector of 16 floats 
	 
	//Create a Transform with an std::vector.
	Transform robotToCamera(transform1);  
	//Compute the rotation to point towards the landmark.
	Transform cameraToLandmarkRotationTransform = robotToCamera.from3DRotation(0, wyCamera, wzCamera);
	 //Compute the translation to reach the landmark.
	Transform cameraToLandmarkTranslationTransform(distanceFromCameraToLandmark, 0, 0);
	//Combine all transformations to get the landmark position in NAO space.
	Transform robotToLandmark =   robotToCamera*cameraToLandmarkRotationTransform*cameraToLandmarkTranslationTransform; 
	//distanceFromCameraToLandmark=sqrt(robotToLandmark.r1_c4 *robotToLandmark.r1_c4 +  robotToLandmark.r2_c4* robotToLandmark.r2_c4);

	 std::vector<float> commandAngles = motionProxy.getAngles("HeadYaw", true);
	 headYawAngle=commandAngles.at(0);

	 *landmark = headYawAngle;
	 *(landmark+1) = distanceFromCameraToLandmark;
	 *(landmark+2) = mark_Flag;
	 *(landmark+3)=robotToLandmark.r1_c4;//x
	 *(landmark+4)= robotToLandmark.r2_c4;//y

	 std::cout<<"......"<<std::endl;
	 std::cout<<"headYawAngle for landMark detection: "<<*landmark<<std::endl;
	 std::cout<<"distance from Camera to Landmark: "<<*(landmark+1)<<std::endl;
	 std::cout<<"......"<<std::endl;
  
	 return 1;
	 //memoryProxy.unsubscribeToEvent("LandmarkDetected","callback");//cancle subscribing to LandmarkDetected event
 }

 
 /*****landmark_地址***
	 *(landmark+3)=robotToLandmark.r1_c4;//x
	 *(landmark+4)= robotToLandmark.r2_c4;//y
    /******/
//----------------------------------- 构造击球直角三角形 (修改)--------------------------------------

void TriangleCalculation(float *calcu_data, float *landmark, float *allballdata)  
{
	//变量初始化
	float s = *(landmark+1);
	float alpha = *landmark; //头部转角，left:alpha>0 , right:alpha<0，构造三角形时判断采取哪种步法
    float x = *(allballdata+6);
	float l2 = 0.00;
	float l  = 0.00;
	float costheta = 0.00;
	float Theta    = 0.00;
	float theta1   = 0.00;
	float turnAngle1 = 0.00;
	float turnAngle2 = 0.00;
	float dis1 = 0.00;
	float dis2 = 0.00;
	//float turndata[4]={0,0,0,0};
	//int *getdata = turndata;

	if (alpha < 0.0) //Mark is on the right of robot
	{
	    alpha = abs(alpha);
        l2 = x*x + s*s - 2*x*s*cos(alpha);
        l = sqrt(l2);
        costheta = (x*x + l2 - s*s)/(2*x*l);
        Theta = acos(costheta);

		if (Theta >= pi/2)
		{
		    theta1 = Theta - pi/2;
            turnAngle1 = - (pi/2 - theta1);
            dis1 = (x*cos(theta1) + 0.1);//左侧步移动dis1距离，修正值0.0
            turnAngle2 = 0;
            dis2 = x*sin(theta1)-0.1; //前进dis2距离
            //#此情况机器人还需绕求顺时针旋转90度		
		}
            
        else if (Theta < pi/2)
		{
		    theta1 = pi/2 - Theta;
            turnAngle1 = Theta;
            dis1 = - (x*sin(Theta) + 0.1);    //向右侧步d1距离,修正值         
            turnAngle2 = 0;
            dis2 = x*cos(Theta)-0.1; //向前移动d2距离
            //#此情况机器人还需绕求逆时针旋转90度
		}
        
		*calcu_data = dis1;
		*(calcu_data+1) = dis2;
		*(calcu_data+2) = turnAngle1;
		*(calcu_data+3) = turnAngle2;
		*(calcu_data+4) = Theta;

        std::cout<<"......"<<std::endl;
		std::cout<<"walk along the first right-angle side: "<<*calcu_data<<std::endl;
		std::cout<<"walk along the second right-angle side: "<<*(calcu_data+1)<<std::endl;
		std::cout<<"the firt Corner: "<<*(calcu_data+2)<<std::endl;
		std::cout<<"the second Corner: "<<*(calcu_data+3)<<std::endl;
		std::cout<<"......"<<std::endl;

	}
        
    
	if (alpha > 0.0) //Mark is on the left of robot
	{
        l2 = x*x + s*s - 2*x*s*cos(alpha);
        l = sqrt(l2);
        costheta = (x*x + l2 - s*s)/(2*x*l);
        Theta = acos(costheta);

        if (Theta >= pi/2)
		{
		    theta1 = Theta - pi/2;
            turnAngle1 = - theta1;
            dis1 = x*sin(theta1) + 0.1; //向左侧步dis1距离，修正值
            turnAngle2 = 0;
            dis2 = x*cos(theta1)-0.1;
		
		}

        else if (Theta < pi/2)
		{
		    theta1 = pi/2 - Theta;
            turnAngle1 = Theta;
            dis1 = - (x*sin(Theta)- 0.1); //向右侧步dis1距离，修正值
            turnAngle2 = 0;
            dis2 = x*cos(Theta)-0.1;
		
		}
               
		*calcu_data = dis1;
		*(calcu_data+1) = dis2;
		*(calcu_data+2) = turnAngle1;
		*(calcu_data+3) = turnAngle2;
		*(calcu_data+4) = Theta;
        
		std::cout<<"......"<<std::endl;
		std::cout<<"walk along the first right-angle side: "<<*calcu_data<<std::endl;
		std::cout<<"walk along the second right-angle side: "<<*(calcu_data+1)<<std::endl;
		std::cout<<"the firt Corner: "<<*(calcu_data+2)<<std::endl;
		std::cout<<"the second Corner: "<<*(calcu_data+3)<<std::endl;	
		std::cout<<"......"<<std::endl;
	
	}

}//#------------------------------------------- 构造直角三角形 (修改)-----------------------------------------#  

//#------------------------------------------- 调整击球位置 -----------------------------------------#     
void AdjustPosition( float *allballdata, float *calcu_data, float *landmark)
{
    float dis1 = *calcu_data;
    float dis2 = *(calcu_data+1);
    float turnAngle1 = *(calcu_data+2);
    float turnAngle2 = *(calcu_data+3);
	float alpha = *landmark;

	if(alpha>0)
	{
	    motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,0.0,turnAngle1,MovConfig);

		motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,dis1,0.0,MovConfig);

		motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,0.0,turnAngle2,MovConfig);
                     
		motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(dis2,0.0,0.0,MovConfig);
	
	}
    
	if(alpha<0)
	{
	    motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,0.0,turnAngle1,MovConfig);

		motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,dis1,0.0,MovConfig);

		motionProxy.setMoveArmsEnabled(true, false);
		motionProxy.moveTo(0.0,0.0,turnAngle2,MovConfig);

			if(*(allballdata+4)>pi/2){
				motionProxy.setMoveArmsEnabled(true, false);
				motionProxy.moveTo(dis2,0.0,0.0,MovConfig);
				motionProxy.moveTo(0.0,0.0,-pi/2,MovConfig);
			}
			else{
			     motionProxy.setMoveArmsEnabled(true, false);
				 motionProxy.moveTo(dis2,0.0,0.0,MovConfig);
				 motionProxy.moveTo(0.0,0.0,-pi/2,MovConfig);
			}
		
	
	}

	//距离修正
	motionProxy.waitUntilMoveIsFinished();
    motionProxy.angleInterpolationWithSpeed("HeadPitch", 0.373, 0.5);
	motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3);

	redball_detect( allballdata); //callback "redball_detect" function

    motionProxy.setMoveArmsEnabled(true, false);
    //#接下来，机器人调整至距离红球dx击球的位置
    motionProxy.moveTo(*(allballdata+2)-dx,0,0,MovConfig); //向前向后距离矫正
                    
}


////////////////////////////
void openhand()//前头触摸开始openhand
{

	  float headTouchedButtonFlag=0;
	  while( headTouchedButtonFlag==0)
	  {
          headTouchedButtonFlag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value");
		  if(headTouchedButtonFlag==1.0)
			{    			
	          //  postureProxy.goToPosture("StandInit", 0.5f);
	          //  Sleep(1000);

				std::vector<float> position(6,0);	
				position[2] = 0.2977f;//0.227
				motionProxy.setPositions("Torso", 2, position,0.15, 63);
			//	Sleep(1000);                        //蹲下
				AL::ALValue names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw","RHand");
				AL::ALValue stiffnessLists=0.7f;
				AL::ALValue  timeLists=1.0f;
				motionProxy.stiffnessInterpolation(names, stiffnessLists, timeLists);//上刚
  
			    names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
				AL::ALValue Angles =AL::ALValue::array(1.4328,0,1.4328,1.57,0);//Right 预备grab
	            motionProxy.setAngles(names,Angles, 0.1);	  
			
				motionProxy.stiffnessInterpolation("RHand", 0.7, 1);
				motionProxy.angleInterpolation( "RHand", 0.9, 1, true );
				//Sleep(1000);
				break;	 
		    }
		 
	  }  
}

void closehand()//后头触摸开始closehand
{

	  float headTouchedButtonFlag=0;
	  while(headTouchedButtonFlag==0)
	  {
          headTouchedButtonFlag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value");
		  if(headTouchedButtonFlag==1.0)
			{  
				motionProxy.stiffnessInterpolation("RHand", 0.7, 1);
				motionProxy.angleInterpolation( "RHand", 0.2, 1, true );
				break;
				 
		    }
	  }
}

void begin_puthand()//中间触摸开始开始
{

	  float headTouchedButtonFlag=0;
	  while(headTouchedButtonFlag==0)
	  {
          headTouchedButtonFlag = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value");
		  if(headTouchedButtonFlag==1.0)
			{  
				AL::ALValue    names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
				AL::ALValue Angles = AL::ALValue::array(1.4328,-0.5,1.4328,1.57,-1.36);//翻手--
				motionProxy.angleInterpolationWithSpeed(names,Angles, 0.1);
			//	Sleep(500);
				Angles = AL::ALValue::array(1.4328,-0.5,0.07,1.57,0);//伸直，手选转-
				motionProxy.angleInterpolationWithSpeed(names,Angles, 0.1);
			//	Sleep(500);
				Angles = AL::ALValue::array(1.4328,-0.5,0.07,0,1.57);    //x向后   //+预备//1.58,-0.503,0.350,0,1.58
				motionProxy.angleInterpolationWithSpeed(names,Angles, 0.1);//1.92
			//	Sleep(500);	

			    break;
				 
		    }
	  }
}

void openclose()
{
	   float  FrontTactilTouched=0;
	   float  RearTactilTouched=0;
	   float  MiddleTactilTouched=0;

	   AL::ALValue  names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
	   AL::ALValue Angles =AL::ALValue::array(1.4328,0,1.4328,1.57,0);//Right 预备grab
	   motionProxy.setAngles(names,Angles, 0.1);	

	   int flag=1;
	   while(flag)
	   {
           FrontTactilTouched = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value");
	       RearTactilTouched = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value");
		   MiddleTactilTouched = memoryProxy.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value");
		   if(FrontTactilTouched==1.0)
		   {
			    motionProxy.stiffnessInterpolation("RHand", 0.7, 1);
				motionProxy.angleInterpolation( "RHand", 0.5, 1.0, true );	       
		   }

		    if( RearTactilTouched==1.0)
			{
			     motionProxy.stiffnessInterpolation("RHand", 0.7, 1);
				 motionProxy.angleInterpolation( "RHand", 0.2, 1.0, true );
			}

			 if(  MiddleTactilTouched==1.0)
			 {
			      
			     flag=0;
				 break;
			 }
	   
	   
	   }


}

void  HitBall( float Distance, float HitSpeed)//"RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw"
{
	AL::ALValue maxSpeedFraction= HitSpeed;
	AL::ALValue minSpeedFraction=0.1f;

	std::vector<float> position(6,0);	
	position[2] = 0.2977f;//0.227
	motionProxy.setPositions("Torso", 2, position,0.15, 63);
	Sleep(500);                        //蹲下

	//---------------------------抬手---------------------------------//
	   AL::ALValue   names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
	   AL::ALValue   Angles = AL::ALValue::array(1.4328,-0.5,0.070,1.57,0);    //x向后   //+预备//1.58,-0.503,0.350,0,1.58
	   motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
	   Sleep(500);
	 
	   Angles = AL::ALValue::array(1.4328,-0.5,1.4328,1.57,-1.57);//伸直，手选转-
	   motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
	   Sleep(500);

	   Angles = AL::ALValue::array(1.4328,0,1.4328,1.57,-0.37);//伸直，手选转-
	   motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);

	   Sleep(500);


	//-----------------------------------------------------------------
	 if(Distance<0.5)
	   {    
			AL::ALValue names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
			AL::ALValue Angles =AL::ALValue::array(1.4328,0,1.4328,1.57,-0.33);//Right 预备//wrist	
         	AL::ALValue minSpeedFraction=0.1f;
			motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
			Sleep(500);	
			AL::ALValue maxSpeedFraction= HitSpeed;
			Angles =AL::ALValue::array(1.4328,0,1.4328,1.57,0.36);//打球
			motionProxy.angleInterpolationWithSpeed(names,Angles, maxSpeedFraction);
			Sleep(500);
		}
    
	 else if((1.5<=Distance) && (Distance<=2.0))
	   {
			AL::ALValue names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
			AL::ALValue Angles =AL::ALValue::array(1.4328,-0.314,+1.4328,1.57,0);//Right 预备//ROll	
	        AL::ALValue minSpeedFraction=0.1f;
			motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
			Sleep(500);
			AL::ALValue maxSpeedFraction= HitSpeed;
			Angles =AL::ALValue::array(1.4328,0.26,1.4328,1.57,0);//打球
			motionProxy.angleInterpolationWithSpeed(names,Angles, maxSpeedFraction);
			Sleep(500);

        }

    else
	    {
			AL::ALValue names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
		    AL::ALValue Angles =AL::ALValue::array(1.4328,-0.314,+1.4328,1.57,-0.43);//Right 预备//Roll and wrist  (1.4328,-0.33*HitSpeed,+1.4328,1.57,-0.33);
	        AL::ALValue minSpeedFraction=0.1f;
			motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
			Sleep(500);
			maxSpeedFraction=HitSpeed;
			Angles =AL::ALValue::array(1.4328,0.26,1.4328,1.57,0.58); //  打球 	Angles =AL::ALValue::array(1.4328,0.33*HitSpeed,1.4328,1.57,0.3); 
			motionProxy.angleInterpolationWithSpeed(names,Angles, maxSpeedFraction);
			Sleep(500);
		
		}

	   names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
	   Angles = AL::ALValue::array(1.4328,-0.5,1.4328,1.57,-1.36);//翻手--
       motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
	   Sleep(500);
	   Angles = AL::ALValue::array(1.4328,-0.5,0.07,1.57,0);//伸直，手选转-
	   motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
	   Sleep(500);
	   Angles = AL::ALValue::array(1.4328,-0.5,0.07,0,1.57);    //x向后   //+预备//1.58,-0.503,0.350,0,1.58
	   motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);//1.92
	   Sleep(500);

	   
	  
      //-------------------------------------------------------------------------------
}
void DirectHit()
{	
	/*****************击打*********/
	AL::ALValue names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
	AL::ALValue Angles =AL::ALValue::array(1.4328,-0.33,+1.4328,1.57,-0.33);//Right 预备//Roll and wrist  (1.4328,-0.33*HitSpeed,+1.4328,1.57,-0.33);
	AL::ALValue minSpeedFraction=0.1f;
	motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
	Sleep(1500);
	AL::ALValue maxSpeedFraction=1.0f;
	Angles =AL::ALValue::array(1.4328,0.33,1.4328,1.57,0.3); //  打球 	Angles =AL::ALValue::array(1.4328,0.33*HitSpeed,1.4328,1.57,0.3); 
	motionProxy.angleInterpolationWithSpeed(names,Angles, maxSpeedFraction);
   //Sleep(500);
	/*************放杆***********/	
	names=AL::ALValue::array("RShoulderPitch","RShoulderRoll","RElbowRoll","RElbowYaw","RWristYaw");
	Angles = AL::ALValue::array(1.4328,-0.5,1.4328,1.57,-1.36);//翻手--
    motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
//   Sleep(500);
	Angles = AL::ALValue::array(1.4328,-0.5,0.07,1.57,0);//伸直，手选转-
	motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);
//   Sleep(500);
	Angles = AL::ALValue::array(1.4328,-0.5,0.07,0,1.57);    //x向后   //+预备//1.58,-0.503,0.350,0,1.58
	motionProxy.angleInterpolationWithSpeed(names,Angles, minSpeedFraction);//1.92
//   Sleep(500);
}

/************近处红球寻找球***********/
void CloseRedBallSearching(float *allballdata)
{
	tts.say(" red ball searching");
	motionProxy.setMoveArmsEnabled(true, false);
	motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
	motionProxy.angleInterpolationWithSpeed("HeadPitch",0.373, 0.2f);//保持低头21.4度:0.373

	int state=1;
	const int StateOne=1;
	const int StateTwo=2;
	const int StateThree=3;
	const int StateFour=4;
	const int StateFive=5;
	const int StateSix=6;
	float HeadYaw= 0;
	float MoveAngle=0;
	float theta,angle,YellowStickAngle=0;
	bool ENDLOOP =false;

	redball_detect(allballdata);
   while(!ENDLOOP)
   {
			if(*(allballdata+4)!=0)
				state=StateSix;
			
			switch( state)
			{
					case StateOne:

								HeadYaw= 0.785f;
								motionProxy.angleInterpolationWithSpeed("HeadYaw", HeadYaw, 0.2f);		
								redball_detect(allballdata);
								if(*(allballdata+4)!=0)
								    {
										tts.say(" I seen the red ball");
										state=StateSix;
										break;
									}
								else
								{
									 state=StateTwo;
	                                 break;
								}

							       
                    case StateTwo:

								HeadYaw=-0.785f;
								motionProxy.angleInterpolationWithSpeed("HeadYaw", HeadYaw, 0.2f);		
								redball_detect(allballdata);
								if(*(allballdata+4)!=0)
								    {
										tts.say(" I seen the red ball");
										state=StateSix;
										break;
									}
								else
								{
									state++;
									motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
									break;
								}
	
	                   case StateThree:
	                            
						         	MoveAngle= 0.78f;			
									motionProxy.moveTo(0, 0, MoveAngle, MovConfig_triangle);		
									redball_detect(allballdata);				
                                    if(*(allballdata+4)!=0)
								     {
										 tts.say(" I seen the red ball");
									     state=StateSix;
										 break;
									 }
							        else
									{
										state++;
										break;
									}
                       
                       case StateFour:  
						            MoveAngle= -1.57f;			
									motionProxy.moveTo(0, 0, MoveAngle, MovConfig_triangle);		
									redball_detect(allballdata);				
                                    if(*(allballdata+4)!=0)
								     {
										 tts.say(" I seen the red ball");
									     state=StateSix;
										 break;
									 }
							        else
									{
										state++;
										break;
									}
                       
                        case StateFive:
							    
								YellowStickAngle= YellowStickSearching();
							    motionProxy.moveTo(0, 0, YellowStickAngle, MovConfig_triangle);	
								VerifyRedBallPresentBeforeHitting(allballdata);
							    ENDLOOP =true;
							    break;

                        case StateSix:

								theta = HeadYaw + *allballdata;
								motionProxy.moveTo(0, 0, theta, MovConfig_triangle);
								motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
								motionProxy.angleInterpolationWithSpeed("HeadPitch",0.373, 0.2f);//保持低头21.4度:0.373
								redball_detect(allballdata);
								if(*(allballdata+4)==0)
								    {
										tts.say("there is no red ball");
									    state=StateOne;
										break;
									}
								motionProxy.moveTo(*(allballdata+6)-0.3, 0, 0,  MovConfig_triangle); //向前向后距离矫正，距红球30c					
								redball_detect(allballdata);
								angle =  *allballdata; //引用变量返回数值
								motionProxy.moveTo(0, 0, angle, MovConfig_triangle);
								redball_detect(allballdata);
								ENDLOOP =true;
								break;

					   default:
						       state=StateOne;
							   break;
	
			   }

   }

	


}
/**********************/

void LandMarkSearching( float *allballdata )
{
	    motionProxy.setMoveArmsEnabled(true, false);
		tts.say("the landmark is in front");
		float angle=0;
		for(int j=0;j<2 ;j++ )
		{     
			float Distance =0.3f;
			float WALKING_X= Distance - Distance*cos(0.69f);
			float WALKING_Y= Distance*sin(0.69f);
			motionProxy.moveTo(WALKING_X,WALKING_Y,-0.69f,MovConfig_triangle);													
		}
		motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1f);
		motionProxy.angleInterpolationWithSpeed("HeadPitch",0.40, 0.1f);//保持低头21.4度:0.373	
		redball_detect( allballdata); //callback "redball_detect" function	
		if(*(allballdata+4)==0)
			 CloseRedBallSearching(allballdata);
			               
		angle =  *allballdata; 
		motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
		redball_detect( allballdata); //callback "redball_detect" function		
		motionProxy.moveTo(*(allballdata+6)-0.16,0,0,MovConfig_triangle);	
		redball_detect( allballdata); //callback "redball_detect" function	
		angle =  *allballdata; 
		motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	

		motionProxy.moveTo(0,0.07,0,MovConfig_triangle);//侧移	
}

/********************/
void TRIANGLE_MAKING( float *allballdata,  float *landmark )
{

	
	tts.say("making triangle"); 
	motionProxy.setMoveArmsEnabled(true, false);
	bool EndLoop=false;
	bool Landmarkflag=true;
	int state=0;
	enum {one,two,three,four,five,six};
	bool farball=true;
	float Distance=0.3f;

	float alpha,landmark_x,landmark_y,redball_x,redball_y,landmark_redball_x,landmark_redball_y,flag,range,AboutAngle, WalkingAngle,WALKING_X,WALKING_Y,ANGLE=0,angle=0,theta=0;

	int REDstate=1;
	const int StateOne=1;
	const int StateTwo=2;
	float HeadYaw= 0;
	float MoveAngle= 0.35f;
	bool ENDREDLOOP =false;


	while(  ! EndLoop  )
	{	  
		
		
	switch(state)
	{
			case one:

				 alpha = *landmark;//
				 landmark_x=*(landmark+3);
				 landmark_y=*(landmark+4);
				 redball_x=-*(allballdata+2);
				 redball_y=-*(allballdata+3);

				 landmark_redball_x=landmark_x+redball_x;
				 landmark_redball_y=landmark_y+redball_y;


				 flag=landmark_redball_x*redball_x+landmark_redball_y*redball_y;
				 range= flag/( sqrt(landmark_redball_x*landmark_redball_x+landmark_redball_y*landmark_redball_y )* sqrt( redball_x*redball_x +redball_y*redball_y)  );
				 AboutAngle=  acos(     (landmark_redball_x*redball_x+landmark_redball_y*redball_y) /  (
										sqrt(landmark_redball_x*landmark_redball_x+landmark_redball_y*landmark_redball_y )* sqrt( redball_x*redball_x +redball_y*redball_y)  ) );

			     WalkingAngle=abs(AboutAngle-pi/2);

				if(alpha>0)
					{
										
						if( ( range<=0.17 && range>=-0.17) &&  (*(allballdata+6) >= 0.28 ) )  //										
							{
								state=six;
								Distance=0.16f;
								break;
							}

						else if(  ( *(allballdata+6) <=0.20) &&( range<=0.087 && range>=-0.087 ) )
						{
							motionProxy.moveTo(0,0.07,0,MovConfig_triangle);	
						    tts.say("start hittint");
						    EndLoop=true;
							//return true;
						    break;
						
						}
						else
							state=two;
		
					}

				else
					state=three;
				break;



             case two:

   	               if(flag<0)
					  {					 					  
						
						for(int j=1;j<3;j++ )
							{														
									WALKING_X= Distance - Distance*cos(WalkingAngle/2)  ;
									WALKING_Y= Distance*sin(WalkingAngle/2);								
								    ANGLE=WalkingAngle/2;
									motionProxy.moveTo(WALKING_X,WALKING_Y,-ANGLE,MovConfig_triangle);	
							}

					  } 
					else
					{
												  
						for(int j=1;j<3;j++ )
						{												
								WALKING_X= Distance - Distance*cos(WalkingAngle/2 );
								WALKING_Y= Distance*sin(WalkingAngle/2);
								ANGLE=WalkingAngle/2;
								motionProxy.moveTo(WALKING_X,-WALKING_Y,ANGLE,MovConfig_triangle);									
						}
							
					}
				   
					state=four;  
					break;


			case three:
				    
						for(int j=0;j<3;j++ )
							{											
									WALKING_X= Distance - Distance*cos(0.35f );
								    WALKING_Y= Distance*sin(0.35f );								
									motionProxy.moveTo(WALKING_X,WALKING_Y,-0.35f,MovConfig_triangle);	
							}
						 state=four; 
		 		         break;


			case four://find ren ball

				if(farball==true)
			       CloseRedBallSearching(allballdata);
				else
				{

					motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1f);
					motionProxy.angleInterpolationWithSpeed("HeadPitch",0.40, 0.1f);//保持低头21.4度:0.373	
					redball_detect( allballdata); //callback "redball_detect" function	
					if(*(allballdata+4)==0)
					{
						CloseRedBallSearching(allballdata);	
						state=four; 
		 				break;
					}
                       
					angle =  *allballdata; //引用变量返回数值
					
					motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	

					redball_detect( allballdata); //callback "redball_detect" function	
					
					motionProxy.moveTo(*(allballdata+6)-0.16,0,0,MovConfig_triangle);	
					redball_detect( allballdata); //callback "redball_detect" function	
					angle =  *allballdata; //引用变量返回数值
					motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
					redball_detect( allballdata); //callback "redball_detect" function						 			
				}

				 state=five;
				 break;

			case five://find landmark

		        Landmarkflag= landmark_Find( landmark, allballdata);
				if(Landmarkflag==false)
					{
							if(farball==false)
							{
								motionProxy.moveTo(0,0.07,0,MovConfig_triangle);	
								tts.say("start hittint");
								EndLoop=true;
								break;											
							}
							/**********Landmark Search*******/
							while(!Landmarkflag)
							{
							
								//  float WalkingAngle=0.524f;
								for(int j=0;j<3;j++ )
								{
				  	
										WALKING_X= Distance - Distance*cos(0.52f);
										WALKING_Y= Distance*sin(0.52f);
										motionProxy.moveTo(WALKING_X,WALKING_Y,-0.52f,MovConfig_triangle);	
														
								}

							//	MovetowardsRedBallBeforeHitting( allballdata);
								 CloseRedBallSearching(allballdata);
								Landmarkflag= landmark_Find( landmark, allballdata);

								if(Landmarkflag==true)
									break;
								else
								{
										for(int i=0;i<3;i++ )
										{
						
											float WALKING_X= Distance - Distance*cos(0.52f  );
											float WALKING_Y= Distance*sin(0.52f );
											motionProxy.moveTo(WALKING_X,-WALKING_Y,0.52f,MovConfig_triangle);		
														
										}
									    CloseRedBallSearching(allballdata);
									    Landmarkflag= landmark_Find(landmark, allballdata);//注意！
			
								}

		
							}
                          
						

				    }
				else
				   state=one;
                break; 


           case six://close red ball

				      tts.say("go to  red ball");
					  farball=false;	  
					  motionProxy.moveTo(0.08,0,0,MovConfig_triangle);	
                      state=four;
					  break;


	   }
	
		
		


	
	}


}

/**************NIGHT_SunDwon**日落**********
float YellowStickDetect_SunDown(  )
{

	//tts.say("yellow stick searching");
	RNG rng(12345);
    int Hm = 27;//18 早上 //27 晚上
	int Sm = 55;
	int Vm = 171;//125 早上 //171  晚上
	int HM = 50;
	int SM = 255;
	int VM = 255;
	int cameraId = 0;//0上，1下
	cameraProxy.setActiveCamera(cameraId);
	 std::string clientName =cameraProxy.subscribe("test", kVGA, kBGRColorSpace, 1);//fps=5
	
     cv::Mat imgHeader = cv::Mat(cv::Size(640, 480),CV_8UC3); 
	 cv:: Mat  frameHSV,img2;
     cv::  Mat planes[] = { Mat::zeros(img2.size(), CV_8UC1), Mat::zeros(img2.size(), CV_8UC1), Mat::zeros(img2.size(), CV_8UC1) };
     cv:: Point point;
      float centerX=0;
	

	//vector<vector<Point>> contours;//下面的
	//vector<Vec4i> hierarchy;
	for(int i=0;i<2;i++  )
	{
	    
			ALValue img =cameraProxy.getImageRemote(clientName);
			imgHeader.data = (uchar*)img[6].GetBinary();  

			img2 = imgHeader(Rect(0,240,640,240));

		 
			cvtColor(img2, frameHSV, CV_BGR2HSV);
			inRange(frameHSV, Scalar(Hm, Sm, Vm), Scalar(HM, SM, VM), frameHSV);
			split(frameHSV, planes);
			blur(planes[0],planes[0], Size(3, 3));
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			findContours(planes[0], contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
			vector<vector<Point>>contours_poly(contours.size());
			vector<Rect>boundRect(contours.size());
	  
			for (size_t i = 0; i < contours.size(); i++)
				{
			
			    if((arcLength(contours[i], true)>100) && (contourArea(contours[i])>300) )
				{
					approxPolyDP(Mat(contours[i]), contours_poly[i],3,true);
					boundRect[i] = boundingRect (Mat(contours_poly[i]));

					Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
					point.x=(boundRect[i].tl().x + boundRect[i].br().x) / 2;
					point.y=(boundRect[i].tl().y + boundRect[i].br().y) / 2;
			
					centerX =(320.0f-point.x)*60.92f/640.0f*0.01744f;
					
				   
				 	//Sleep(1000);
						//std::cout<<centerX<<std::endl;

					//rectangle(img2,boundRect[i].tl(),boundRect[i].br(),color,2,8,0);
					//circle(img2, point, 4, color, -1, 8, 0); 
				}
		  
		   }

				//imshow("原图",img2);


			if (waitKey(10) >= 0)
			break;
			cameraProxy.releaseImage(clientName);
	}
    
	
	cameraProxy.unsubscribe(clientName);
	destroyAllWindows();
	return centerX;

}
/***********************/

/******Sun_Up*******日出**********/
float YellowStickDetect_SunUp(  )
{

	//tts.say("yellow stick searching");
	RNG rng(12345);
    int Hm = 27;//18 早上//晚上 27
	int Sm = 55;
	int Vm = 171;//125 早上  //晚上 171 
	int HM = 50;
	int SM = 255;
	int VM = 255;
	int cameraId = 0;//0上，1下
	cameraProxy.setActiveCamera(cameraId);
	 std::string clientName =cameraProxy.subscribe("test", kVGA, kBGRColorSpace, 1);//fps=5
	
     cv::Mat imgHeader = cv::Mat(cv::Size(640, 480),CV_8UC3); 
	 cv:: Mat  frameHSV,img2;
     cv::  Mat planes[] = { Mat::zeros(img2.size(), CV_8UC1), Mat::zeros(img2.size(), CV_8UC1), Mat::zeros(img2.size(), CV_8UC1) };
     cv:: Point point;
      float centerX=0;
	

	//vector<vector<Point>> contours;//下面的
	//vector<Vec4i> hierarchy;
	for(int i=0;i<2;i++  )
	{
	    
			ALValue img =cameraProxy.getImageRemote(clientName);
			imgHeader.data = (uchar*)img[6].GetBinary();  

			img2 = imgHeader(Rect(0,240,640,240));

		 
			cvtColor(img2, frameHSV, CV_BGR2HSV);
			inRange(frameHSV, Scalar(Hm, Sm, Vm), Scalar(HM, SM, VM), frameHSV);
			split(frameHSV, planes);
			blur(planes[0],planes[0], Size(3, 3));
			vector<vector<Point>> contours;
			vector<Vec4i> hierarchy;
			findContours(planes[0], contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
			vector<vector<Point>>contours_poly(contours.size());
			vector<Rect>boundRect(contours.size());
	  
			for (size_t i = 0; i < contours.size(); i++)
				{
			
			    if((arcLength(contours[i], true)>100) && (contourArea(contours[i])>300) )
				{
					approxPolyDP(Mat(contours[i]), contours_poly[i],3,true);
					boundRect[i] = boundingRect (Mat(contours_poly[i]));

					Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
					point.x=(boundRect[i].tl().x + boundRect[i].br().x) / 2;
					point.y=(boundRect[i].tl().y + boundRect[i].br().y) / 2;
			
					centerX =(320.0f-point.x)*60.92f/640.0f*0.01744f;
					
				   
				 	//Sleep(1000);
						//std::cout<<centerX<<std::endl;

					//rectangle(img2,boundRect[i].tl(),boundRect[i].br(),color,2,8,0);
					//circle(img2, point, 4, color, -1, 8, 0); 
				}
		  
		   }

				//imshow("原图",img2);


			if (waitKey(10) >= 0)
			break;
			cameraProxy.releaseImage(clientName);
	}
    
	
	cameraProxy.unsubscribe(clientName);
	destroyAllWindows();
	return centerX;

}
/***********************/

float YellowStickSearching()
{
	tts.say("stick  searching");
	motionProxy.setMoveArmsEnabled(true, false);			     
	motionProxy.angleInterpolationWithSpeed("HeadYaw",0.0f, 0.1f);	
	motionProxy.angleInterpolationWithSpeed("HeadPitch",-0.17f, 0.1f);
	float  CenterYellowStick=0,headYawAngle= 2.0f;;
	CenterYellowStick=YellowStickDetect_SunUp();

	while(headYawAngle>-1.9f  )
	{
		if(  CenterYellowStick!=0  )
		{
			tts.say("I see the stick");
			std::vector<float> commandAngles =motionProxy.getAngles("HeadYaw", true);
			headYawAngle=commandAngles.at(0);	
			std::cout<<" CenterYellowStick:"<< CenterYellowStick<<std::endl;
			std::cout<<"YawAngle:"<<(headYawAngle+ CenterYellowStick )<<std::endl;
			//centerX+=headYawAngle;
			//motionProxy.moveTo(0,0,(headYawAngle+ CenterYellowStick ),MovConfig_triangle);	
			break;
		 }

	   headYawAngle-=0.5f;
	   motionProxy.angleInterpolationWithSpeed("HeadYaw",headYawAngle, 0.1f);
	   CenterYellowStick=YellowStickDetect_SunUp();
	   tts.say("yellow  searching");
			
	  }
  if( CenterYellowStick!=0)
    return (headYawAngle+ CenterYellowStick );
  else
	return 0;

}
/************************/
void STICK_RED_Triangle(  float *allballdata  ) 
{
	int state=0;
	enum {one,two,three,four,five,six};
	bool ENDLOOP=false,FarDistance=true;
	float YellowStick=0;
    float Distance=0.3f,WALKING_X,WALKING_Y, WalkingAngle,ANGLE,angle,YellowBallAngle=0;
	motionProxy.setMoveArmsEnabled(true, false);
	/*********************
	
				motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1f);
				motionProxy.angleInterpolationWithSpeed("HeadPitch",0.373, 0.1f);//保持低头21.4度:0.373
				redball_detect(allballdata);
				if(*(allballdata+4)==0)
					    CloseRedBallSearching(allballdata);
					
				angle =  *allballdata; //引用变量返回数值
				motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
			    motionProxy.moveTo(*(allballdata+6)-0.30,0,0,MovConfig_triangle);
				redball_detect(allballdata);
			    angle =  *allballdata; //引用变量返回数值
				motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
	 YellowBallAngle= YellowStickSearching();
	 if( YellowBallAngle==0)
		  YellowBallAngle= YellowStickSearching();
	 /*********************/
     YellowBallAngle= YellowAngle;
	while(!ENDLOOP)
	{
		switch (state)
		{
		     case one: 
				 
				//   CloseRedBallSearching( motionProxy,  tts,  postureProxy, cameraProxy,allballdata,  MovConfig);
				//   YellowBallAngle= YellowStickSearching();
				  YellowBallAngle=YellowBallAngle-*allballdata;
				  WalkingAngle =pi/2 - YellowBallAngle; // pi/2 -  (YellowBallAngle- *allballdata);
				  if( ((*(allballdata+6) >=0.28))&& (YellowBallAngle>=1.3f) && ( YellowBallAngle<=1.91 ) )
				  {
					 
					 state=four;
					 Distance=0.16f;
					 break;
					 
				  }
				  
				  else if( (YellowBallAngle>=1.3f) && ( YellowBallAngle<=1.91 ) && ((*(allballdata+6) <=0.20)) )
				  {
					motionProxy.moveTo(0,0.07,0,MovConfig_triangle);	
					tts.say("start hittint");
					ENDLOOP=true;						 			
					break;
				  
				  }
				  else 
				  {
					  state=two;
					  break;
				  }
				 
		 case two:
			 if(YellowBallAngle>0 )
			 {
			   for(int j=2;j<4;j++ )
				{
																		
						WALKING_X= Distance - Distance*cos(WalkingAngle/2)  ;
						WALKING_Y= Distance*sin(WalkingAngle/2);								
						//motionProxy.setMoveArmsEnabled(true, false);
						ANGLE=WalkingAngle/2;
						motionProxy.moveTo(WALKING_X,WALKING_Y,-ANGLE,MovConfig_triangle);	//moveConfig
						
				}
			 }
			 else 
			 {
				 for(int i=2;i<4;i++)
				 {
				        WALKING_X= Distance - Distance*cos(0.7f)  ;
						WALKING_Y= Distance*sin(0.7f);								
						//motionProxy.setMoveArmsEnabled(true, false);
						ANGLE=0.7f;
						motionProxy.moveTo(WALKING_X,WALKING_Y,-ANGLE,MovConfig_triangle);	//moveConfig	 
				 }
			 }
			state=three;
			break;

		 case three:
			 if(FarDistance==true)
			 {
				  //CloseRedBallSearching(allballdata);

				 /**************/
				motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
				motionProxy.angleInterpolationWithSpeed("HeadPitch",0.373, 0.2f);//保持低头21.4度:0.373
				Sleep(1000);
				redball_detect(allballdata);
				if(*(allballdata+4)==0)
					{
						CloseRedBallSearching(allballdata);
						if(*(allballdata+4)==0)
					        VerifyRedBallPresentBeforeHitting( allballdata);
						//state=three; 
		 				//break;
					}
				angle =  *allballdata; //引用变量返回数值
				motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
			    motionProxy.moveTo(*(allballdata+6)-0.30,0,0,MovConfig_triangle);
				redball_detect(allballdata);
				 /**************/

			 }
			 else
			 {
					motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2f);
					motionProxy.angleInterpolationWithSpeed("HeadPitch",0.40, 0.2f);//保持低头21.4度:0.373	
					redball_detect( allballdata); //callback "redball_detect" function	
					
					if(*(allballdata+4)==0)
						{
							CloseRedBallSearching(allballdata);
							state=three; 
		 					break;
						}
					angle =  *allballdata; //引用变量返回数值
					motionProxy.moveTo(0, 0, angle, MovConfig_triangle);	
					motionProxy.moveTo(*(allballdata+6)-0.16,0,0,MovConfig_triangle);
					redball_detect( allballdata); 
			 
			 }
            YellowBallAngle= YellowStickSearching();
			if(YellowBallAngle==0)
				 {
					 FarDistance=true;
					 state=three;
		             break;
			     }
		    state=one;
		 	break;
		
	case four:

		    tts.say("go to red ball");
			FarDistance=false;
			motionProxy.moveTo(0.07,0,0,MovConfig);//MovConfig	,MovConfig_triangle
			state=three;
			break;
	
	}

  }
}

/**********Task_Two  任务二************/
void Task_two()
 {
	 float  YellowAngle=0;
	 motionProxy.setMoveArmsEnabled(true, false);
     YellowAngle=YellowStickSearching();
	if(YellowAngle != 0)
	{  
		STICK_RED_Triangle( allballdata  );
		HitBall( 2.5, 0.26f);
		motionProxy.moveTo(0,0,1.57,MovConfig);
		motionProxy.moveTo(0.5,0,0,MovConfig);
		motionProxy.moveTo(0.5,0,0,MovConfig);

	}
	else
	{           
		//WALKING_X,WALKING_Y,ANGLE,Distance
		 tts.say("there is no stick");
	     for(int i=2;i<4;i++)
		{
			float WALKING_X,WALKING_Y,ANGLE,Distance;
			WALKING_X= Distance - Distance*cos(0.7f)  ;
			WALKING_Y= Distance*sin(0.7f);								
			ANGLE=0.7f;
			motionProxy.moveTo(WALKING_X,WALKING_Y,-ANGLE,MovConfig_triangle);	//moveConfig	 
		}   
		 YellowAngle=YellowStickSearching();
		if(YellowAngle != 0)
		{  
			STICK_RED_Triangle( allballdata  );
			HitBall( 2.5, 0.26f);
			motionProxy.moveTo(0,0,1.57,MovConfig);
			motionProxy.moveTo(0.5,0,0,MovConfig);
			motionProxy.moveTo(0.5,0,0,MovConfig);
			 
		}
		else
			ROBOT_STATE=7;
	
	}
 
 }

/**********************/
