/*
 * Copyright (c) 2015,
 * National Instruments Corporation.
 * All rights reserved.
 */

#include <vector>
#include <iostream>

#include <stdio.h>

#include "MyRio.h"
#include "I2C.h"
#include "Motor_Controller.h"
#include "Utils.h"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/operations.hpp"

#include "ImageSender.h"

using namespace std;
using namespace cv;

#define ENABLE_SERVER 1

NiFpga_Status setupI2CB(MyRio_I2c* i2c);

extern NiFpga_Session myrio_session;
NiFpga_Status status;

int main(int argc, char **argv) {

	//clock_t startTime = clock();

#if  ENABLE_SERVER
	ImageSender imageSender;

	if(imageSender.init() < 0)
	{
		return 1;
	}
#endif

	//clock_t serverInit = clock();

	VideoCapture capWebcam(0); // open the default camera
	if (!capWebcam.isOpened())  // check if we succeeded
		return -1;

	//clock_t camInit = clock();

	status = MyRio_Open();
	if (MyRio_IsNotSuccess(status)) {
		return status;
	}

	//clock_t myrioInit = clock();

	MyRio_I2c i2c;
	status = setupI2CB(&i2c);

	Motor_Controller mc = Motor_Controller(&i2c);
	mc.controllerEnable(DC);

	int volt = mc.readBatteryVoltage(1);
	printf("%d\n\n", volt);


	/*
	clock_t myrioEnable = clock();

	clock_t serverInitTaken = serverInit - startTime;
	clock_t camInitTaken = camInit - serverInit;
	clock_t myrioInitTaken = myrioInit - camInit;
	clock_t myrioEnableTaken = myrioEnable - myrioInit;
	cout << "serverInitTaken " << serverInitTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
	cout << "camInitTaken " << camInitTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
	cout << "myrioInitTaken " << myrioInitTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
	cout << "myrioEnableTaken " << myrioEnableTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
	*/

	Mat imgThreshLow;
	Mat imgThreshHigh;


	vector<cv::Vec3f> v3fCircles;// 3 element vector of floats, this will be the pass by reference output of HoughCircles()

    Mat original;
    Mat processed;

    Scalar minLowThresh = Scalar(0, 155, 155);
	Scalar maxLowThresh = Scalar(18, 255, 255);
	Scalar minHighThresh = Scalar(165, 155, 155);
	Scalar maxHighThresh = Scalar(179, 255, 255);

    clock_t current_ticks, delta_ticks;
    clock_t fps = 0;
    for(;;)
    {
    	
    	//cout << "\n\n####################################################\n\n####################################################\n\n####################################################\n\n" << endl;

        current_ticks = clock();
    	//clock_t loopstart = clock();
		



		bool frameReadSuccessfully = capWebcam.read(original);// get next frame
		if (!frameReadSuccessfully || original.empty()) {
			std::cout << "Error: frame not read from webcam\n";
			break;
		}

	    capWebcam.set(CAP_PROP_FRAME_WIDTH,320);
	    capWebcam.set(CAP_PROP_FRAME_HEIGHT,200);

    	/*
    	clock_t readCam = clock();
    	clock_t readCamTaken = readCam - loopstart;
    	cout << "readCamTaken " << readCamTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/




    	//convert color to HSV
		cvtColor(original, processed, COLOR_BGR2HSV);

    	/*
    	clock_t cvtColor = clock();
    	clock_t cvtColorTaken = cvtColor - readCam;
    	cout << "cvtColorTaken " << cvtColorTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/


		inRange(processed, minLowThresh, maxLowThresh, imgThreshLow);
		inRange(processed, minHighThresh, maxHighThresh, imgThreshHigh);

    	/*
    	clock_t inRange = clock();
    	clock_t inRangeTaken = inRange - cvtColor;
    	cout << "inRangeTaken " << inRangeTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/





		add(imgThreshLow, imgThreshHigh, processed);

    	/*
    	clock_t add = clock();
    	clock_t addTaken = add - inRange;
    	cout << "addTaken " << addTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/






		GaussianBlur(processed, processed, cv::Size(3, 3), 0);

    	/*
    	clock_t GaussianBlur = clock();
    	clock_t GaussianBlurTaken = GaussianBlur - add;
    	cout << "GaussianBlurTaken " << GaussianBlurTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/





		Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    	/*
    	clock_t getStructuringElement = clock();
    	clock_t getStructuringElementTaken 	= getStructuringElement - GaussianBlur;
    	cout << "getStructuringElementTaken " << getStructuringElementTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/






		dilate(processed, processed, structuringElement);

    	/*
    	clock_t dilate = clock();
    	clock_t dilateTaken = dilate - getStructuringElement;
    	cout << "dilateTaken " << dilateTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/









		erode(processed, processed, structuringElement);

    	/*
    	clock_t erode = clock();
    	clock_t erodeTaken = erode - dilate;
    	cout << "erodeTaken " << erodeTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/




		HoughCircles(processed,			// input image
						v3fCircles,	// function output (must be a standard template library vector
						HOUGH_GRADIENT,// two-pass algorithm for detecting circles, this is the only choice available
						2,// size of image / this value = "accumulator resolution", i.e. accum res = size of image / 2
						processed.rows / 4,	// min distance in pixels between the centers of the detected circles
						100,// high threshold of Canny edge detector (called by cvHoughCircles)
						50,	// low threshold of Canny edge detector (set at 1/2 previous value)
						15,	// min circle radius (any circles with smaller radius will not be returned)
						400);// max circle radius (any circles with larger radius will not be returned)

    	
    	/*
    	clock_t HoughCircles = clock();
    	clock_t HoughCirclesTaken = HoughCircles - erode;
    	cout << "HoughCirclesTaken " << HoughCirclesTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/







		int maxValue = 0;
		int maxIndex = -1;
		for (int i = 0; i < (int)v3fCircles.size(); i++) {

			// show ball position x, y, and radius to command line
			//cout << "ball position x = " << v3fCircles[i][0] << ", y = " << v3fCircles[i][1] << ", radius = " << v3fCircles[i][2] << "\n";

			// draw small green circle at center of detected object
			circle(original, Point((int) v3fCircles[i][0], (int) v3fCircles[i][1]), 3, Scalar(0, 255, 0), FILLED);

			// draw red circle around the detected object
			circle(original, Point((int) v3fCircles[i][0], (int) v3fCircles[i][1]), (int) v3fCircles[i][2], Scalar(0, 0, 255), 3);

			if( maxValue < ((int) v3fCircles[i][2]) )
			{
				maxIndex = i;
				maxValue = ((int) v3fCircles[i][2]);
			}

		}

		/*
    	clock_t printCircles = clock();
    	clock_t printCirclesTaken = printCircles - HoughCircles;
    	cout << "printCirclesTaken " << printCirclesTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/






    	int speed = 20;

		int percent = 40;
		int leftThreshold = original.cols/100.0*percent;
		int rigthThreshold = original.cols-(original.cols/100.0*percent);

		line(original, Point(leftThreshold, 0), Point(leftThreshold, original.rows), Scalar(255, 0, 0), 2);
		line(original, Point(rigthThreshold, 0), Point(rigthThreshold, original.rows), Scalar(255, 0, 0), 2);

		if(maxIndex > -1){

			// draw blue circle around the max object
			circle(original, Point((int) v3fCircles[maxIndex][0], (int) v3fCircles[maxIndex][1]), (int) v3fCircles[maxIndex][2]+3, Scalar(255, 0, 0), 2);
			//cout << "ball position x = " << v3fCircles[maxIndex][0] << ", y = " << v3fCircles[maxIndex][1] << ", radius = " << v3fCircles[maxIndex][2] << "\n";

			if(v3fCircles[maxIndex][0] < leftThreshold)
			{
				mc.setMotorSpeeds(DC, speed, speed);
				//cout << "Left";
			}
			else if(v3fCircles[maxIndex][0] > rigthThreshold)
			{
				mc.setMotorSpeeds(DC, -1*speed, -1*speed);
				//cout << "Right";
			}
			else
			{
				//cout << "Forward";
				mc.setMotorSpeeds(DC, speed, -1*speed);
			}
		}else{
			mc.setMotorSpeeds(DC, 0, 0);
		}

    	/*
    	clock_t setSpeed = clock();
    	clock_t setSpeedTaken = setSpeed - printCircles;
    	cout << "setSpeedTaken " << setSpeedTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/

//    	rectangle(original, Point(10, 10), Point(30, 30), minLowThresh, FILLED );
//    	rectangle(original, Point(40, 10), Point(60, 30), maxLowThresh, FILLED );
//    	rectangle(original, Point(70, 10), Point(90, 30), minHighThresh, FILLED );
//    	rectangle(original, Point(100, 10), Point(120, 30), maxHighThresh, FILLED );



#if  ENABLE_SERVER
	    imageSender.send(&original);
#endif

    	/*
    	clock_t sendImg = clock();
    	clock_t sendImgTaken = sendImg - setSpeed;
    	cout << "sendImgTaken " << sendImgTaken / (double) CLOCKS_PER_SEC << " sec" << endl;
		*/


    	cout << "numOfCircle " << v3fCircles.size() << endl;

    	//fps counter
    	delta_ticks = clock() - current_ticks; //the time, in ms, that took to render the scene
        if(delta_ticks > 0)
            fps = CLOCKS_PER_SEC / delta_ticks;
        cout << fps << endl;
	}

	Utils::waitFor(2);

	mc.controllerReset(DC);

	status = MyRio_Close();

	return status;
}




NiFpga_Status setupI2CB(MyRio_I2c* i2c){
	uint8_t selectReg;

	/*
	 * Initialize the I2C struct with registers from the FPGA personality.
	 */
	i2c->addr = NiFpga_MyRio1900Fpga60_ControlU8_I2CBADDR;//I2CAADDR;
	i2c->cnfg = NiFpga_MyRio1900Fpga60_ControlU8_I2CBCNFG;//I2CBCNFG;
	i2c->cntl = NiFpga_MyRio1900Fpga60_ControlU8_I2CBCNTL;//I2CBCNTL;
	i2c->cntr = NiFpga_MyRio1900Fpga60_ControlU8_I2CBCNTR;//I2CBCNTR;
	i2c->dati = NiFpga_MyRio1900Fpga60_IndicatorU8_I2CBDATI;//I2CBDATI;
	i2c->dato = NiFpga_MyRio1900Fpga60_ControlU8_I2CBDATO;//I2CBDATO;
	i2c->go = NiFpga_MyRio1900Fpga60_ControlBool_I2CBGO;//I2CBGO;
	i2c->stat = NiFpga_MyRio1900Fpga60_IndicatorU8_I2CBSTAT;//I2CBSTAT;

	/*
	 * Enable the I2C functionality on Connector B
	 * Read the value of the SELECTB register.
	 */
	status = NiFpga_ReadU8(myrio_session, NiFpga_MyRio1900Fpga60_ControlU8_SYSSELECTB, &selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status, "Could not read from the SYSSELECTB register!");

	/*
	 * Set bit7 of the SELECT register to enable the I2C functionality. The
	 * functionality of this bit is specified in the documentation.
	 */
	selectReg = selectReg | (1 << 7);

	/*
	 * Write the updated value of the SELECT register.
	 */
	status = NiFpga_WriteU8(myrio_session, NiFpga_MyRio1900Fpga60_ControlU8_SYSSELECTB, selectReg);
	MyRio_ReturnValueIfNotSuccess(status, status, "Could not write to the SYSSELECTB register!");

	/*
	 * Set the speed of the I2C block.
	 *
	 * Standard mode (100 kbps) = 213.
	 * Fast mode (400 kbps) = 63.
	 */
	I2c_Counter(i2c, 213);

	 /*
	  * Enable the I2C block.
	  */
	I2c_Configure(i2c, I2c_Enabled);

	return status;
}
