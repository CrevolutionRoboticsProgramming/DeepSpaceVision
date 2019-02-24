#include "UDPHandler.h"
#include "Contour.h"
#include "utility/helper.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <array>
#include <unistd.h>

cv::Scalar hsvLow{36, 64, 200},
	hsvHigh{83, 255, 255};

double fovAngle{40};//20.93}; //The one we have now is for the smaller one //41.86};

int contourPolygonAccuracy{5};

int minArea{60},
	minRotation{30};

int processingVideoSource{0},
	viewingVideoSource{1};

std::string udpHost{"10.28.51.2"};
int udpSendPort{9000}, udpReceivePort{9001};

int width{640}, height{480};
int framerate{15};
std::string videoHost{"10.28.51.210"};
int videoPort{9001};

void transmitVideo()
{
	char buffer[500];
	sprintf(buffer,
		"gst-launch-1.0 -v v4l2src device=/dev/video%d ! "
		"image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
		"rtpjpegpay ! "
		"udpsink host=%s port=%d sync=false async=false",
		viewingVideoSource, width, height, framerate, videoHost.c_str(), videoPort);		
	system(buffer);
}

void extractContours(std::vector<std::vector<cv::Point>> &contours, cv::Mat frame, cv::Scalar &hsvLowThreshold, cv::Scalar &hsvHighThreshold, cv::Mat morphElement)
{
	//cv::imshow("Base Image", frame);

	cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

	//Singles out the pixels that meet the HSV range of the target and displays them
	cv::inRange(frame, hsvLowThreshold, hsvHighThreshold, frame);

	//cv::imshow("After inRange", frame);

	//Applies an open morph to the frame (erosion (dark spaces expand) followed by a dilation (light spaces expand) to remove small particles with a kernel specified by morphElement) and displays it
	//cv::morphologyEx(frame, frame, cv::MORPH_OPEN, morphElement);
	//if (displaySteps)
	//cv::imshow(name + "'s Morph", frame);

	//Shaves down the bright parts of the image and then expands them to remove small false positives
	cv::erode(frame, frame, morphElement, cv::Point(-1, -1), 2);
	cv::dilate(frame, frame, morphElement, cv::Point(-1, -1), 2);

	//cv::imshow("After erosion and dilation", frame);

	//Applies the Canny edge detection algorithm to extract edges
	cv::Canny(frame, frame, 0, 0);

	//cv::imshow("Canny", frame);

	//Finds the contours in the image and stores them in a vector of vectors of cv::Points (each vector of cv::Points represents the curve of the contour)
	//CV_RETR_EXTERNAL specifies to only detect contours on the edges of particles
	//CV_CHAIN_APPROX_SIMPLE compresses the points of the contour to only include their end points
	cv::findContours(frame, contours, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
}

int main()
{	
	//Creates a new thread in which we create a gstreamer pipeline that transmits video to the Driver Station
	std::thread transmitVideoThread{transmitVideo};

	cv::Mat frame;

	double distanceTo{0},
		verticalAngleError{0},
		horizontalAngleError{0};

	cv::Mat morphElement{cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3))};

	UDPHandler udpHandler{udpHost, udpSendPort, udpReceivePort};

	CvCapture_GStreamer camera;
	
	char buffer[500];

	//Flashes the camera with optimal settings for identifying the targets
	sprintf(buffer,
		"v4l2-ctl -d /dev/video%d \
		--set-ctrl brightness=100 \
		--set-ctrl contrast=0 \
		--set-ctrl saturation=100 \
		--set-ctrl white_balance_temperature_auto=0 \
		--set-ctrl white_balance_temperature=9000 \
		--set-ctrl power_line_frequency=2 \
		--set-ctrl sharpness=24 \
		--set-ctrl exposure_auto=1 \
		--set-ctrl exposure_absolute=5",
		processingVideoSource);
	system(buffer);

	//Makes sure the camera is set to its optimal settings for actually seeing what's going on
	sprintf(buffer, 
		"v4l2-ctl -d /dev/video1 \
		--set-ctrl brightness=100 \
		--set-ctrl contrast=25 \
		--set-ctrl saturation=30 \
		--set-ctrl white_balance_temperature_auto=0 \
		--set-ctrl white_balance_temperature=50 \
		--set-ctrl power_line_frequency=2 \
		--set-ctrl sharpness=50 \
		--set-ctrl exposure_auto=1 \
		--set-ctrl exposure_absolute=50",
		viewingVideoSource);
	system(buffer);

	//Creates an array of characters (acts like a string) to hold the split
	//gstreamer pipeline. One half is sent to the Driver Station and the
	//other is used for vision processing.
	sprintf(buffer,
		"v4l2src device=/dev/video%d ! "
		"video/x-raw,format=(string)I420,width=(int)%d,height=(int)%d,framerate=(fraction)%d/1 ! "
		"queue ! autovideoconvert ! appsink",
		processingVideoSource, width, height, framerate);
		
	//Tells the camera to start reading from the pipeline to process video
	camera.open(CV_CAP_GSTREAMER_FILE, buffer);

	while (true)
	{
		//Gets the next frame from the camera
		//This returns true if it was successful
		while (!camera.grabFrame())
		{
			std::cout << "Could not retrieve frame! Please check camera connection. Trying again...\n";
		}

		IplImage *img = camera.retrieveFrame(0);
		frame = cv::cvarrToMat(img);
		
		std::vector<std::vector<cv::Point>> contoursRaw;
		extractContours(contoursRaw, frame, hsvLow, hsvHigh, morphElement);
		std::vector<Contour> contours(contoursRaw.size());
		for (int i{0}; i < contoursRaw.size(); ++i)
		{
			contours.at(i) = Contour(contoursRaw.at(i));
		}

		//Filters out bad contours and adds the dimensions of the good contours to the above vectors
		for (int c{0}; c < contours.size(); ++c)
		{
			if (!contours.at(c).isValid(minArea, minRotation, 3))
			{
				contours.erase(contours.begin() + contours.size() - 1);
				--c;
				continue;
			}
		}

		//Least distant contour initialized with -1 so it's not confused for an actual contour and can be tested for not being valid
		int leastDistantContour{-1};

		//Now that we've identified compliant targets, we find their match (if they have one)
		for (int origContour{0}; origContour < contours.size(); ++origContour)
		{
			//We identify the left one first because why not
			if (contours.at(origContour).angle > 0)
			{
				//Iterates through all of the contours and compares them against the original
				for (int compareContour{0}; compareContour < contours.size(); ++compareContour)
				{
					//If the contour to compare against isn't the original
					//and the contour is angled left
					//and the contour is right of the original
					//and (if the least distant contour hasn't been set
					//OR this contour is closer than the last least distant contour)
					//then this contour is the new least distant contour
					if (compareContour != origContour && contours.at(compareContour).angle < 0 && contours.at(origContour).rotatedBoundingBoxPoints[0].x < contours.at(compareContour).rotatedBoundingBoxPoints[0].x)
					{
						//We test if it's closer to the original contour after checking if the
						//index is negative since passing a negative number to a vector will
						//throw an OutOfBounds exception
						if (leastDistantContour == -1)
						{
							leastDistantContour = compareContour;
						}
						else if (contours.at(compareContour).rotatedBoundingBoxPoints[0].x - contours.at(origContour).rotatedBoundingBoxPoints[0].x < contours.at(leastDistantContour).rotatedBoundingBoxPoints[0].x)
						{
							leastDistantContour = compareContour;
						}
					}
				}

				//If we found the second contour, calculate its position in relation to us
				if (leastDistantContour != -1)
				{
					//For clarity
					double origCenterX = contours.at(origContour).rotatedBoundingBox.center.x,
						   origCenterY = contours.at(origContour).rotatedBoundingBox.center.y;
					double leastDistantX = contours.at(leastDistantContour).rotatedBoundingBox.center.x,
						   leastDistantY = contours.at(leastDistantContour).rotatedBoundingBox.center.y;

					//The original contour will always be the left one since that's what we've specified
					//Calculates and spits out some values for us
					//distanceTo = (whatever calculation);
					horizontalAngleError = -((frame.cols / 2.0) - (origCenterX + ((leastDistantX - origCenterX) / 2))) / frame.cols * fovAngle;
					verticalAngleError = ((frame.rows / 2.0) - (origCenterY + ((leastDistantY - origCenterY) / 2))) / frame.rows * fovAngle;

					//std::cout << horizontalAngleError << '\n';

					udpHandler.send(std::to_string(horizontalAngleError));
				}
			}
		}
			

		//Allows us to see the frames we displayed with cv::imshow
		cv::waitKey(1);
	}

	camera.close();
}

/*
	Theory

	Tape is 5.5in long by 2in wide
	Angled at ~14.5 degrees towards each other
	8in gap at their closest

	1. Identify compliant shapes
	2. Calculate angles of tape
	3. Identify compliant pairs (angled right is left of angled left by certain amount determined by ratio of size)
		a. If no compliant pairs, do nothing
	4. Calculate distance to tape and angle of error

	What do when no see full tape?!?!?
	Tell drivers to back up enough to see full tape -> angle correctly -> drive forward?

	Positive angle means angled right, negative angle means angled left
*/

//3.1875'
//theta = arctan(Tft*FOVpixel/(Tpixel*d))
//3.75' = 41.96
//5.91667 = 42.788
//8.75 = 41.828
//11 = 41.3132
//13.1667 = 41.6
//15.58333 = 41.686

//Average = 41.86
