#include "UDPHandler.h"
#include "Contour.h"
#include "utility/helper.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <array>
#include <unistd.h>
#include <chrono>
#include <X11/Xlib.h> // For XInitThreads()

double distanceTo{0},
	verticalAngleError{0},
	horizontalAngleError{0};

cv::Scalar hsvLow{70, 160, 230},
	hsvHigh{110, 200, 255};

double horizontalFOV{30},
	verticalFOV{60}; //20.93}; //The one we have now is for the smaller one //41.86};

int contourPolygonAccuracy{5};

int minArea{60},
	minRotation{30};

int camSrc{0};

CvCapture_GStreamer cam;
cv::Mat frame;

std::string udpHost{"10.28.51.2"};
int udpSendPort{1182}, udpReceivePort{1183};

int width{160}, height{120};
int framerate{15};

std::string videoHost{"10.28.51.175"}; //"10.0.0.178"};////"10.28.51.201"};//" "10.28.51.210"};//"192.168.137.1"};//
int videoPort{1181};
std::string outputFileDir{"/home/ryan"},
	outputFileName{"pic.jpeg"};

bool verbose{true};
bool showImages{true};

void setCameraNumbers()
{
	FILE *uname;
	char consoleOutput[300];
	int lastchar;

	// Executes the command supplied to popen and saves the output in the char array
	uname = popen("v4l2-ctl --list-devices", "r");
	lastchar = fread(consoleOutput, 1, 300, uname);
	consoleOutput[lastchar] = '\0';

	// Converts the char array to a std::string
	std::string outputString = consoleOutput;

	/**
	 * Working from the inside out:
	 * 	- Finds where the camera's name is in the string
	 * 	- Uses the location of the first character in that string as the starting point for a new search for the location of the first character in /dev/video
	 * 	- Looks ten characters down the string to find the number that comes after /dev/video
	 * 	- Parses the output character for an integer
	 * 	- Assigns that integer to the appropriate variable
	 */
	camSrc = outputString.at(outputString.find("/dev/video", outputString.find("USB 2.0 Camera: HD USB Camera")) + 10) - '0';

	pclose(uname);
}

void transmitVideo()
{
	if (verbose)
	{
		std::cout << "*** Transmitting video ***\n";
	}

	char buffer[500];

	sprintf(buffer,
			"appsrc ! "
			"video/x-raw,format=BGR,width=%d,height=%d,framerate=15/1 ! "
			//"autovideoconvert ! video/x-raw,format=H264 ! "
			//"omxh264enc ! "
			//"rtph264pay ! "
			"videoconvert ! video/x-raw,format=I420 ! "
			"jpegenc ! "
			"multifilesink location=%s max-files=1",
			//"rtpjpegpay ! "
			//"udpsink host=%s port=%d sync=false async=false",
			//videoHost.c_str(), videoPort
			width, height, (outputFileDir + "/" + outputFileName).c_str());

	CvVideoWriter_GStreamer videoWriter;
	videoWriter.open(buffer, 0, framerate, cv::Size(width, height), true);

	if (verbose)
	{
		std::cout << "*** Opened video writer ***\n";
	}

	IplImage *img;
	cv::Mat transmitFrame;
	while (true)
	{
		cam.grabFrame();

		img = cam.retrieveFrame(0);
		frame = cv::cvarrToMat(img);

		if(frame.empty())
		{
			if(verbose)
			{
				std::cout << "*** Frame is empty, retrying... ***\n";
			}
			continue;
		}

		frame.copyTo(transmitFrame);

		if (verbose)
		{
			std::cout << "*** Retrieved frame ***\n";
		}

		cv::line(transmitFrame, cv::Point(frame.cols / 2, 0), cv::Point(frame.cols / 2, frame.rows), cv::Scalar(0, 0, 0), 1.5);

		cv::putText(transmitFrame, "AoE: " + std::to_string(horizontalAngleError), cv::Point(frame.cols - 50, 7), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255));

		if (verbose)
		{
			std::cout << "*** Performed operations on camera feed ***\n";
		}

		IplImage outImage = (IplImage)transmitFrame;
		videoWriter.writeFrame(&outImage);

		if (showImages)
		{
			cv::imshow("Transmitted Image", transmitFrame);
		}

		if (verbose)
		{
			std::cout << "*** Wrote frame to UDP stream ***\n";
		}
	}
}

void legitSendVideo()
{
	char buffer[500];

	sprintf(buffer,
			"LD_LIBRARY_PATH=/usr/local/lib mjpg_streamer "
			"-i \"input_file.so -f %s -n %s -d 0\" "
			"-o \"output_http.so -w /tmp -p %d\"",
			outputFileDir.c_str(), outputFileName.c_str(), videoPort);

	system(buffer);
}

void flashCamera()
{
	char buffer[500];
	sprintf(buffer,
			"v4l2-ctl -d /dev/video%d \
		--set-ctrl brightness=100 \
		--set-ctrl contrast=255 \
		--set-ctrl saturation=100 \
		--set-ctrl white_balance_temperature_auto=0 \
		--set-ctrl white_balance_temperature=0 \
		--set-ctrl sharpness=24 \
		--set-ctrl gain=24 \
		--set-ctrl exposure_auto=1 \
		--set-ctrl exposure_absolute=120",
			camSrc);
	system(buffer);
}

void extractContours(std::vector<std::vector<cv::Point>> &contours, cv::Mat frame, cv::Scalar &hsvLowThreshold, cv::Scalar &hsvHighThreshold, cv::Mat morphElement)
{
	if (showImages)
	{
		cv::imshow("Base Image", frame);
	}

	cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);

	//Singles out the pixels that meet the HSV range of the target and displays them
	cv::inRange(frame, hsvLowThreshold, hsvHighThreshold, frame);

	if (showImages)
	{
		cv::imshow("After inRange", frame);
	}

	//Applies an open morph to the frame (erosion (dark spaces expand) followed by a dilation (light spaces expand) to remove small particles with a kernel specified by morphElement) and displays it
	//cv::morphologyEx(frame, frame, cv::MORPH_OPEN, morphElement);
	//if (displaySteps)
	//cv::imshow(name + "'s Morph", frame);

	//Shaves down the bright parts of the image and then expands them to remove small false positives
	cv::erode(frame, frame, morphElement, cv::Point(-1, -1), 2);
	cv::dilate(frame, frame, morphElement, cv::Point(-1, -1), 2);

	if (showImages)
	{
		cv::imshow("After erosion and dilation", frame);
	}

	//Applies the Canny edge detection algorithm to extract edges
	cv::Canny(frame, frame, 0, 0);

	if (showImages)
	{
		cv::imshow("Canny", frame);
	}

	//Finds the contours in the image and stores them in a vector of vectors of cv::Points (each vector of cv::Points represents the curve of the contour)
	//CV_RETR_EXTERNAL specifies to only detect contours on the edges of particles
	//CV_CHAIN_APPROX_SIMPLE compresses the points of the contour to only include their end points
	cv::findContours(frame, contours, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
}

int main()
{
	if (verbose)
	{
		std::cout << "--- Starting program ---\n";
	}

	cv::Mat morphElement{cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3))};

	XInitThreads();

	system("ls /dev/video*");

	setCameraNumbers();

	UDPHandler udpHandler{udpHost, udpSendPort, udpReceivePort};

	char buffer[500];
	sprintf(buffer,
			"gst-launch-1.0 v4l2src device=/dev/video%d ! "
			"video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1 ! "
			"videoflip method=clockwise ! "
			"queue ! autovideoconvert ! fakesink &",
			camSrc, width, height, framerate);
	system(buffer);

	system("pkill gst-launch-1.0");

	//Creates an array of characters (acts like a string) to hold the
	//gstreamer pipeline
	sprintf(buffer,
			"v4l2src device=/dev/video%d ! "
			"videoscale ! video/x-raw,width=%d,height=%d,framerate=30/1 ! "
			"videorate ! video/x-raw,framerate=%d/1 ! "
			"queue ! autovideoconvert ! appsink",
			camSrc, width, height, framerate);
	//Tells the cam to start reading from the pipeline to process video
	cam.open(CV_CAP_GSTREAMER_FILE, buffer);

	if (verbose)
	{
		std::cout << "--- Opened camera ---\n";
	}

	if (verbose)
	{
		std::cout << "--- Initialization complete ---\n";
	}

	//Creates a new thread in which we create a gstreamer pipeline that transmits video to the Driver Station
	std::thread transmitVideoThread{transmitVideo};
	std::thread legitSendVideoThread{legitSendVideo};

	cv::Mat processingFrame;
	for (int frameCounter{0};; ++frameCounter)
	{
		//Allows us to see the frames we will display with cv::imshow (this slows the program down severely when enabled)
		if (showImages)
		{
			cv::waitKey(1);
		}

		if (frameCounter == 45)
		{
			flashCamera();
		}

		if(frame.empty())
		{
			if(verbose)
			{
				std::cout << "--- Frame is empty, retrying... ---\n";
			}
			continue;
		}

		frame.copyTo(processingFrame);

		std::vector<std::vector<cv::Point>> contoursRaw;
		extractContours(contoursRaw, processingFrame, hsvLow, hsvHigh, morphElement);
		std::vector<Contour> contours(contoursRaw.size());
		for (int i{0}; i < contoursRaw.size(); ++i)
		{
			contours.at(i) = Contour(contoursRaw.at(i));
		}

		if (verbose)
		{
			std::cout << "--- Extracted contours ---\n";
		}

		//Filters out bad contours and adds the contour to the vector
		for (int c{0}; c < contours.size(); ++c)
		{
			if (!contours.at(c).isValid(minArea, minRotation, 3))
			{
				contours.erase(contours.begin() + contours.size() - 1);
				--c;
				continue;
			}
		}

		if (verbose)
		{
			std::cout << "--- Filtered out bad contours ---\n";
		}

		std::vector<std::array<Contour, 2>> pairs{};

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

				//If we found the second contour, add the pair to the list
				if (leastDistantContour != -1)
				{
					pairs.push_back(std::array<Contour, 2>{contours.at(origContour), contours.at(leastDistantContour)});
					break;
				}
			}
		}

		if (verbose)
		{
			std::cout << "--- Matched contour pairs ---\n";
		}

		if (pairs.size() == 0)
		{
			continue;
		}

		std::array<Contour, 2> closestPair{pairs.back()};
		for (int p{0}; p < pairs.size(); ++p)
		{
			double comparePairCenter{((std::max(pairs.at(p).at(0).rotatedBoundingBox.center.x, pairs.at(p).at(1).rotatedBoundingBox.center.x) - std::min(pairs.at(p).at(0).rotatedBoundingBox.center.x, pairs.at(p).at(1).rotatedBoundingBox.center.x)) / 2) + std::min(pairs.at(p).at(0).rotatedBoundingBox.center.x, pairs.at(p).at(1).rotatedBoundingBox.center.x)};
			double closestPairCenter{((std::max(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x) - std::min(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x)) / 2) + std::min(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x)};

			if (std::abs(comparePairCenter) - (width / 2) <
				std::abs(closestPairCenter) - (width / 2))
			{
				closestPair = std::array<Contour, 2>{pairs.at(p).at(0), pairs.at(p).at(1)};
			}
		}

		if (verbose)
		{
			std::cout << "--- Found pairs closest to the center ---\n";
		}

		//For clarity
		double centerX{((std::max(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x) - std::min(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x)) / 2) + std::min(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.x)};
		double centerY{((std::max(closestPair.at(0).rotatedBoundingBox.center.y, closestPair.at(1).rotatedBoundingBox.center.y) - std::min(closestPair.at(0).rotatedBoundingBox.center.y, closestPair.at(1).rotatedBoundingBox.center.y)) / 2) + std::min(closestPair.at(0).rotatedBoundingBox.center.x, closestPair.at(1).rotatedBoundingBox.center.y)};

		//The original contour will always be the left one since that's what we've specified
		//Calculates and spits out some values for us
		//distanceTo = (regression function);
		horizontalAngleError = -((processingFrame.cols / 2.0) - centerX) / processingFrame.cols * horizontalFOV;
		//verticalAngleError = ((processingFrame.rows / 2.0) - centerY) / processingFrame.rows * horizontalFOV;

		double height = closestPair.at(0).rotatedBoundingBox.size.width;

		/*
		height(pixels) / vertical(total pixels) = 6.31(height of tape in inches) / height(of frame in inches)
		height of frame(inches) = 6.31 * vertical(pixels) / height(pixels)
		
		tan(30) = 0.5*height of frame(inches) / distance
		distance = 0.5*height of frame(inches) / tan(30)
		
		distance = 0.5 * 6.31 * vertical(pixels) / height(pixels) / tan(vertical FOV / 2)
		*/
		double distance = 0.5 * 6.31 * processingFrame.rows / height / std::tan(verticalFOV * 0.5 * 3.141592654 / 180); //1751.45 / height; //.1945 * height * height + -7.75 * height + 122.4;

		// Conversion to radians (the std trigonometry functions only take radians)
		horizontalAngleError *= 3.141592654 / 180;

		//horizontalAngleError = std::atan(distance * std::sin(horizontalAngleError) / (distance * std::cos(horizontalAngleError) - 7.5));

		// Conversion back to degrees
		horizontalAngleError *= 180 / 3.141592654;

		udpHandler.send(std::to_string(horizontalAngleError));

		//std::cout << "Max - min y: " << closestPair.at(0).rotatedBoundingBoxPoints[3] -  closestPair.at(0).rotatedBoundingBoxPoints[1] << "\n\n";

		//std::cout << "Height (in pixels): " << height << '\n';
		//std::cout << "Distance: " << distance << '\n';
		//std::cout << "AOE: " << horizontalAngleError << "\n\n";

		if (verbose)
		{
			std::cout << "--- Sent angle of error to the roboRIO ---\n";
		}
	}
}
