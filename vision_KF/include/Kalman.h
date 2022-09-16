#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>


using namespace std;
using namespace cv;



class Kalman{
private:
	cv::KalmanFilter kf;
	cv::Mat measurement;
	cv::Point2f _rect_lu;
	double _rect_width;
	double _rect_height;
	double _vx;
	double _vy;
	double _t;
public:
	Kalman();
	// update
	Point2f update();
	void setRectinfo(cv::Rect& preRect);
	void setFrameTime(double t);
	void setSpeed(cv::Rect& last_rect, cv::Rect& rect);
};
