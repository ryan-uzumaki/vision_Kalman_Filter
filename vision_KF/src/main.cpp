#include "opencv2/opencv.hpp"
#include "Kalman.h"
#include <iostream>
#include <sstream>
#include <wiringPi.h>
#include <wiringSerial.h>
// #include <thread>

using namespace std;
using namespace cv;

typedef struct ThreshHSV {
	int rh_h = 10;
	int rh_l = 0;
	int rh_h2 = 180;
	int rh_l2 = 160;
}ThreshHSV;


double get_distance(int W, int P) {
	double F = 550;
	double D = 0;
	D = (W * F) / P;
	return D;
}

string Convert(float Num)
{
	std::ostringstream oss;
	oss << Num;
	std::string str(oss.str());
	return str;
}


/**
  * 绘制十字
  * @param[in] img 目标图像
  * @param[in] point 十字中心点
  * @param[in] color 颜色
  * @param[in] size 十字尺寸
  * @param[in] thickness 粗细
*/
inline void drawCross(cv::Mat img, cv::Point2d point, cv::Scalar color,int size,int thickness)
{
    //绘制横线
    cv::line(img,cv::Point2d(point.x-size/2,point.y),cv::Point2d(point.x+size/2,point.y),color,thickness,cv::LINE_AA,0);
    //绘制竖线
    cv::line(img,cv::Point2d(point.x,point.y-size/2),cv::Point2d(point.x,point.y+size/2),color,thickness,cv::LINE_AA,0);

    return;
}


inline void sent(int &com,string targetstr){
	serialPrintf(com,targetstr.c_str());
}

int main()
{
	Point center(320,240);
	int com, receive;
	stringstream fmt;
	string targetstr;
	wiringPiSetup();
	com = serialOpen("/dev/ttyUSB0",115200);
	const int STOP = 0;
	const int FORWARD = 1;
	const int BACKWARD = 2;
	const int LEFT = 3;
	const int RIGHT = 4;
	const int CLOCKWISE = 5;
	const int COUNTERWISE = 6;
	VideoCapture capture(0);
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);

	Kalman kalman;
	cv::Mat image = cv::Mat::zeros(Size(640,480), CV_8UC3);
	cv::Rect last_rect;
	while (true) {
		double t = (double)cv::getTickCount();
		capture >> image;
		if (image.empty()) {
			printf("Error! The frame is empty!\n");
			break;
		}

		int known_W = 9;
		int known_P = 510;
		Mat temp;
		vector<Mat> channels;
		split(image,channels);
		temp = channels[0] - channels[1];//blue
		// temp = channels[1] - channels[0];//green
		// temp = channels[2] - channels[1];//red

		// Mat m = Mat::zeros(image.size(), image.type());
		// addWeighted(image, 0.24, m, 0.0, 0, temp);
		// Mat dst;
		// bilateralFilter(temp, dst, 5, 20, 20);
		// Mat m_ResImg;
		// cvtColor(dst, m_ResImg, COLOR_BGR2HSV);
		// Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		// erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		// erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		// erode(m_ResImg, m_ResImg, element);//œøÐÐž¯ÊŽ²Ù×÷
		// Mat dstImage;
		// inRange(m_ResImg, Scalar(100, 43, 46), Scalar(124, 255, 255), dstImage);

		// vector<vector<Point>> contours;
		// vector<Vec4i> hierarchy;
		// findContours(dstImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
		// cv::Rect rect, predict_rect;
		// cv::Point2f predict_pt;
		// for (size_t i = 0; i < contours.size(); ++i) {
		// 	double contour_area = cv::contourArea(contours[i]);
		// 	if (contour_area < 100) continue;
		// 	rect = cv::boundingRect(contours[i]);
		// 	//if (rect.height < 50) continue;
		// 	kalman.setRectinfo(rect);
		// }

    Mat dst;
	// bilateralFilter(temp, dst, 5, 20, 20);
	// fastNlMeansDenoising(temp, dst, 10);
	// GaussianBlur(temp, dst, Size(3,3), 15);
	blur(temp, dst, Size(25,25), Point(-1,-1));
	Mat dstImage;
	threshold(dst, dstImage, 85, 255, THRESH_BINARY);//for blue
	// threshold(dst, dstImage, 150, 255, THRESH_BINARY);//for green
	// threshold(dst, dstImage, 150, 255, THRESH_BINARY);//for red
	imshow("dstImage", dstImage);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(dstImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	cv::Rect rect, predict_rect;
	// cv::RotatedRect rect1;
	cv::Point2f predict_pt;
	//vector<Rect> rect_outer(contours.size());
	vector<RotatedRect> rect_min(contours.size());
	Point2f point_rect[4];
	// int counter = 0;
	for (size_t i = 0; i < contours.size(); ++i) {
		//rect_outer[i] = cv::boundingRect(Mat(contours[i]));
		//double contour_area = cv::contourArea(contours[i]);
		rect_min[i] = minAreaRect(Mat(contours[i]));//https://www.cnblogs.com/little-monkey/p/7429579.html
		if ((rect_min[i].size.width/rect_min[i].size.height<=0.8)||(rect_min[i].size.width/rect_min[i].size.height>=1.2)) {
			continue;
		}
		// counter++;
		rect = cv::boundingRect(Mat(contours[i]));
		rect_min[i].points(point_rect);
		for (int i = 0; i < 4; i++) {
			line(image, point_rect[i], point_rect[(i+1)%4], Scalar(0,0,255), 1, 8);
		}
		circle(image, Point(rect_min[i].center.x, rect_min[i].center.y), 2, Scalar(0, 0, 255), -1, 8);
		//if (contour_area < 100) continue;
		//if (rect.height < 50) continue;
		kalman.setRectinfo(rect);
	}
		// vector<double> area;
		// for (size_t i = 0; i < contours.size(); ++i) {
		// 	area.push_back(contourArea(contours[i]));
		// }
		// int maxIndex = max_element(area.begin(), area.end()) - area.begin();
		// Rect ret_1 = boundingRect(contours[maxIndex]);
		// // rect = cv::boundingRect(contours[i]);
		// // //if (rect.height < 50) continue;
		// // kalman.setRectinfo(rect);
		// for (size_t i=0;i <contours.size(); i++){
		// 	kalman.setRectinfo(ret_1);
		// }
		predict_pt = kalman.update();

		//cv::circle(image, predict_pt, 5, cv::Scalar(0, 255, 0), -1, 8);
		predict_rect = cv::Rect(predict_pt.x, predict_pt.y, rect.width, rect.height);

		//if (rect.empty()) {
		//	kalman.setRectinfo(last_rect);
		//	cout << predict_rect.size().width << endl;
		//	predict_pt = kalman.update();
		//	predict_rect = cv::Rect(predict_pt.x, predict_pt.y, predict_rect.width, rect.height);
		//}

		const float scale = 1;
		cv::Point rev_pt;
		if ((((rect.tl().x + scale * (rect.tl().x - predict_pt.x)) <= 800) && ((rect.tl().x + scale * (rect.tl().x - predict_pt.x)) >= 0) &&
			((rect.tl().y + scale * (rect.tl().y - predict_pt.y)) <= 600) && ((rect.tl().y + scale * (rect.tl().y - predict_pt.y)) >= 0))) {
			if ((fabs(rect.tl().x - predict_pt.x) > 3) || (fabs(rect.tl().y - predict_pt.y) > 3)) {
				rev_pt.x = rect.tl().x + scale * (rect.tl().x - predict_pt.x);
				rev_pt.y = rect.tl().y + scale * (rect.tl().y - predict_pt.y);
			}
			else {
				rev_pt.x = rect.tl().x;
				rev_pt.y = rect.tl().y;
			}
		}
		else {
			rev_pt.x = rect.tl().x;
			rev_pt.y = rect.tl().y;
		}

		predict_rect = cv::Rect(rev_pt.x, rev_pt.y, rect.width, rect.height);
		//cv::circle(image, predict_rect.tl(), 5, cv::Scalar(0, 255, 0), -1, 8);
		cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 2, 8);
		cv::rectangle(image, predict_rect, cv::Scalar(0, 255, 0), 2, 8);
		circle(image, Point(predict_rect.x+(rect.width/2), predict_rect.y+(rect.height/2)), 2, Scalar(0, 255, 0), -1, 8);
		// circle(image, Point(320, 240), 5, Scalar(0, 0, 255), -1, 8);//draw circle in the center
		drawCross(image,center,Scalar(0,255,0),20,1);
		double dist = get_distance(known_W, rect.width);
		string dist_str = Convert(dist);
		putText(image, "Distance:" + dist_str + "cm", Point(50, 50), FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 1, 8);
		cv::imshow("detected", image);
		fmt<<predict_rect.x+predict_rect.width/2<<","<<predict_rect.y+predict_rect.height/2<<"\n";
		targetstr = fmt.str();
		// sent(com,targetstr.c_str());
		serialPrintf(com,targetstr.c_str());
		delay(10);
		t = ((double)getTickCount()-t)/getTickFrequency();
		cout<<"execute time: "<<t<<endl;
		// if ((predict_rect.x+(predict_rect.width/2)<center.x+20)&&(predict_rect.x+(predict_rect.width/2)>center.x-20)&&
		// (predict_rect.y+(predict_rect.height/2)<center.y+20)&&(predict_rect.y+(predict_rect.height/2)>center.y-20)){
		// 	serialPutchar(com,STOP);
		// }
		// receive = serialGetchar(com);
		// cout<<receive<<endl;
		int c = waitKey(10);
		if (c == 27) { // ÍË³ö
			break;
		}
	}
	serialClose(com);
	capture.release();
	cv::destroyAllWindows();

	return 0;
}