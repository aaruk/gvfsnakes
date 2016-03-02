#include <imutils.hpp>

using namespace std;
using namespace cv;

void drawOptFlowMap(Mat &fx, Mat &fy, Mat& cflowmap, int step,
                           double scaleFactor, Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step) {
            Point2f fxy;
			fxy.y = fy.at<float>(y, x);
			fxy.x = fx.at<float>(y, x);

			if(fxy.x !=0 || fxy.y != 0) {
				line(cflowmap, Point(x,y),
                     Point(cvRound(x+(fxy.x)*scaleFactor),
                     cvRound(y+(fxy.y)*scaleFactor)),
                     color);
			}
            circle(cflowmap, Point(x,y), 1, color, -1);
        }
}


//--Overloaded functions to display an image in a new window--//
void dispImage(Mat& img) {

	if(img.empty()) { //Read image and display after checking for image validity
		cout << "Error reading image file!";
		cin.ignore();
	}else {
		namedWindow("Image", 0);
		imshow("Image", img);
		waitKey();
	}
}

void dispImage(Mat& img, String windowName) {

	if(img.empty()) { //Read image and display after checking for image validity
		cout << "Error reading image File!";
		cin.ignore();
	}else {
		imshow(windowName, img);
		waitKey();
	}
}

void dispImage(Mat& img, String windowName, int delay) {

	if(img.empty()) { //Read image and display after checking for image validity
		cout << "Error reading image File!";
		cin.ignore();
	}else {
		namedWindow(windowName, 0);
		imshow(windowName, img);
		waitKey(delay);
	}
}

void dispImage(Mat& img, String windowName, String error_msg) {

	if(img.empty()) { //Read image and display after checking for image validity
		cout << error_msg;
		cin.ignore();
	}else {
		namedWindow(windowName, 0);
		imshow(windowName, img);
		waitKey();
	}
}

void dispImage(Mat& img, String windowName, String error_msg, int delay) {

	if(img.empty()) { //Read image and display after checking for image validity
		cout << error_msg;
		cin.ignore();
	}else {
		namedWindow(windowName, 0);
		imshow(windowName, img);
		waitKey(delay);
	}
}

