#ifndef MATH_OPS_H
#define MATH_OPS_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int interp1d(const cv::Mat& x, const cv::Mat& fx, const cv::Mat& xq, cv::Mat& fq);
cv::Mat interp1d(cv::Mat x, cv::Mat fx, cv::Mat xq);
int interp2d(const cv::Mat& xs, const cv::Mat& ys, // Sample points
             const cv::Mat& fs, cv::Mat& fq);       // Function values: in&out
int calcDist(const cv::Mat& x, cv::Mat& dx);

#endif
