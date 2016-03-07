/* Functions to implement basic snakes functionality */
#ifndef SNAKES_HPP
#define SNAKES_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace snake {
int deformSnake(cv::Mat& x, cv::Mat& y,
                const cv::Mat& ext_fx,
                const cv::Mat& ext_fy,
                const float& alpha,
                const float& beta,
                const float& gamma,
                const float& kappa,
                const int& iter);


int interploate1d();
int interpolate2d();
int interpolateSnake(const float dmin, const float dmax, cv::Mat& x, cv::Mat& y, cv::Mat& img);
//int getInterpIndex(const cv::Mat& inp_indicator, cv::Mat& q_pts);
cv::Mat getInterpIndex(const cv::Mat& inp_indicator);
void gradVectorField(cv::Mat& fx, cv::Mat& fy, cv::Mat& u, cv::Mat& v,
                                               float mu, int ITER);
void normalizeMat(cv::Mat& fx, cv::Mat& fy);
}

#endif
