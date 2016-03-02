#include <opencv2/imgproc/imgproc.hpp>
#include <../include/snakes.hpp>
#include <../include/imutils.hpp>

using namespace std;
using namespace cv;

Mat im, edge_img;
int edge_threshold = 0;

void applyEdgeThresh(int, void*) {
  
  Canny(im, edge_img, 0, edge_threshold, 3);
  imshow("Edge Image", edge_img);
}

int main(int argc, char* argv[]) {

  if (argc != 2) {
    cout << "Error! Syntax is ./build/snake <img_name>" << endl;
    return -1;
  }

  // Read Image
  im = imread(argv[1], 0);
  edge_img.create(im.size(), im.type());

  // Find Edges
  //dispImage(im, "Input Image");
  namedWindow("Input Image", 0);
  imshow("Input Image", im);
  waitKey(0);
  namedWindow("Edge Image", 0);
  //createTrackbar("Canny Threshold",
  //               "Edge Image",
  //               &edge_threshold, 255, applyEdgeThresh);
  Canny(im, edge_img, 0, 80, 3);
  imshow("Edge Image", edge_img);
  waitKey(0);

  // Compute x, y components of gradients
  Mat dem_img, smove, fx, fy, ux, uy;
  Sobel(edge_img, fx, CV_32F, 1, 0, 3); // Compute gradient of blurred edge map
  Sobel(edge_img, fy, CV_32F, 0, 1, 3); // x, y components separately
  snake::normalizeMat(fx, fy);

  // Diffuse gradients across images
  float mu = 0.08;
  int diffusion_iter = 40;
  cvtColor(im, dem_img, CV_GRAY2RGB);
  dem_img.copyTo(smove);
  snake::gradVectorField(fx, fy, ux, uy, mu, diffusion_iter);
  Scalar color(0, 255, 0);
  drawOptFlowMap(ux, uy, dem_img, 8, 8, color);
  namedWindow("Grad Field", 0);
  dispImage(dem_img, "Grad Field");

  // Initialize Snakes
  Mat contour_img = Mat::zeros(im.rows, im.cols, CV_8U); 
  Point center(400, 400);
  circle(contour_img, center, 90, 255, 1, 8);
  vector<vector<Point>> snake_coords;
  findContours(contour_img, snake_coords, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 

  size_t slen = snake_coords[0].size();
  Mat x(1, slen, CV_32F);
  Mat y(1, slen, CV_32F);
  cout << x.size() << endl;
  float* x_ptr, *y_ptr;
  x_ptr = x.ptr<float>(0);
  y_ptr = y.ptr<float>(0);
  for (size_t i=0; i<slen; i++) {
    x_ptr[i] = snake_coords[0][i].x;
    y_ptr[i] = snake_coords[0][i].y;
  }

  // Find shape by moving snake on image
  namedWindow("Snake on Image", 0);
  for (size_t i=0; i<10; i++) {
    snake::deformSnake(x, y, ux, uy, 2.1, 0.8, 0.5, 0.9, 6);
    snake::interpolateSnake(1, 2, x, y);
    for (size_t j=0; j<x.cols; j++) {
      smove.at<Vec3b>(floor(y.at<float>(0, j)), floor(x.at<float>(0, j))) = 255;
    }
    dispImage(smove, "Snake on Image");
  }
  
  return 0;
}
