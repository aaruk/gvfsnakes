#include<../include/snakes.hpp>
#include<../include/mathOps.hpp>

using namespace std;
using namespace cv;
using namespace snake;

int main(int arc, char* argv[]) {

  Mat test = Mat::zeros(1, 9, CV_32S);
  for (size_t i=0; i<test.cols; i++) {
    if ((i == 3) || (i == 8)) {
      test.at<int>(0, i) = 1;
    }
  }

  Mat ind_var(test.size(), CV_32F);
  for (size_t i=0; i<ind_var.cols; i++) {
    ind_var.at<float>(0, i) = float(i);
  }

  cout << test << endl;
  Mat query;
  query = getInterpIndex(test);
  Mat q_pts = query;
  Mat xq_new(q_pts.size(), CV_32F);
  Mat fx = ind_var + 1;
  cout << fx ;
  interp1d(ind_var, fx, q_pts, xq_new);
  cout << query << endl;
  cout << xq_new << endl;
  return 0;
}
