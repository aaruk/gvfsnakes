#include<../include/snakes.hpp>

using namespace std;
using namespace cv;
using namespace snake;

int main(int arc, char* argv[]) {

  Mat test = Mat::zeros(1, 9, CV_32S);
  for (size_t i=0; i<test.cols; i++) {
    if ((i == 3) || (i == 4)) {
      test.at<int>(0, i) = 1;
    }
  }
  cout << test << endl;
  Mat query;
  getInterpIndex(test, query);
  cout << query << endl;
  return 0;
}
