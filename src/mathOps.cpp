#include <mathOps.hpp>

using namespace std;
using namespace cv;

int interp2d(const cv::Mat& xs, const cv::Mat& ys, // Sample points
             const cv::Mat& fs, cv::Mat& fq) {     // Function values: in&out
  int xs_len = xs.cols;
  fq.create(xs.size(), xs.type());

  // Get grid coordinates surrounding snake location

  /*------------*
    |  |  |  |  |
    *--*--*--*--*
    |  |  |  |  |
    *--*--*--*--*
    |  |  |  |  |
    *--*--*--*--*
   */
  Mat x1(xs.size(), xs.type());
  Mat x2(xs.size(), xs.type());
  Mat y1(ys.size(), ys.type());
  Mat y2(ys.size(), ys.type());

  // Access opencv Mat elements using pointers
  const float* xs_ptr  = xs.ptr<float>(0);
  const float* ys_ptr  = ys.ptr<float>(0);
  float* x1_ptr = x1.ptr<float>(0);
  float* x2_ptr = x2.ptr<float>(0);
  float* y1_ptr = y1.ptr<float>(0);
  float* y2_ptr = y2.ptr<float>(0);
  
  for (size_t i=0; i<xs_len; i++) {
    x1_ptr[i] = floor(xs_ptr[i]);
    x2_ptr[i] = ceil(xs_ptr[i]);
    y1_ptr[i] = floor(ys_ptr[i]);
    y2_ptr[i] = ceil(ys_ptr[i]);
  }

  Mat xdiff1, xdiff2, xdiff3, ydiff1, ydiff2, ydiff3;
  subtract(x2, xs, xdiff1);
  subtract(xs, x1, xdiff2);
  subtract(x2, x1, xdiff3);
  subtract(y2, ys, ydiff1);
  subtract(ys, y1, ydiff2);
  subtract(y2, y1, ydiff3);

  // Formula used to compute BLT:
  // fq(x, y) = 1/((x2-x1) * (y2-y1))*[((x2-x) * (y2-y) * fx(x1, y1)) 
  //            +((x-x1) * (y2-y) * fx(x2, y1))
  //            +((x2-x) * (y-y1) * fx(x1, y2))
  //            +((x-x1) * (y-y1) * fx(x2, y2))]

  // Calculate the numerator and denominator components independently
  Mat nr1, nr2, nr3, nr4;
  Mat dr;
  multiply(xdiff1, ydiff1, nr1);    //nr1 = xdiff1 * ydiff1;
  multiply(xdiff2, ydiff1, nr2);   //nr2 = xdiff2 * ydiff1;
  multiply(xdiff1, ydiff2, nr3);   //nr3 = xdiff1 * ydiff2;
  multiply(xdiff2, ydiff2, nr4);    //nr4 = xdiff2 * ydiff2;
  multiply(xdiff3, ydiff3, dr);    //dr = xdiff3 * ydiff3;

  // Pointers to newly created matrices
  float* xdiff2_ptr = xdiff2.ptr<float>(0);
  float* xdiff3_ptr = xdiff3.ptr<float>(0);
  float* ydiff2_ptr = ydiff2.ptr<float>(0);
  float* ydiff3_ptr = ydiff3.ptr<float>(0);
  float* nr1_ptr = nr1.ptr<float>(0);
  float* nr2_ptr = nr2.ptr<float>(0);
  float* nr3_ptr = nr3.ptr<float>(0);
  float* nr4_ptr = nr4.ptr<float>(0);
  float* dr_ptr  = dr.ptr<float>(0);
  float* fq_ptr  = fq.ptr<float>(0);


  // Compute function value at query points
  for(int i=0; i<xs_len; i++) {
  
    if ((xdiff3_ptr[i] == 0) && (ydiff3_ptr[i] == 0)) {
	  // If point is located at a valid location, copy function value at that point directly
      fq_ptr[i] = fs.at<float>(int(xs_ptr[i]), int(ys_ptr[i]));

    } else if ((xdiff3_ptr[i] != 0) && (ydiff3_ptr[i] == 0)) {
      // For Points located on same grid line along y make it a linear interpolation along x
      fq_ptr[i] =  (fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))
                  - fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i])))
                  * xdiff2_ptr[i] / xdiff3_ptr[i]
                  + fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]));
    		
    } else if((xdiff3.at<float>(0, i) == 0) && (ydiff3.at<float>(0, i) != 0)) {
      // For Points located on same grid line along x make it a linear interpolation along y
      fq_ptr[i] =  (fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))
                  - fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i])))
                  * ydiff2_ptr[i] / ydiff3_ptr[i]
                  + fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]));

    } else {
      // For points located at positions in between grid points, use formula for BLT
      fq_ptr[i] =  (nr1_ptr[i] * fs.at<float>(int(x1_ptr[i]), int(y1_ptr[i]))
                   + nr2_ptr[i] * fs.at<float>(int(x2_ptr[i]), int(y1_ptr[i]))
                   + nr3_ptr[i] * fs.at<float>(int(x1_ptr[i]), int(y2_ptr[i]))
                   + nr4_ptr[i] * fs.at<float>(int(x2_ptr[i]), int(y2_ptr[i]))) / dr_ptr[i];
    }
  }

  xdiff1.release();
  xdiff2.release();
  xdiff3.release();
  ydiff1.release();
  ydiff2.release();
  ydiff3.release();
  nr1.release();
  nr2.release();
  nr3.release();
  nr4.release();
  dr.release();

  return 0;
}


int interp1d(const cv::Mat& x, const cv::Mat& fx, const cv::Mat& xq, cv::Mat& fq) {
  //Mat fq(xq.size(), xq.type());
  Mat mask(x.size(), CV_8U, Scalar(1));
  Mat diff_mat(x.size(), x.type());
  double min_val;
  Point nbr_00, nbr_01;

  int arr_len = xq.cols;
  const float* xq_ptr = xq.ptr<float>(0);
  float* fq_ptr = fq.ptr<float>(0);

  for (size_t i=0; i<arr_len; i++) {
    diff_mat = abs(xq_ptr[i] - x);

    // Locate the 2 closest neighbors to query point in sample point array
    minMaxLoc(diff_mat, &min_val, 0, &nbr_00, 0, mask);
    mask.at<uchar>(nbr_00.y, nbr_00.x) = (uchar)0;
    minMaxLoc(diff_mat, &min_val, 0, &nbr_01, 0, mask);
    mask.at<uchar>(nbr_00.y, nbr_00.x) = (uchar)1;

    float thresh = 0.1; // Set heuristically - play around for better results
    if (abs(x.at<float>(nbr_01.y, nbr_01.x) - x.at<float>(nbr_00.y, nbr_00.x)) < thresh) {
      //if ((nbr_01.x == (x.cols - 1) || (nbr_00.x == (x.cols - 1`))) {
       //fq_ptr[i] = fx.at<float>(0, 0)r;
      //} else {
        // If 2 neighbors are too close, take function value from one of the existing pts
        fq_ptr[i] = fx.at<float>(nbr_00.y, nbr_00.x); 
      //}

      //if (i == arr_len -1) {
      //cout << "Inside If" << endl;
      //cout << x.at<float>(nbr_01.y, nbr_01.x) << endl;
      //cout << x.at<float>(nbr_00.y, nbr_00.x) << endl;
      //cout << fq_ptr[i] << endl;
       //}
    } else {
      // If not apply 1D interpolation function
      fq_ptr[i] = (fx.at<float>(nbr_01.y, nbr_01.x) - fx.at<float>(nbr_00.y, nbr_00.x))
                  * (xq_ptr[i] - x.at<float>(nbr_00.y, nbr_00.x))
                  / (x.at<float>(nbr_01.y, nbr_01.x) - x.at<float>(nbr_00.y, nbr_00.x))
                  + fx.at<float>(nbr_00.y, nbr_00.x);
    }
  }
  return 0;
}


int calcDist(const cv::Mat& x, cv::Mat& dx) {

  if (x.cols < 5) {
    cout << "Error! Contour too small! " << endl;
    return -1;
  }

  const float* x_ptr = x.ptr<float>(0);
  float* dx_ptr = dx.ptr<float>(0);

  size_t x_len = x.cols;
  for (size_t i=0; i<x_len; i++) {
    dx_ptr[i] = abs(x_ptr[i] - x_ptr[i+1]);
  }

  // for closed contour, use first and last point to compute dx[0]
  dx_ptr[x_len-1] = abs(x_ptr[0] - x_ptr[x_len-1]);
  return 0;
}


//--Function to return values of a 1D function at specific query points based on sample function values--//
//--using Linear Interpolation//
Mat interp1d(Mat x, Mat fx, Mat xq) { 

//--x: Independent Variable; f(x): Dependent Variable;
//--xq: Query points; fq(xq) is calculated using this function--//
	Mat fq(xq.size(), xq.type());
	
	Mat Mask(x.size(), CV_8U, Scalar(1));			//Mask image used in finding second minimum point
	double min_val;
	Point min_pt1, min_pt2;
	Mat diff_mat(x.size(), x.type());				//Matrix to hold distance between xq and x values
    
	int col_count = xq.cols;	//Length of vector containing query points
	float* xq_ptr, *fq_ptr;
	xq_ptr = xq.ptr<float>(0);
	fq_ptr = fq.ptr<float>(0);
	
	for(int i = 0; i <  col_count; i++) {

		Mat ones_vector = Mat::ones(x.size(), x.type()); //Temporary vector used during distance value computation
		diff_mat = abs(xq_ptr[i] - x); //Calcualte distance between query point and all x's
		//multiply(ones_vector, xq_ptr[i], ones_vector);
		//absdiff(ones_vector,  x, diff_mat);
				
		//Find the two closest neighbours to xq and interpolate (or extrapolate)
		minMaxLoc(diff_mat, &min_val, 0, &min_pt1, 0, Mask);
		Mask.at<uchar>(min_pt1.y, min_pt1.x) = (uchar)0;
		minMaxLoc(diff_mat, &min_val, 0, &min_pt2, 0, Mask);
		Mask.at<uchar>(min_pt1.y, min_pt1.x) = (uchar)1;
		
		if((x.at<float>(min_pt2.y, min_pt2.x) - x.at<float>(min_pt1.y, min_pt1.x)) < 0.1) {
			
			if(min_pt1.x == (x.cols - 1))
				fq_ptr[i] = fx.at<float>(0, 0);
			else 
				fq_ptr[i] = fx.at<float>(min_pt1.y, min_pt1.x);
			
		}else 
			fq_ptr[i] = (fx.at<float>(min_pt2.y, min_pt2.x) - fx.at<float>(min_pt1.y, min_pt1.x))
								* (xq_ptr[i] - x.at<float>(min_pt1.y, min_pt1.x)) / (x.at<float>(min_pt2.y, min_pt2.x) - x.at<float>(min_pt1.y, min_pt1.x))
								+ fx.at<float>(min_pt1.y, min_pt1.x);	

		//cout << "Interpolated x: " << fq.at<float>(i, 0) << ", ";

	}
		
	return fq;
}
