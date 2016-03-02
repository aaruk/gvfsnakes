#include <snakes.hpp>
#include <mathOps.hpp>

using namespace std;
using namespace cv;

namespace snake {

int deformSnake(cv::Mat& x, cv::Mat& y,
                const cv::Mat& ext_fx,
                const cv::Mat& ext_fy,
                const float& alpha,
                const float& beta,
                const float& gamma,
                const float& kappa,
                const int& iter) {

  int snake_len = x.cols;

  float a = gamma * (2 * alpha + 6 * beta) + 1;
  float b = gamma * (- alpha - 4 * beta);
  float c = beta * gamma;

  Mat ldiag(snake_len, 1, CV_32F, Scalar(a));
  Mat pd_mat = Mat::diag(ldiag);

  int last_elem = snake_len - 1; 
  float* pd_ptr;
  // Populate elements of the NxN penta-diagonal matrix
  // Where N, is the length of snake
  for (size_t i = 0; i < snake_len; i++) {
     pd_ptr = pd_mat.ptr<float>(i);
     if (i == 0) {
       pd_ptr[i+1] = b;
       pd_ptr[i+2] = c;
       pd_ptr[last_elem-1] = c;
       pd_ptr[last_elem] = b;
     } else if (i == 1) {
       pd_ptr[i-1] = b;
       pd_ptr[i+1] = b;
       pd_ptr[i+2] = c;
       pd_ptr[last_elem] = c;
     } else if ((i > 1) && (i < last_elem-1)) {
       pd_ptr[i-2] = c;
       pd_ptr[i-1] = b;
       pd_ptr[i+1] = b;
       pd_ptr[i+2] = c;
     } else if (i == last_elem-1) {
       pd_ptr[i-1] = b;
       pd_ptr[i+1] = b;
       pd_ptr[i-2] = c;
       pd_ptr[0] = c;
     } else {
       pd_ptr[i-1] = b;
       pd_ptr[i-2] = c;
       pd_ptr[0] = b;
       pd_ptr[1] = c;
     }
  }

  cout << "After Penta Diag " << endl;
  // Compute matrix inverse: (I-gamma*pd_mat)^(-1)
  Mat i_gamma_pd_inv = pd_mat.inv();
  Mat fxq, fyq;

  // Deform snake for the specified no of iterations
  // Update snake coordinates based on value of external constraint function
  // at each coordinate of x and y
  for (size_t i = 0; i < iter; i++) {

    // Get x and y components of external constraint force
    // at snake location
    interp2d(y, x, ext_fx, fxq);
    interp2d(y, x, ext_fy, fyq);

    x = (x + kappa*gamma + fxq) * i_gamma_pd_inv;
    y = (y + kappa*gamma + fyq) * i_gamma_pd_inv;
  }

 cout << "Snake Deformed " << endl;
 return 0;
}


int interpolateSnake(const float dmin, const float dmax, cv::Mat& x, Mat& y) {
  Mat dx, dy, d; int j;
  calcDist(x, dx); 
  calcDist(y, dy); 
  d = dx+dy;

  //-- Find and eliminate points located closer than dmin
  Mat x_dmin(x.size(), x.type());
  Mat y_dmin(y.size(), y.type());
  float* x_dmin_ptr, *y_dmin_ptr, *x_ptr, *y_ptr, *d_ptr;
  x_ptr = x.ptr<float>(0);
  y_ptr = y.ptr<float>(0);
  x_dmin_ptr = x_dmin.ptr<float>(0);
  y_dmin_ptr = y_dmin.ptr<float>(0);
  d_ptr = d.ptr<float>(0);
  size_t loop_term = x.cols;
  int valid_pt_count = 0;

  cout << "Snake Cols " << x.cols << endl;
  // Go through dist vector and flag points that are too close
  for (size_t i=0; i<loop_term; i++) {
    if (d_ptr[i] < dmin) {
      x_dmin_ptr[i] = float(-1);
      y_dmin_ptr[i] = float(-1);
    } else {
      x_dmin_ptr[i] = x_ptr[i];
      y_dmin_ptr[i] = y_ptr[i];
      valid_pt_count++;
    }
  }

  // Create a new matrix and copy only valid points from original array
  Mat x_new (1, valid_pt_count, CV_32F);
  Mat y_new (1, valid_pt_count, CV_32F);
  float* x_new_ptr = x_new.ptr<float>(0);
  float* y_new_ptr = y_new.ptr<float>(0);

  for (size_t i=0; i<loop_term; i++) {
    if (x_dmin_ptr[i] == -1) {
      continue;
    } else {
      x_new_ptr[i] = x_ptr[i];
      y_new_ptr[i] = y_ptr[i];
    }
  }

  // Find points that are too far away [d > dmax]
  Mat dmax_indicator, ind_var, q_pts;
  double max_d;

  float* ind_var_ptr;
  cout << "Check " <<  endl;
  //cout << dx.cols << endl;
  //cout << dy.cols << endl;
  //cout << x_new.cols << endl;
  //cout << y_new.cols << endl;
  calcDist(x_new, dx);
  calcDist(y_new, dy);
  d = dx + dy;
  d_ptr = d.ptr<float>(0);
  minMaxLoc(d, 0, &max_d, 0, 0);
  Mat xq_new, yq_new;

  while (max_d > dmax) {
    dmax_indicator.create(x.size(), CV_32S);
    int* dmax_ptr = dmax_indicator.ptr<int>(0);
    loop_term = dmax_indicator.cols;
    ind_var.create(x_new.size(), x_new.type());

    for (size_t i=0; i<loop_term; i++) {
      if (d_ptr[i] > dmax)
        dmax_ptr[i] = 1;
      else
        dmax_ptr[i] = 0;
    }

    // indices for x
    ind_var.create(x_new.size(), x_new.type());
    ind_var_ptr = ind_var.ptr<float>(0);

    cout << "before " << endl;
    size_t ind_var_siz = xq_new.cols;
    for (size_t i=0; i<loop_term; i++) {
      ind_var_ptr[i] = float(i);
    }

    q_pts = getInterpIndex(dmax_indicator);
    cout << "after" << endl;
    xq_new.create(q_pts.size(), x_new.type());
    yq_new.create(q_pts.size(), x_new.type());
    interp1d(ind_var, x_new, q_pts, xq_new);
    interp1d(ind_var, y_new, q_pts, yq_new);
    x_new.release(); y_new.release();
    x_new = xq_new; y_new = yq_new;

    if (x_new.cols < 10) {
      cout << "Error! Contour too small!" << endl;
      break;
    }

    calcDist(x_new, dx);
    calcDist(y_new, dy);
    d = dx + dy;
    d_ptr = d.ptr<float>(0);
    minMaxLoc(d, 0, &max_d, 0, 0);
  }
  x = x_new;
  y = y_new;
  return 0;
}

//--Function to calculate new indices to interpolate--//
/*Mat getInterpIndex(const Mat& inp_indicator) {

  int y_col_count = 2 * inp_indicator.cols;
  Mat y(1, y_col_count, CV_32F);
  //float* y_ptr = y.ptr<float>(0);

  // Create indexes inbetween all points (this will be the index of x/y coordinates)
  for (size_t i = 0; i < y_col_count; i++) {
    //y_ptr[i] = 0.5 * i;
    y.at<float>(0, i) = 0.5*i;
  }

  // Create new array to hold index points temporarily
  //float y_new[2000];
  vector<float> y_new(2000);
  int count = 0;

  cout << "Input Cols " << inp_indicator.cols << endl;
  // Iterate over the inp array and keep new points 
  for (size_t i = 0; i < inp_indicator.cols; i++) {
    //(Eg: point 1.5 is retained if d(1, 2) > dmax)
    // where distance between 2 points is greater than dmax
    y_new[count++] = y.at<float>(0, 2*i);//y_ptr[2*i + 0];
    // Odd locations in y hold decimal values, even locations the whole numbers
    if (inp_indicator.at<int>(0, i) == 1) {  
      y_new[count++] = y.at<float>(0, 2*i + 1);//y_ptr[2*i + 1]; 
    }
  }

  cout << "Check2 " << endl;
  // Create new matrix and copy values(indices) from array to matrix
  Mat qp(1, count, CV_32F);
  //q_pts.create(1, count, CV_32F);
  for (size_t i = 0; i < count; i++) {
    qp.at<float>(0, i) = y_new[i];
  }
  cout << "Check3" << endl;
  cout << qp.size() << endl;
  cout << "InterpIndex Done! " << endl;
  return qp;
}*/
Mat getInterpIndex(const Mat &IDX) {

	int y_col_count = 2 * IDX.cols;
	Mat y(1, y_col_count, CV_32F);

	//Create indexes inbetween all points (this will be the index of x/y coordinates)
	for(int i = 0; i < y_col_count; i++) {
		y.at<float>(0, i) = 0.5 * i;
	}

	//Create new array to hold index points temporarily
	float y_new[2000];
	int count = 0;

	for(int i = 0; i < IDX.cols; i++) { //Iterate over the IDX array and keep new points 
										//(Eg: point 1.5 is retained if d(1, 2) > dmax)
										//where distance between 2 points is greater than dmax
		
		y_new[count++] = y.at<float>(0, 2*i + 0);
		
		if(IDX.at<int>(0, i) == 1)		//odd locations in y hold decimal values, even locations the whole numbers
		{
			y_new[count++] = y.at<float>(0, 2*i + 1); 
		}
	}

	Mat y1(1, count, CV_32F); //Create new matrix and copy values(indices) from array to matrix

	for(int i = 0; i < count; i++) {

		y1.at<float>(0, i) = y_new[i];
		//cout << "New indices: " << y1.at<float>(i, 0) << ", ";
	}
	
	return y1;
}


//Function to normalize a given matrix//
void normalizeMat(Mat& fx, Mat& fy) {

	Mat mag(fx.size(), CV_32F);				
	Mat prod_temp1, prod_temp2;
		
	//Magnitude Computation
	multiply(fx, fx, prod_temp1);
	multiply(fy, fy, prod_temp2);
	mag = prod_temp1 + prod_temp2;
	sqrt(mag, mag);

	fx = fx / mag; fy = fy / mag;
}

//--Diffuse gradient Vectors across image--//
//--[fx, fy] - Gradient components;
//--[u v] - Diffused Gradients
//--mu - Regularization parameter: If image has more noise increase mu
//--dt - time step: must satisfy the Courant-Fruedrichs-Lewy restriction
//--dt <= (dx)(dy)/(4mu) Eg: If dx = dy = 1; mu = 0.5; dt must be <= 0.5
void gradVectorField(Mat& fx, Mat& fy, Mat& u, Mat& v, float mu, int ITER) {
  
  fx.copyTo(u);
  fy.copyTo(v);
  Mat gradX, gradY;
  Mat sqrMag, fx2, fy2;
  Mat u_term2, v_term2;
  
  multiply(fx, fx, fx2);
  multiply(fy, fy, fy2);
  sqrMag = fx2 + fy2;
  
  int dt;
  dt = 1;
  
  //Solve the differential equation iteratively//
  //Diffusion function being implemented is from the paper Snakes, Shapes and Gradient Vector FLow 
  //by Chenyang Xu and Jerry L Prince
  // u = u + mu * del2(u) -(u - fx) * (fx2 + fy2);
  // v = v + mu * del2(v) - (v - fy) * (fx2 + fy2);
  for(int i = 0; i < ITER; i++) {
     Laplacian(u, gradX, CV_32F, 1, BORDER_REFLECT);
     Laplacian(v, gradY, CV_32F, 1, BORDER_REFLECT);
     
     multiply((u- fx), sqrMag, u_term2);
     multiply((v- fy), sqrMag, v_term2);
     
     u = u + dt * (mu * gradX - u_term2) ;
     v = v + dt * (mu * gradY - v_term2) ;
  }
  normalizeMat(u, v);
}

}
