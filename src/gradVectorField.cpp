#include"gvfSnake.hpp"

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
