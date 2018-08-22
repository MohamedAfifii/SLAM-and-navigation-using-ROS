
//Perform a kalman filter udpate step and return the current estimate of the pose

#include <Eigen/Dense>
#include "my_lib.h"
using namespace Eigen;


Vector3d k, mu = Vector3d::Zero();
Matrix3d G, sigma = Matrix3d::Zero(), R = 1e-3*Matrix3d::Identity(), I = Matrix3d::Identity();
RowVector3d c(0,0,1);
double Q = 1e-6;

void ekf_prediction_step(double dl, double dr)
{
	//Update mu
	mu = motion_command(mu, dl, dr);

	//Update sigma
	G = compute_jacobian(mu, dl, dr);
	sigma = G*sigma*G.transpose() + R;
}


double n = 3, lambda = 1;

void ukf_prediction_step(double dl, double dr)
{
	//Compute sigma points
	Matrix3d m = my_sqrt((n+lambda)*sigma);
	Matrix<double, 3, 7> sigma_points;
	sigma_points << mu, m.colwise()+mu, m.colwise()-mu;
	lp(j, 0, 7)	sigma_points(2, j) = normalize_angle(sigma_points(2,j));
	
	//Transform
	lp(j, 0, 7)	
	{
		Vector3d x;
		lp(i, 0, 3)	x(i) = sigma_points(i, j);
		x = motion_command(x, dl, dr);
		lp(i, 0, 3)	sigma_points(i, j) = x(i);
	}
	
	//Compute weights
	Matrix<double, 1, 7> w;
	w(0) = lambda/(n+lambda);
	lp(j, 1, 7)	w(j) = 1/(2*(n+lambda));
	
	//Recover mu
	mu = Vector3d::Zero();
	lp(j, 0, 7)	lp(i, 0, 2)	mu(i) += w(j)*sigma_points(i, j);
	double cosines = 0, sines = 0;
	lp(j, 0, 7)	cosines += w(j)*cos(sigma_points(2,j)), sines += w(j)*sin(sigma_points(2,j));
	mu(2) = atan2(sines,cosines);
	
	//Recover sigma
	sigma = Matrix3d::Zero();
	lp(j, 0, 7)
	{
		Vector3d x;
		lp(i, 0, 3)	x(i) = sigma_points(i, j);
		
		Vector3d deviation = x - mu;
		deviation(2) = normalize_angle(deviation(2));
		sigma += w(j)*deviation*deviation.transpose();
	}
	sigma += R;
}	


void kf_correction_step(double z)
{
	double s = c*sigma*c.transpose() + Q;
	k = sigma*c.transpose()/s;
	mu += k*(z-c*mu);
	sigma = (I-k*c)*sigma;
}


Vector3d kalman_filter(const geometry_msgs::Vector3 &wheels_feedback, double z)
{
	double dl = wheels_feedback.x/100, dr = wheels_feedback.y/100;
	ekf_prediction_step(dl, dr);
	kf_correction_step(z);
	return mu;
}
