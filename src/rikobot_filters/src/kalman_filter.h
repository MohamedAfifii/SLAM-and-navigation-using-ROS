
//Perform a kalman filter udpate step and return the current estimate of the pose

#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

#define L  0.27    //Distance between the two wheels of the robot

Vector3d k, mu = Vector3d::Zero();
Matrix3d G, sigma = Matrix3d::Zero(), R = 1e-3*Matrix3d::Identity(), I = Matrix3d::Identity();
RowVector3d c(0,0,1);
double Q = 1e-6;


void ekf_prediction_step(double dl, double dr)
{
	//Update mu
	double dc = (dl+dr)/2;
	double theta = mu(2), newTheta = theta+(dr-dl)/L;
	newTheta = atan2(sin(newTheta), cos(newTheta));
	double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

	mu(0) += dc*cos(avTheta);
	mu(1) += dc*sin(avTheta);
	mu(2) = newTheta;

	//Update sigma
	G << 1, 0, -dc*sin(avTheta),
		 0, 1, dc*cos(avTheta),
		 0, 0, 1;	 
		 
	sigma = G*sigma*G.transpose() + R;
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
