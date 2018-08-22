
#include "my_lib.h"

Matrix3d my_sqrt(Matrix3d A)
{
    return A.sqrt();
}


Vector3d motion_command(Vector3d mu, double dl, double dr)
{
	double dc = (dl+dr)/2;
	double theta = mu(2), newTheta = theta+(dr-dl)/L;
	newTheta = normalize_angle(newTheta);
	double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

	mu(0) += dc*cos(avTheta);
	mu(1) += dc*sin(avTheta);
	mu(2) = newTheta;
	
	return mu;
}

Matrix3d compute_jacobian(Vector3d mu, double dl, double dr)
{
	double dc = (dl+dr)/2;
	double theta = mu(2), newTheta = theta+(dr-dl)/L;
	newTheta = normalize_angle(newTheta);
	double avTheta = atan2(sin(newTheta)+sin(theta), cos(newTheta)+cos(theta)); 

	Matrix3d G;
	G << 1, 0, -dc*sin(avTheta),
		 0, 1, dc*cos(avTheta),
		 0, 0, 1;	 
		 
	return G;
}

double normalize_angle(double theta)
{
	return atan2(sin(theta), cos(theta));
}
