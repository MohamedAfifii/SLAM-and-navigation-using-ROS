

#include <unsupported/Eigen/MatrixFunctions>	//This library takes much time to compile, that's why I put it in a separate file so that it complies only once.

#include <cmath>
using namespace Eigen;

#define loop(n)			for(int i = 0; i < n; i++)
#define lp(i, s, e)		for(int i = s; i < e; i++)
#define lpe(i, s, e)	for(int i = s; i <= e; i++)


#define L  0.27    //Distance between the two wheels of the robot	

Matrix3d my_sqrt(Matrix3d A);
Vector3d motion_command(Vector3d mu, double dl, double dr);
Matrix3d compute_jacobian(Vector3d mu, double dl, double dr);
double normalize_angle(double theta);
