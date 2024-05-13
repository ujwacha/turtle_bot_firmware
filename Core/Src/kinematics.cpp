#include "joystick.hpp"

class kinematics{
	private:
		float motor_omegas[4];
		double inv_matrix[4][3];
		float wheel_radius;
		float base_radius;
	public:
		float get_motor_omegas()
		{
			float twist[3];
			twist[0] = joystick.get_twist(0); // get Vx
			twist[1] = joystick.get_twist(1); // get Vy
			twist[2] = joystick.get_twist(2) * base_radius; // get omega

			for (int i=0;i<4;i++) {
				for(int j=0;j<3;j++)
				{
					motor_omegas[i] += inv_matrix[i][j] * twist[j]; 
				}
				motor_omegas[i] /= wheel_radius;
			}

			return *motor_omegas;
		}
};   


