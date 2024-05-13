#include "joystick.hpp"

class kinematics:
{
	private:
		float motor_omegas[4];
		double inv_matrix[4][3];
	public:
		float get_motor_omegas()
		{
			float twist[3];
			twist[0] = joystick.get_twist(0);
			twist[1] = joystick.get_twist(1);
			twist[2] = joystick.get_twist(2);

			for (i=0;i<4;i++) {
				motor_omegas[0] 			
			}
		}
}   


