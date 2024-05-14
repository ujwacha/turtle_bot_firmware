#include <arm_math.h>
class Kinematics{
	private:
		float motor_omegas[4];
		double inv_matrix[4][3];
		float wheel_radius;
		float base_radius;
		float Vx,Vy,omega;

	public:
		float v0,v1,v2,v3;

		Kinematics() {
			// fill up the kinematics matrix
			inv_matrix[0][0] = -0.3535533905932738;
			inv_matrix[0][1] = 0.3535533905932737;
			inv_matrix[0][2] = 0.25;

			inv_matrix[1][0] = 0.3535533905932738;
			inv_matrix[1][1] = 0.3535533905932738;
			inv_matrix[1][2] = -0.25;

			inv_matrix[2][0] = -0.3535533905932738;
			inv_matrix[2][1] = 0.3535533905932738;
			inv_matrix[2][2] = -0.25;

			inv_matrix[3][0] = 0.3535533905932738;
			inv_matrix[3][1] = 0.3535533905932738;
			inv_matrix[3][2] = 0.25;
		}

		void set_value(float velocity,float theta_deg, float omega_inp)
		{
			Vx = velocity * arm_sin_bs(theta_deg);
			Vy = velocity * arm_cos_bs(theta_deg);
			omega = omega_inp;
		}

		void get_motor_omegas()
		{
			float twist[3];
			twist[0] = Vx; // get Vx
			twist[1] = Vy; // get Vy
			twist[2] = omega * base_radius; // get omega

			for (int i=0;i<4;i++) {
				for(int j=0;j<3;j++)
				{
					motor_omegas[i] += inv_matrix[i][j] * twist[j]; 
				}
				motor_omegas[i] /= wheel_radius;
				v0 = motor_omegas[0];
				v1 = motor_omegas[1];
				v2 = motor_omegas[2];
				v3 = motor_omegas[3];
			}
		}
};   
