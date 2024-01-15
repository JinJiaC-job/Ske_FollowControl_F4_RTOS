#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "lkmoto.h"
#include "ids830can.h"
#include "fourier_series_traj_exciting.h"
#include "tim.h"
#include "Sensors_reading.h"

//电机控制时间节点：
unsigned int motor_control_k = 0;
//motor control interval time：每过motor control interval time秒输出一次控制命令
float control_interval_time = 0.05;
// sampling period
float traj_Ts = 0.1;
// trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
float traj_f = 0.05;
// trajectory fundamental frequency in radian
float traj_wf;
// number of sampling points
uint8_t traj_n;
// order of trajectory generation 
uint8_t traj_order = 5;
// number of revolute joints
uint8_t dof = 6;
// 6个关节角度信息
float q[7] = {0};
float qd[7] = {0};
float qdd[7] = {0};
float q_last[7] = {0, 0, 0, 0, 0, 0, 0};
float q_next[7] = {0};
// fourier series params
float traj_param[] = {0,
0.13249,
0.84265,
-0.61763,
-0.62935,
1.1823,
0.51965,
-1.0063,
-0.18653,
0.3091,
-0.079349,
97.383,
-0.16202,
0.037126,
0.14317,
-0.0095187,
-0.13003,
-0.12242,
0.10733,
0.0451,
0.041543,
0.033753,
0.030514,
0.062815,
-0.13427,
0.048746,
0.018498,
-0.025162,
0.093033,
0.33808,
-0.088307,
-0.42448,
0.034281,
-0.34769,
0.04826,
-0.006213,
0.054578,
0.26625,
0.16519,
-0.11289,
-0.12732,
0.040054,
-0.14071,
-0.069567,
-1.299,
0.17329,
0.045145,
-0.14034,
-0.011873,
0.36856,
-0.12058,
0.016315,
0.32496,
-0.41783,
-0.1919,
1.7041,
-0.18458,
0.093306,
-0.21073,
-0.039703,
-0.22417,
-0.07346,
0.72046,
0.13674,
-0.10098,
-0.068099,
0.22133};

int sign(float value)
{
	if(value > 0)
		return 1;
	else if(value == 0)
		return 0;
	else
		return -1;
}
//动力学各关节扭矩值（需要更改）
float TAU[7] = {0};
float compute_TAU(float *q, float *qd, float *qdd, int id)
{
	switch(id)
	{
		case 1:
			TAU[id] = 0.0006731*qd[1] - 1.386e-5*sign(qd[1]) + sin(q[2])*(4411.0*sin(q[2]) + cos(q[3])*(sin(q[4])*(4.344*cos(q[2])*cos(q[4]) + sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 4.411*cos(q[3])*sin(q[2])*sin(q[4]) + 0.7659*cos(q[4])*sin(q[2])*sin(q[3])) + 4.411*cos(q[3])*sin(q[2]) - 1.0*cos(q[4])*(4.344*cos(q[2])*sin(q[4]) + cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.7659*sin(q[2])*sin(q[3])*sin(q[4]) - 4.411*cos(q[3])*cos(q[4])*sin(q[2]))) - 1.0*sin(q[3])*(1.509*cos(q[2]) - 12.97*sin(q[2])*sin(q[3]) + 0.0004432*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0004432*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.1736*sin(q[4])*(4.344*cos(q[2])*sin(q[4]) + cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.7659*sin(q[2])*sin(q[3])*sin(q[4]) - 4.411*cos(q[3])*cos(q[4])*sin(q[2])) - 0.1736*cos(q[4])*(4.344*cos(q[2])*cos(q[4]) + sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 4.411*cos(q[3])*sin(q[2])*sin(q[4]) + 0.7659*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[2])*(4416.0*cos(q[2]) - 1.509*sin(q[2])*sin(q[3]) + 7.814e-5*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 7.814e-5*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.9848*sin(q[4])*(4.344*cos(q[2])*sin(q[4]) + cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.7659*sin(q[2])*sin(q[3])*sin(q[4]) - 4.411*cos(q[3])*cos(q[4])*sin(q[2])) + 0.9848*cos(q[4])*(4.344*cos(q[2])*cos(q[4]) + sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 4.411*cos(q[3])*sin(q[2])*sin(q[4]) + 0.7659*cos(q[4])*sin(q[2])*sin(q[3]))) - 4428.0;
			break;
		case 2:
			TAU[id] = 4.864*qd[2] - 0.1052*cos(q[2]) - 1452.0*sin(q[2]) + 0.9602*sign(qd[2]) + 81.24*cos(q[3])*(sin(q[4])*(4.344*cos(q[2])*cos(q[4]) + sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 4.411*cos(q[3])*sin(q[2])*sin(q[4]) + 0.7659*cos(q[4])*sin(q[2])*sin(q[3])) + 4.411*cos(q[3])*sin(q[2]) - 1.0*cos(q[4])*(4.344*cos(q[2])*sin(q[4]) + cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.7659*sin(q[2])*sin(q[3])*sin(q[4]) - 4.411*cos(q[3])*cos(q[4])*sin(q[2]))) + cos(q[3])*(2157.0*cos(q[2])*cos(q[4]) - 1.108*cos(q[2]) + 0.2235*cos(q[2])*sin(q[4]) - 0.1736*cos(q[4])*(1094.0*sin(q[2])*sin(q[3]) - 1.0*cos(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 192.8*cos(q[2]) + 0.1136*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 12.15*cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*sin(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 0.1136*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 12.15*sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) - 0.04963*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 248.7*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 0.04963*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.1736*sin(q[4])*(0.04001*cos(q[2]) - 1.0*sin(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.2269*sin(q[2])*sin(q[3]) - cos(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 12.15*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 12.15*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) - 0.9848*cos(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 1.625e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) + 0.9848*sin(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.04955*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.04955*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) + 1.625e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) - 248.7*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.03941*sin(q[2])*sin(q[3])*sin(q[4]) - 0.2269*cos(q[3])*cos(q[4])*sin(q[2]) + 2190.0*cos(q[3])*sin(q[2])*sin(q[4]) + 380.4*cos(q[4])*sin(q[2])*sin(q[3])) + sin(q[3])*(sin(q[4])*(1094.0*sin(q[2])*sin(q[3]) - 1.0*cos(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 192.8*cos(q[2]) + 0.1136*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 12.15*cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*sin(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 0.1136*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 12.15*sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) - 109.3*cos(q[2]) + cos(q[4])*(0.04001*cos(q[2]) - 1.0*sin(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.2269*sin(q[2])*sin(q[3]) - cos(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 12.15*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 12.15*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))))) - 81.24*sin(q[3])*(1.509*cos(q[2]) - 12.97*sin(q[2])*sin(q[3]) + 0.0004432*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0004432*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.1736*sin(q[4])*(4.344*cos(q[2])*sin(q[4]) + cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.7659*sin(q[2])*sin(q[3])*sin(q[4]) - 4.411*cos(q[3])*cos(q[4])*sin(q[2])) - 0.1736*cos(q[4])*(4.344*cos(q[2])*cos(q[4]) + sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 4.411*cos(q[3])*sin(q[2])*sin(q[4]) + 0.7659*cos(q[4])*sin(q[2])*sin(q[3])));
			break;
		case 3:
			TAU[id] = 0.7089*qd[3] + 0.08565*sign(qd[3]) - 380.4*cos(q[2])*cos(q[4]) - 109.3*cos(q[3])*sin(q[2]) - 0.03941*cos(q[2])*sin(q[4]) - 0.9848*cos(q[4])*(1094.0*sin(q[2])*sin(q[3]) - 1.0*cos(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 192.8*cos(q[2]) + 0.1136*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 12.15*cos(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 1.0*sin(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 0.1136*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 12.15*sin(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 1.108*sin(q[2])*sin(q[3]) + 0.008752*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 43.85*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - 0.008752*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.9848*sin(q[4])*(0.04001*cos(q[2]) - 1.0*sin(q[5])*(0.01592*sin(q[2])*sin(q[3]) - 0.002808*cos(q[2]) - 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.2269*sin(q[2])*sin(q[3]) - cos(q[5])*(485.7*sin(q[2])*sin(q[3]) - 85.65*cos(q[2]) + 0.0504*sin(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.0504*cos(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + cos(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 12.15*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) + 12.15*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])))) + 0.1736*cos(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 2.864e-7*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.1736*sin(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.008738*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) - 0.008738*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 2.864e-7*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 43.85*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.006948*sin(q[2])*sin(q[3])*sin(q[4]) + 0.04001*cos(q[3])*cos(q[4])*sin(q[2]) - 386.2*cos(q[3])*sin(q[2])*sin(q[4]) - 67.07*cos(q[4])*sin(q[2])*sin(q[3]);
			break;
		case 4:
			TAU[id] = 2.897*qd[4] + 1.382*sign(qd[4]) - 2190.0*cos(q[2])*cos(q[4]) - 0.2269*cos(q[2])*sin(q[4]) + 0.0504*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 252.5*sin(q[5])*(0.00045*cos(q[6])*(sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) - 0.00045*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 0.00045*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00045*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - 0.0504*sin(q[6])*(cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])))) + 1.0*cos(q[6])*(9.983e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 9.983e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 1.65e-6*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - sin(q[6])*(8.209e-5*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.05032*cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) - 0.05032*sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 1.65e-6*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + 252.5*cos(q[5])*(0.0009*cos(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0009*sin(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.04001*sin(q[2])*sin(q[3])*sin(q[4]) + 0.2304*cos(q[3])*cos(q[4])*sin(q[2]) - 2224.0*cos(q[3])*sin(q[2])*sin(q[4]) - 386.2*cos(q[4])*sin(q[2])*sin(q[3]);
			break;
		case 5:
			TAU[id] = 0.236929*qd[5] + 0.177573*sign(qd[5]) - 1.0*qdd[1]*(0.0503186*sin(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) - 0.00000164958*cos(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) - 1.0*cos(q[6])*(0.00000998342*cos(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) - 0.00000998342*sin(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.0504*cos(q[6])*(cos(q[6])*(sin(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) + cos(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2]))) + sin(q[6])*(0.173648*cos(q[2]) - 0.984808*sin(q[2])*sin(q[3]))) + 0.0504*sin(q[6])*(cos(q[6])*(0.173648*cos(q[2]) - 0.984808*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(sin(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) + cos(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2])))) + sin(q[6])*(0.0000820925*cos(q[5])*(0.984808*cos(q[2])*cos(q[4]) + cos(q[3])*sin(q[2])*sin(q[4]) + 0.173648*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0000820925*sin(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.0503186*cos(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2])) + 0.00000164958*sin(q[5])*(0.984808*cos(q[2])*sin(q[4]) + 0.173648*sin(q[2])*sin(q[3])*sin(q[4]) - 1.0*cos(q[3])*cos(q[4])*sin(q[2]))) + 0.0504*cos(q[6])*(sin(q[6])*(1702.1*cos(q[2]) - 9653.09*sin(q[2])*sin(q[3])) + cos(q[6])*(cos(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])))) + qdd[3]*(0.0504*cos(q[6])*(0.179656*cos(q[4])*cos(q[5]) - 0.179656*sin(q[4])*sin(q[5]) - 1.0*sin(q[6])*(248.664*cos(q[4]) + 110.298*cos(q[4])*sin(q[5]) + 110.298*cos(q[5])*sin(q[4])) + cos(q[6])*(sin(q[5])*(11.9654*sin(q[4]) + 43.8462) - 11.9654*cos(q[4])*cos(q[5]) + 19.4486)) + 0.625869*cos(q[4])*cos(q[5]) + 0.31687*cos(q[4])*sin(q[5]) + 0.31685*cos(q[5])*sin(q[4]) - 0.0237856*sin(q[4])*sin(q[5]) - 1.0*sin(q[6])*(0.000338637*cos(q[6]) - 0.00157245*sin(q[6]) - 0.149577*cos(q[4])*cos(q[5]) + 0.000982271*cos(q[4])*sin(q[5]) + 0.149577*sin(q[4])*sin(q[5]) + 0.0000820925*cos(q[5])*(11.9654*sin(q[4]) + 43.8462) - 0.00905535*cos(q[6])*(0.984808*cos(q[4])*sin(q[5]) + 0.984808*cos(q[5])*sin(q[4])) - 0.00195013*sin(q[6])*(0.984808*cos(q[4])*sin(q[5]) + 0.984808*cos(q[5])*sin(q[4]))) + 0.00000164958*cos(q[5])*(11.9654*sin(q[4]) + 43.8462) - 0.0503186*sin(q[5])*(11.9654*sin(q[4]) + 43.8462) + 0.0504*sin(q[6])*(0.0218483*cos(q[4])*cos(q[5]) - 0.0218483*sin(q[4])*sin(q[5]) + cos(q[6])*(248.664*cos(q[4]) + 110.298*cos(q[4])*sin(q[5]) + 110.298*cos(q[5])*sin(q[4])) + sin(q[6])*(sin(q[5])*(11.9654*sin(q[4]) + 43.8462) - 11.9654*cos(q[4])*cos(q[5]) + 19.4486)) - 1.0*cos(q[6])*(0.000338637*sin(q[6]) - 0.0508209*cos(q[4])*cos(q[5]) - 0.000119456*cos(q[4])*sin(q[5]) + 0.0508209*sin(q[4])*sin(q[5]) - 0.00000998342*cos(q[5])*(11.9654*sin(q[4]) + 43.8462) + 0.00195013*cos(q[6])*(0.984808*cos(q[4])*sin(q[5]) + 0.984808*cos(q[5])*sin(q[4]))) - 1.06127) - 1.0*qdd[6]*(0.0607992*cos(q[6]) + 0.153002*sin(q[6])) - 0.0504*sin(q[6])*(cos(q[6])*(1702.1*cos(q[2]) - 9653.09*sin(q[2])*sin(q[3])) - 1.0*sin(q[6])*(cos(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])))) + 1.0*cos(q[6])*(0.00000998342*cos(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])) - 0.00000998342*sin(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) + qdd[4]*(0.00041652*cos(q[5]) - 12.7055*sin(q[5]) + 0.0504*cos(q[6])*cos(q[6])*(252.5*sin(q[5]) + 112.0) + 0.0504*sin(q[6])*sin(q[6])*(252.5*sin(q[5]) + 112.0) - 1.0*sin(q[6])*(0.0207284*cos(q[5]) + 0.00195013*cos(q[6]) - 0.00905535*sin(q[6])) + cos(q[6])*(0.00252081*cos(q[5]) - 0.00195013*sin(q[6])) - 6.11159) + 0.00000164958*cos(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])) - sin(q[6])*(0.0000820925*cos(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])) - 0.0000820925*sin(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2]))) - 0.0503186*cos(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) - 0.0503186*sin(q[5])*(9653.09*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.1*cos(q[4])*sin(q[2])*sin(q[3])) + qdd[2]*(6.01875*cos(q[3]) + 0.321738*cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) - 0.0241525*sin(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + sin(q[6])*(0.00192051*cos(q[3])*cos(q[6]) - 0.00891778*cos(q[3])*sin(q[6]) + 0.00905535*cos(q[6])*(cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4]))) - 0.151884*sin(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + 0.00195013*sin(q[6])*(cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4]))) - 0.0000820925*cos(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) + 0.151884*cos(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) - 0.0000820925*sin(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4]))) + 0.00000164958*cos(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) - 0.0503186*sin(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) + 0.0241525*cos(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) + 0.321738*sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) + 0.0503186*cos(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4])) + cos(q[6])*(0.00192051*cos(q[3])*sin(q[6]) - 0.00195013*cos(q[6])*(cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4]))) - 0.0516049*sin(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + 0.00000998342*cos(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) + 0.0516049*cos(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) + 0.00000998342*sin(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4]))) + 0.00000164958*sin(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4])) - 0.0504*cos(q[6])*(0.182428*sin(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + cos(q[6])*(110.298*cos(q[3]) - 1.0*sin(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) + cos(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4]))) - 0.182428*cos(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) + sin(q[6])*(80.0058*sin(q[3]) + 43.8462*cos(q[3])*cos(q[4]) + 112.0*cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) - 252.5*sin(q[3])*sin(q[4]) + 112.0*sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])))) - 0.0504*sin(q[6])*(0.0221854*sin(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) + 1.0*sin(q[6])*(110.298*cos(q[3]) - 1.0*sin(q[5])*(83.3498*cos(q[3])*sin(q[4]) - 248.664*cos(q[3]) + 26.2572*cos(q[4])*sin(q[3])) + cos(q[5])*(83.3498*cos(q[3])*cos(q[4]) - 26.2572*sin(q[3])*sin(q[4]))) - 0.0221854*cos(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4])) - cos(q[6])*(80.0058*sin(q[3]) + 43.8462*cos(q[3])*cos(q[4]) + 112.0*cos(q[5])*(0.173648*cos(q[3])*sin(q[4]) + cos(q[4])*sin(q[3])) - 252.5*sin(q[3])*sin(q[4]) + 112.0*sin(q[5])*(0.173648*cos(q[3])*cos(q[4]) - 1.0*sin(q[3])*sin(q[4]))))) - 0.00000164958*sin(q[5])*(9653.09*cos(q[2])*sin(q[4]) + 1702.1*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) - 1.0*qdd[5]*(0.00195013*cos(q[6])*sin(q[6]) - 5.6448*cos(q[6])*cos(q[6]) - 5.6448*sin(q[6])*sin(q[6]) + sin(q[6])*(0.00195013*cos(q[6]) - 0.00905535*sin(q[6])) + 5.71338);
			break;
		case 6:
			TAU[id] = 0.3693*qd[6] + 0.1184*sign(qd[6]) + 9.983e-6*cos(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 8.209e-5*sin(q[6])*(1702.0*cos(q[2]) - 9653.0*sin(q[2])*sin(q[3])) - 8.209e-5*cos(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3]))) - 9.983e-6*sin(q[6])*(cos(q[5])*(9653.0*cos(q[2])*sin(q[4]) + 1702.0*sin(q[2])*sin(q[3])*sin(q[4]) - 9802.0*cos(q[3])*cos(q[4])*sin(q[2])) + sin(q[5])*(9653.0*cos(q[2])*cos(q[4]) + 9802.0*cos(q[3])*sin(q[2])*sin(q[4]) + 1702.0*cos(q[4])*sin(q[2])*sin(q[3])));
			
		default:printf("compute_TAU id error\n"); 
			break;
		
	}
	return TAU[id];

}

void compensation_ske_GF(void)
{
	//读取各关节角度、角速度
	for(int cpt_id=1; cpt_id<=6; cpt_id++)
	{
		if(cpt_id == 1)
		{
			LinearActuator_read_position(cpt_id);
			LinearActuator_read_CurrentandSpeed(cpt_id);
			pressure_SensorReading();
		}
		else
		{
			read_status2(cpt_id);
			read_angle(cpt_id);
		}
		q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f;
		qd[cpt_id] = motor_speed_float[cpt_id] * pi /180.0f;
		qdd[cpt_id] = 0;
		
	}
	//计算各关节补偿扭矩大小
	//note: joint 4 torque control current = -compute torque; according to the diffence between DH model and actual mechanical model;
	for(int i=1; i<=6; i++)
	{
		compute_TAU(q, qd, qdd, i);
	}
	//进行扭矩控制
	for(int ctr_id=1; ctr_id<=6; ctr_id++)
	{
		if(ctr_id == 4)
			torque_close_loop(ctr_id, -TAU[ctr_id]);
		else
			torque_close_loop(ctr_id, TAU[ctr_id]);
	}
}

void compensation_singleJoint_GF_angle(int joint_id)
{
	//读取各关节角度、角速度
	for(int cpt_id=1; cpt_id<=6; cpt_id++)
	{
		if(cpt_id == 1)
		{
			LinearActuator_read_position(cpt_id);//单位：mm
			LinearActuator_read_CurrentandSpeed(cpt_id);
			// pressure_SensorReading();
		}
		else
		{
			read_status2(cpt_id);//单位：du
			read_angle(cpt_id);//单位：du
		}
		
		switch (cpt_id)
		{
		case 1:
			/* 直线电机；DH初始位置joint1：95.35mm */
			q[cpt_id] = motorAngle_float[cpt_id] + 95.35;//单位：rad
			qd[cpt_id] = 0;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		case 4:
			/* DH初始位置joint4：-pi/2 */
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f - pi/2;//单位：rad
			qd[cpt_id] = 0;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		case 5:
			/* DH初始位置joint5： pi/2 ；有8.2du的偏差*/
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f + (90-8.2)/180*pi;//单位：rad
			qd[cpt_id] = 0;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		default:
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f;//单位：rad
			qd[cpt_id] = 0;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		}
	}
	
	compute_TAU(q, qd, qdd, joint_id);

	// switch(joint_id)
	// {
	// 	case 5: 
	// 		// if(q[joint_id] < -1*pi/9-8.2/180*pi || q[joint_id] > 11*pi/18-8.2/180*pi)
	// 		if(q[joint_id] < (-20-8.2)/180*pi || q[joint_id] > (210-8.2)/180*pi)
	// 		{
	// 			torque_close_loop(joint_id, -TAU[joint_id]);
	// 		}

	// 		else
	// 			torque_close_loop(joint_id, TAU[joint_id]);
	// 		// printf("TAU%d=%.3f\r\n", joint_id, TAU[joint_id]);
	// 		break;
	// }
	if(joint_id == 4)
		TAU[joint_id] = -TAU[joint_id];

	// torque_close_loop(joint_id, TAU[joint_id]);
	// for(int i=1; i<=6; i++)
	// {
	// 	printf("q%d=%.3f\r\n", i, q[i]);
	// }
	// printf("angle%d=%.3f\r\n", joint_id, motorAngle_float[joint_id]);
	printf("q%d=%.3f\t", joint_id, q[joint_id]);
	printf("qd%d=%.3f\t", joint_id, motor_speed_float[joint_id]);
	printf("current%d=%.3f\t", joint_id, motor_current_float[joint_id]);
	printf("TAU%d=%.3f\r\n", joint_id, TAU[joint_id]);
}


void compensation_singleJoint_GF(int joint_id)
{
	//读取各关节角度、角速度
	for(int cpt_id=1; cpt_id<=6; cpt_id++)
	{
		if(cpt_id == 1)
		{
			LinearActuator_read_position(cpt_id);//单位：mm
			LinearActuator_read_CurrentandSpeed(cpt_id);
			// pressure_SensorReading();
		}
		else
		{
			read_status2(cpt_id);//单位：du
			read_angle(cpt_id);//单位：du
		}
		
		switch (cpt_id)
		{
		case 1:
			/* 直线电机；DH初始位置joint1：95.35mm */
			q[cpt_id] = motorAngle_float[cpt_id] + 95.35;//单位：rad
			qd[cpt_id] = motor_speed_float[cpt_id];//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		case 4:
			/* DH初始位置joint4：-pi/2 */
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f - pi/2;//单位：rad
			qd[cpt_id] = motor_speed_float[cpt_id] * pi /180.0f;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		case 5:
			/* DH初始位置joint5： pi/2 ；有8.2du的偏差*/
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f + (90-8.2)/180*pi;//单位：rad
			qd[cpt_id] = motor_speed_float[cpt_id] * pi /180.0f;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		default:
			q[cpt_id] = motorAngle_float[cpt_id] * pi /180.0f;//单位：rad
			qd[cpt_id] = motor_speed_float[cpt_id] * pi /180.0f;//单位：rad/s
			qdd[cpt_id] = 0;
			break;
		}
	}
	
	compute_TAU(q, qd, qdd, joint_id);

	// switch(joint_id)
	// {
	// 	case 5: 
	// 		// if(q[joint_id] < -1*pi/9-8.2/180*pi || q[joint_id] > 11*pi/18-8.2/180*pi)
	// 		if(q[joint_id] < (-20-8.2)/180*pi || q[joint_id] > (210-8.2)/180*pi)
	// 		{
	// 			torque_close_loop(joint_id, -TAU[joint_id]);
	// 		}

	// 		else
	// 			torque_close_loop(joint_id, TAU[joint_id]);
	// 		// printf("TAU%d=%.3f\r\n", joint_id, TAU[joint_id]);
	// 		break;
	// }
	// torque_close_loop(joint_id, TAU[joint_id]);
	// for(int i=1; i<=6; i++)
	// {
	// 	printf("q%d=%.3f\r\n", i, q[i]);
	// }
	// printf("angle%d=%.3f\r\n", joint_id, motorAngle_float[joint_id]);
	printf("q%d=%.3f\t", joint_id, q[joint_id]);
	printf("TAU%d=%.3f\r\n", joint_id, TAU[joint_id]);
}

void fourier_series_traj(float time)
{
	uint8_t order_prod_2, m;
  	order_prod_2 = traj_order * 2;

	traj_wf = traj_f * 2.0f * pi;
	
	for(int i=1; i<=dof; i++)
	{
		m = (order_prod_2 + 1) * (i - 1); 
		q[i] = traj_param[m + order_prod_2 + 1];//q0
		for(int j=1; j<=traj_order; j++)
		{
			// alpha(a)=traj_param(m+2*(j-1)+1), beta(b)=traj_param(m+2*(j-1)+2)
			q[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * time) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * time));//需要matlab检验sin中是否为弧度值
//			if(motor_control_k<200)
//			{
//				q_next[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * (time+control_interval_time)) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * (time+control_interval_time)));
//			}
//			else
//				q_next[i] = q[i];
		}
	}
}

void traj_exciting_init(void)
{
	traj_n = 1.0f / control_interval_time / traj_f;
}

void run_fourier_series_traj(void)
{
	float motor_speed_ctr = 0;

	fourier_series_traj(motor_control_k*control_interval_time);//电机控制信号点
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
		{
			q[i] = q[i]-95.35f;//直线电缸单位mm
//			q_next[i] = q_next[i]*1000;
			motor_speed_ctr = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			LinearActuator_startRun_maxspeed_position(i, q[i], motor_speed_ctr);
			q_last[i] = q[i];
//			printf("linear=%.3f, linearSpeed=%.3f\r\n",q[1], motor_speed);
		}
		else if(i == 4)
		{
			q[i] = q[i]/pi*180 + 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed_ctr = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed_ctr<1)//速度不能太小
				motor_speed_ctr=1;
			angle_close_loop_with_speed(i, -q[i], motor_speed_ctr);
			q_last[i] = q[i];
//			printf("MOTORSPEED4=%.3f, MOTORpos4=%.3f\r\n", motor_speed, q[4]);
		}
		if(i == 5)
		{
			q[i] = q[i]/pi*180 - 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed_ctr = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed_ctr<1)
				motor_speed_ctr=1;
			angle_close_loop_with_speed(i, q[i], motor_speed_ctr);
			q_last[i] = q[i];
		}
		else if(i == 6)
		{
			if((fabs(q[i]-q_last[i]))>=0.05)
			{
				q[i] = q[i]/pi*180;
//				q_next[i] = q_next[i]/pi*180;
				motor_speed_ctr = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
				if(motor_speed_ctr<1)
				  motor_speed_ctr=1;
				angle_close_loop_with_speed(i, q[i], motor_speed_ctr);
				q_last[i] = q[i];
			}
		}
		else
		{
			q[i] = q[i]/pi*180;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed_ctr = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed_ctr<1)
				motor_speed_ctr=1;
			angle_close_loop_with_speed(i, q[i], motor_speed_ctr);
			q_last[i] = q[i];
//			if(i==6)
//				printf("MOTORSPEED6=%.3f, MOTORpos=%.3f\r\n", motor_speed, q[6]);
		}
		driv_ans_singlejoint(i);
	}
	
	printf("LA_pos:%.3fmm, LA_c:%.3fA, LA_s:%.3frpm, LA_Prs:%.3fV\t\r\n  \
		m_c2:%.3fA , m_s2:%.3fdps, m_ang2:%.3fdu, m_acc2:%.3fdps/s\t\r\n   \
		m_c3:%.3fA , m_s3:%.3fdps, m_ang3:%.3fdu, m_acc3:%.3fdps/s\t\r\n   \
		m_c4:%.3fA , m_s4:%.3fdps, m_ang4:%.3fdu, m_acc4:%.3fdps/s\t\r\n   \
		m_c5:%.3fA , m_s5:%.3fdps, m_ang5:%.3fdu, m_acc5:%.3fdps/s\t\r\n   \
		m_c6:%.3fA , m_s6:%.3fdps, m_ang6:%.3fdu, m_acc6:%.3fdps/s\t\r\n"  \
		, LinAcr_position_float, LinAcr_current_float, LinAcr_speed_float, ADC_Pressure_Value
		, motor_current_float[2], motor_speed_float[2], motorAngle_float[2], motorAccel_float[2]
		, motor_current_float[3], motor_speed_float[3], motorAngle_float[3], motorAccel_float[3]
		, motor_current_float[4], motor_speed_float[4], motorAngle_float[4], motorAccel_float[4]
		, motor_current_float[5], motor_speed_float[5], motorAngle_float[5], motorAccel_float[5]
		, motor_current_float[6], motor_speed_float[6], motorAngle_float[6], motorAccel_float[6]);

//	printf("k=%d", motor_control_k);
	if(++motor_control_k > 400)
		HAL_TIM_Base_Stop_IT(&htim2);
}




