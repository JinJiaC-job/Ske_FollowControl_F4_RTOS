#ifndef FOURIER_TRAJ_H
#define FOURIER_TRAJ_H

#include "main.h"

// PARAMETERS
#define pi 3.141592654f
//�������ʱ���
extern unsigned int motor_control_k;
//motor control interval time
extern float control_interval_time;
// sampling period
extern float traj_Ts;
// trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
extern float traj_f;
// trajectory fundamental frequency in radian
extern float traj_wf;
// number of sampling points
extern uint8_t traj_n;
// order of trajectory generation 
extern uint8_t traj_orde;
// number of revolute joints
extern uint8_t dof;
// 6���ؽڽǶ���Ϣ
extern float q[7];
extern float qd[7];
extern float qdd[7];
extern float q_last[7];
extern float q_next[7];
//extern float traj_param[66];
extern float TAU[7];

void fourier_series_traj(float time);
void traj_exciting_init(void);
void run_fourier_series_traj(void);
float compute_TAU(float *q, float *qd, float *qdd, int id);
void compensation_ske_GF(void);
void compensation_singleJoint_GF(int joint_id);

//math function
int sign(float value);


#endif

