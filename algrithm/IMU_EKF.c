#include "IMU_EKF.h"
#include "sys.h"
#include "usart.h"


//X =F*X =X+dX*dt
arm_matrix_instance_f32 X_matrix;
arm_matrix_instance_f32 F_matrix;
//P=F*P*FT+Q
arm_matrix_instance_f32 FT_matrix;
arm_matrix_instance_f32 P_matrix;
arm_matrix_instance_f32 FP_matrix;
arm_matrix_instance_f32 Q_matrix;
//K=P*HT/(H*P*HT+R)
//PHT=P*HT
//HZ=H*P*HT+R
//HZI=1/(H*P*HT+R)
arm_matrix_instance_f32 H_matrix;
arm_matrix_instance_f32 HT_matrix;
arm_matrix_instance_f32 PHT_matrix;
arm_matrix_instance_f32 HZ_matrix;
arm_matrix_instance_f32 HZI_matrix;
arm_matrix_instance_f32 K_matrix;
arm_matrix_instance_f32 R_matrix;
//X=X+K*(Z-H*X)
//Y=K*Z
arm_matrix_instance_f32 Y_matrix;
arm_matrix_instance_f32 Z_matrix;
//P=P-KHP
arm_matrix_instance_f32 KH_matrix;
arm_matrix_instance_f32 KHP_matrix;



float X[13];
float F[169];

float FT[169];
float P[169];
float FP[169];
float Q[169];

float H[13*6];
float HT[13*6];
float PHT[13*6];
float HZ[6*6];
float HZI[6*6];
float K[13*6];
float R[36];

float Y[6];
float Z[6];

float KH[13*13];
float KHP[13*13];

float Ix; //magnetic inclination
float Iz; //

bool Init_ekf(float MagInclination)
{
	arm_mat_init_f32(&X_matrix,13,1,X);
	arm_mat_zero_f32(&X_matrix);
	X[1]=1;
	
	arm_mat_init_f32(&F_matrix,13,13,F);
	arm_mat_identity_f32(&F_matrix,1.0f);
	arm_mat_init_f32(&FT_matrix,13,13,FT);
	arm_mat_zero_f32(&FT_matrix);
	
	arm_mat_init_f32(&P_matrix,13,13,P);
	arm_mat_identity_f32(&P_matrix,1.0f);
	
	arm_mat_init_f32(&FP_matrix,13,13,FP);
	arm_mat_identity_f32(&FP_matrix,0.5f);
	
	arm_mat_init_f32(&Q_matrix,13,13,Q);
	arm_mat_identity_f32(&Q_matrix,0.001f);
	Q[0]=0.001;Q[14]=0.001;Q[28]=0.001;Q[42]=0.001;
	Q[56]=0.0001;Q[70]=0.0001;
	//
//	Q[0]=0.1;
	
	arm_mat_init_f32(&R_matrix,6,6,R);
	arm_mat_identity_f32(&R_matrix,0.01f);
	R[0]=0.001; R[7]=0.001; R[14]=0.001;
	R[21]=0.005;R[28]=0.005;R[35]=0.005;
	//
	
	arm_mat_init_f32(&H_matrix,6,13,H);
	arm_mat_zero_f32(&H_matrix);	
	arm_mat_init_f32(&HT_matrix,13,6,HT);
	arm_mat_zero_f32(&HT_matrix);
	
	arm_mat_init_f32(&PHT_matrix,13,6,PHT);
	arm_mat_zero_f32(&PHT_matrix);
	arm_mat_init_f32(&HZ_matrix,6,6,HZ);
	arm_mat_zero_f32(&HZ_matrix);
	arm_mat_init_f32(&HZI_matrix,6,6,HZI);
	arm_mat_zero_f32(&HZI_matrix);
	arm_mat_init_f32(&K_matrix,13,6,K);
	arm_mat_zero_f32(&K_matrix);
	
	arm_mat_init_f32(&Z_matrix,6,1,Z);
	arm_mat_zero_f32(&Z_matrix);
	arm_mat_init_f32(&Y_matrix,6,1,Y);
	arm_mat_zero_f32(&Y_matrix);
	
	arm_mat_init_f32(&KH_matrix,13,13,KH);
	arm_mat_zero_f32(&KH_matrix);
	arm_mat_init_f32(&KHP_matrix,13,13,KHP);
	arm_mat_zero_f32(&KHP_matrix);
	
	Ix=FastSin(MagInclination);
	Iz=FastCos(MagInclination);
	
	
	return true;
}


bool EKFupdata(/*float *q,*/float* acc,float* gyro,float* mag,float* v,float dt)
{
	float _2q0q1,_2q0q2,_2q0q3;
	float _2q1q1,_2q1q2,_2q1q3;
	float _2q2q3,_2q2q2,_2q3q3;
	float _2q0,_2q1,_2q2,_2q3;
	float half_q0dt,half_q1dt,half_q2dt,half_q3dt;
	
	float half_Wdt[3];
	float Acc_dt[3];
	float norm;
	//test 
//	int i;
	_2q0=X[0]*2.0f;
	_2q1=X[1]*2.0f;
	_2q2=X[2]*2.0f;
	_2q3=X[3]*2.0f;
	
	_2q0q1=_2q0*X[1];//_2q0q1=X[0]*X[1]*2.0f;
	_2q0q2=_2q0*X[2];//_2q0q2=X[0]*X[2]*2.0f;
	_2q0q3=_2q0*X[3];//_2q0q3=X[0]*X[3]*2.0f;
	
	_2q1q1=_2q1*X[1];//_2q1q1=X[1]*X[1]*2.0f;
	_2q1q2=_2q1*X[2];//_2q1q2=X[1]*X[2]*2.0f;
	_2q1q3=_2q1*X[3];//_2q1q3=X[1]*X[3]*2.0f;
	
	_2q2q2=_2q2*X[2];//_2q2q2=X[2]*X[2]*2.0f;
	_2q2q3=_2q2*X[3];//_2q2q3=X[2]*X[3]*2.0f;	
	_2q3q3=_2q3*X[3];//_2q3q3=X[3]*X[3]*2.0f;
	
	
	half_Wdt[0]=(gyro[0]-X[7])*dt*0.5f;
	half_Wdt[1]=(gyro[1]-X[8])*dt*0.5f;
	half_Wdt[2]=(gyro[2]-X[9])*dt*0.5f;
	
	half_q0dt=X[0]*dt*0.5f;
	half_q1dt=X[1]*dt*0.5f;
	half_q2dt=X[2]*dt*0.5f;
	half_q3dt=X[3]*dt*0.5f;
	
	
	Acc_dt[0]=(acc[0]-X[4])*dt;
	Acc_dt[1]=(acc[1]-X[5])*dt;
	Acc_dt[2]=(acc[2]-X[6])*dt;
	//X=X + dX*dt
	X[0] =X[0] -half_Wdt[0]*X[1] -half_Wdt[1]*X[2] -half_Wdt[2]*X[3];
	X[1] =half_Wdt[0]*X[0] +X[1] +half_Wdt[2]*X[2] -half_Wdt[1]*X[3];
	X[2] =half_Wdt[1]*X[0] -half_Wdt[2]*X[1] +X[2] +half_Wdt[0]*X[3];
	X[3] =half_Wdt[2]*X[0] +half_Wdt[1]*X[1] -half_Wdt[0]*X[2] +X[3];
	
	
	X[10]=X[10] +Acc_dt[0];
	X[11]=X[11] +Acc_dt[1];
	X[12]=X[12] +Acc_dt[2];
	//F
	/*F[0] =1;*/F[1] =-half_Wdt[0];F[2] =-half_Wdt[1];F[3] =-half_Wdt[2];
//	F[4] =0;F[5] =0;F[6] =0;
	F[7] =half_q1dt;F[8] =half_q2dt;F[9] =half_q3dt;
//	F[10]=0;F[11]=0;F[12]=0;
	
	F[13]=half_Wdt[0];/*F[14]=1;*/F[15]=half_Wdt[2];F[16]=-half_Wdt[1];
//	F[17]=0;F[18]=0;F[19]=0;
	F[20]=-half_q0dt;F[21]=half_q3dt;F[22]=-half_q2dt;
//	F[23]=0;F[24]=0;F[25]=0;
	
	F[26]=half_Wdt[1];F[27]=-half_Wdt[2];/*F[28]=1;*/F[29]=half_Wdt[0];
//	F[30]=0;F[31]=0;F[32]=0;
	F[33]=-half_q3dt;F[34]=-half_q0dt;F[35]=half_q1dt;
//	F[36]=0;F[37]=0;F[38]=0;
	
	F[39]=half_Wdt[2];F[40]=half_Wdt[1];F[41]=-half_Wdt[0];/*F[42]=1;*/
//	F[43]=0;F[44]=0;F[45]=0;
	F[46]=half_q2dt;F[47]=-half_q1dt;F[48]=-half_q0dt;
//	F[49]=0;F[50]=0;F[51]=0;
	
//	F[130]=0;F[131]=0;F[132]=0;F[133]=0;
	F[134]=-dt;/*F[135]=0;F[136]=0;*/
//	F[137]=0;F[138]=0;F[139]=0;
//	F[140]=1;/*F[141]=0;F[142]=0;*/
	
//	F[143]=0;F[144]=0;F[145]=0;F[146]=0;
	/*F[147]=0;*/F[148]=-dt;/*F[149]=0;*/
//	F[150]=0;F[151]=0;F[152]=0;
//	F[153]=0;F[154]=1;F[155]=0;
	
//	F[156]=0;F[157]=0;F[158]=0;F[159]=0;
	/*F[160]=0;F[161]=0;*/F[162]=-dt;
//	F[163]=0;F[164]=0;F[165]=0;
//	F[166]=0;F[167]=0;F[168]=1;
	//P=F*P*F' +Q
	arm_mat_trans_f32(&F_matrix,&FT_matrix);
	arm_mat_mult_f32(&F_matrix,&P_matrix,&FP_matrix);
	arm_mat_mult_f32(&FP_matrix,&FT_matrix,&P_matrix);
	arm_mat_add_f32(&P_matrix,&Q_matrix,&P_matrix);
	
	//
	Z[0] =_2q1q3-_2q0q1;
	Z[1] =_2q2q3+_2q0q1;
	Z[2] =1-_2q1q1-_2q2q2;
	//?
	Z[3] =(1-_2q2q2-_2q3q3)*Ix+(_2q1q3-_2q0q2)*Iz;
	Z[4] =(_2q1q2-_2q0q3)*Ix+(_2q2q3+_2q0q1)*Iz;
	Z[5] =(_2q1q3+_2q0q2)*Ix+(1-_2q1q1-_2q2q2)*Iz;
//printf("z= ");
//for(i=0;i<6;i++)
//{
//	printf("%f :: ",Z[i]);
//}
//printf("\r\n");
	
	H[0]=-_2q2;	H[1]=_2q3;	H[2]=-_2q0;	H[3]=_2q1;
//	H[4]=0;		H[5]=0;		H[6]=0;
//	H[7]=0;		H[8]=0;		H[9]=0;
//	H[10]=0;	H[11]=0;	H[12]=0;
	
	H[13]=_2q1;	H[14]=_2q0; H[15]=_2q3; H[16]=_2q2;
//	H[17]=0;	H[18]=0;	H[19]=0;
//	H[20]=0;	H[21]=0;	H[22]=0;
//	H[23]=0;	H[24]=0;	H[25]=0;
	
	H[26]=0;	H[27]=_2q1*(-2.0f); H[28]=_2q2*(-2.0f); H[29]=0;
//	H[30]=0;	H[31]=0;	H[32]=0;	
//	H[33]=0;	H[34]=0;	H[35]=0;	
//	H[36]=0;	H[37]=0;	H[38]=0;
	
	H[39]=-_2q2*Iz;					H[40]=_2q3*Iz;
	H[41]=_2q2*Ix*(-2.0f)-_2q0*Iz;	H[42]=_2q3*Ix*(-2.0f)+_2q1*Iz;
//	H[43]=0;	H[44]=0;	H[45]=0;
//	H[46]=0;	H[47]=0;	H[48]=0;
//	H[49]=0;	H[50]=0;	H[51]=0;	
	H[52]=_2q1*Iz-_2q3*Ix;			H[53]=_2q2*Ix+_2q0*Iz;
	H[54]=_2q1*Ix+/*_2q3*Iz*/H[40];	H[55]=_2q2*Iz-_2q0*Ix;
//	H[56]=0;	H[57]=0;	H[58]=0;
//	H[59]=0;	H[60]=0;	H[61]=0;
//	H[62]=0;	H[63]=0;	H[64]=0;
	H[65]=_2q2*Ix;	H[66]=_2q3*Ix-_2q1*Iz*(2.0f);
	H[67]=_2q0*Ix-_2q2*Iz*(2.0f);	H[68]=_2q1*Ix;
//	H[69]=0;	H[70]=0;	H[71]=0;
//	H[72]=0;	H[73]=0;	H[74]=0;
//	H[75]=0;	H[76]=0; 	H[77]=0;
	
	//k=(P*H')/(H*P*H' +R)
	//HZ=H*P*H' +R
	arm_mat_trans_f32(&H_matrix,&HT_matrix);
	arm_mat_mult_f32(&P_matrix,&HT_matrix,&PHT_matrix);
	arm_mat_mult_f32(&H_matrix,&PHT_matrix,&HZ_matrix);
	arm_mat_add_f32(&HZ_matrix,&R_matrix,&HZ_matrix);
	arm_mat_inverse_f32(&HZ_matrix,&HZI_matrix);
	arm_mat_mult_f32(&PHT_matrix,&HZI_matrix,&K_matrix);
	
	//X=X+k(Z-Y)
	norm =FastSqrtI(mag[0]*mag[0]+mag[1]*mag[1]+mag[2]*mag[2]+mag[3]*mag[3]);
	mag[0]*=norm;
	mag[1]*=norm;
	mag[2]*=norm;
	mag[3]*=norm;
	norm =FastSqrtI(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]+acc[3]*acc[3]);
	acc[0]*=norm;
	acc[1]*=norm;
	acc[2]*=norm;
	acc[3]*=norm;
	
	Z[0]=acc[0]-Z[0];
	Z[1]=acc[1]-Z[1];
	Z[2]=acc[2]-Z[2];
	Z[3]=mag[0]-Z[3];
	Z[4]=mag[1]-Z[4];
	Z[5]=mag[2]-Z[5];
	
	arm_mat_mult_f32(&K_matrix,&Z_matrix,&Y_matrix);
	arm_mat_add_f32(&X_matrix,&Y_matrix,&X_matrix);
	
	//P=(I-KH)P=P-KHP
	//
	arm_mat_mult_f32(&K_matrix,&H_matrix,&KH_matrix);
	arm_mat_mult_f32(&KH_matrix,&P_matrix,&KHP_matrix);
	arm_mat_sub_f32(&P_matrix,&KHP_matrix,&P_matrix);
	
	//
	norm=FastSqrtI(X[0]*X[0]+X[1]*X[1]+X[2]*X[2]+X[3]*X[3]);
	X[0]*=norm;
	X[1]*=norm;
	X[2]*=norm;
	X[3]*=norm;
//printf("q= %f : %f : %f : %f \r\n",X[0],X[1],X[2],X[3]);
	
//	q[0]=X[0];
//	q[1]=X[1];
//	q[2]=X[2];
//	q[3]=X[3];
	return true;
}

#define TWOPI 	6.283185307179f
//#define QUAT_TODEG(x) ((x) * 57.2957796f)
void QUAT_GetAngle(float* rpy)
{
	float R[3][3];
	//Z-Y-X
	R[0][0] = 2.0f * (X[0] * X[0] + X[1] * X[1]) - 1.0f;
	R[0][1] = 2.0f * (X[1] * X[2] + X[0] * X[3]);
	R[0][2] = 2.0f * (X[1] * X[3] - X[0] * X[2]);
	//R[1][0] = 2.0f * (X[1] * X[2] - X[0] * X[3]);
	//R[1][1] = 2.0f * (X[0] * X[0] + X[2] * X[2]) - 1.0f;
	R[1][2] = 2.0f * (X[2] * X[3] + X[0] * X[1]);
	//R[2][0] = 2.0f * (X[1] * X[3] + X[0] * X[2]);
	//R[2][1] = 2.0f * (X[2] * X[3] - X[0] * X[1]);
	R[2][2] = 2.0f * (X[0] * X[0] + X[3] * X[3]) - 1.0f;

	//roll
	rpy[0] = FastAtan2(R[1][2], R[2][2]);
	if (rpy[0] == PI)
		rpy[0] = -PI;
	//pitch
	if (R[0][2] >= 1.0f)
		rpy[1] = -PI_2;
	else if (R[0][2] <= -1.0f)
		rpy[1] = PI_2;
	else
		rpy[1] = FastAsin(-R[0][2]);
	//yaw
	rpy[2] = FastAtan2(R[0][1], R[0][0]);
	if (rpy[2] < 0.0f){
		rpy[2] += TWOPI;
	}
	if (rpy[2] > TWOPI){
		rpy[2] = 0.0f;
	}

	rpy[0] = RADTODEG(rpy[0]);
	rpy[1] = RADTODEG(rpy[1]);
	rpy[2] = RADTODEG(rpy[2]);
}
