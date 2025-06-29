#include "Balancer2.h"


volatile static double Tspd_L_i = 0.0,Tspd_R_i = 0.0;	//移動速度積分用変数
extern int first_call;


/*
 * 制御用編雨数の初期化
 */
void ClearVariables(){
	ClearGyroVariables();
	ClearEncVariables();
	ClearCurrentVariables();
	Tspd_L_i = 0.0;
	Tspd_R_i = 0.0;
}

/*
 * キャリブレーション
 */
unsigned char calibrarion(){
	static int64_t currenttemp[2] = {0L,0L};
	static int64_t gyrosum = 0L;

	//変数初期化
	if(memmap.values.CALIBRATISON_CNT == 0){
		memmap.values.BODY_ANGULAR_SPD_OFFSET = 0;
		memmap.values.CURRENT_OFFSET_L = 0;
		memmap.values.CURRENT_OFFSET_R = 0;

		gyrosum = 0L;
		currenttemp[0] = 0L;
		currenttemp[1] = 0L;
	}

	//加算
	{
		gyrosum += (int64_t)memmap.values.GYRO_DATA;
		currenttemp[0] += (int64_t)memmap.values.M_CURRENT_L;
		currenttemp[1] += (int64_t)memmap.values.M_CURRENT_R;
	}
	memmap.values.CALIBRATISON_CNT++;

	//終了処理
	if((int)memmap.values.CALIBRATISON_CNT >= (int)(memmap.values.CALIBRATISON_TIME * MAIN_CYCLE)){
		uint32_t num = memmap.values.CALIBRATISON_TIME * MAIN_CYCLE;
		memmap.values.BODY_ANGULAR_SPD_OFFSET = (double)gyrosum/num;
		memmap.values.CURRENT_OFFSET_L = (short)(currenttemp[0]/num);
		memmap.values.CURRENT_OFFSET_R = (short)(currenttemp[1]/num);

		gyrosum = 0L;
		currenttemp[0] = 0L;
		currenttemp[1] = 0L;
		memmap.values.CALIBRATISON_CNT = 0;
		return 1;
	}
	return 0;
}

double Control_PID(const double x[])
{
	double u=0.;
	return u;
}

double Control_Feedback(const double x[])
{
	// Pythonで最適制御のゲインを計算し代入する
	double K[4] = {0.0, 0.0, 0.0, 0.0};

	// pole place
	K[0] = 0.29814475;
	K[1] = 12.70135219;
	K[2] = 0.11637412;
	K[3] = 2.01665043;


	// Q=(10,10,1,1) R=10
	//K[0] = 1.;
	//K[1] = 76.3478;
	//K[2] = 0.65244;
	//K[3] = 12.6179;
	// Q=(10,1000,1,1) R=10
	//K[0] = 1.;
	//K[1] = 77.2384;
	//K[2] = 0.65538;
	//K[3] = 12.665833;
	// Q=(10,100,1,1) R=10
	//K[0] = 1.;
	//K[1] = 76.4294;
	//K[2] = 0.652712;
	//K[3] = 12.621626;
	// Q=(10,100,1,1) R=100
	//K[0] = 0.316;
	//K[1] = 25.4954;
	//K[2] = 0.20871034;
	//K[3] = 4.1605286;

	// θ-ψ + cd^2
	//K[0] =  1.;
	//K[1] = 77.40876787;
	//K[2] = 0.65243195;
	//K[3] = 13.27076764;

	// 最適制御による状態フィードバック制御
	double u = 0.0, uu[4];
	int i = 0;
	for(i=0; i<4; i++){
		uu[i] = -K[i]*x[i];
		u += uu[i];
	}
	// USB経由で変数参照できるようにセット
	memmap.values.USER_ARIA1 = uu[0];
	memmap.values.USER_ARIA2 = uu[1];
	memmap.values.USER_ARIA3 = uu[2];
	memmap.values.USER_ARIA4 = uu[3];

	memmap.values.USER_ARIA5 = MAIN_CYCLE;

	return u;
}

double Control_Adaptive(const double x[])
{
	/*---------------------------------------------------------------
	 *  Adaptive MRAC controller for an inverted pendulum (C)
	 *   - called every sampling period Ts by Control()
	 *   - state  x[4]    : measured  [θ1, θ̇1, θ2, θ̇2]
	 *   - gain   K[4]    : fixed LQR row vector
	 *   - model  A[4][4] : nominal closed-loop matrix A_c = A0-BK
	 *             B[4]   : input vector (nominal)
	 *
	 *   u = −Kx + θ̂ᵀ x
	 *   θ̂̇ = −γ (Bᵀ P e) x             (e = x − x_m)
	 *   ẋ_m = A_c x_m                 (continuous Euler update)
	 *--------------------------------------------------------------*/

	double u=0;
	int N=4;                /* state dimension */
	//double gamma = 300.0;   /* adaptation gain */
	double TS=0.002;        /* sampling period [s] 500Hzらしい(MAIN_CYCLEマクロ)*/

	/* -------- Auto‑generated C declarations -------- */
	const double Ac[4][4] = {
	    { 0.0000000000e+00, 0.0000000000e+00, 1.0000000000e+00, 0.0000000000e+00 },
	    { 0.0000000000e+00, 0.0000000000e+00, 0.0000000000e+00, 1.0000000000e+00 },
	    { 9.7325766501e+02, 7.5458843233e+04, 6.3681485010e+02, 1.3044311454e+04 },
	    { -7.5998295233e+01, -5.8561185200e+03, -4.9726649658e+01, -1.0185847681e+03 }
	};

	const double B[4] = { 0.0000000000e+00, 0.0000000000e+00, -9.7325766501e+02, 7.5998295233e+01 };

	const double K[4] = { 1.0000000000e+00, 7.7978431519e+01, 6.5431270913e-01, 1.3402731787e+01 };

	const double P[4][4] = {
	    { 2.0150485660e+00, 4.5356567803e+01, 6.0440922331e-01, 7.7468304727e+00 },
	    { 4.5356567803e+01, 3.0975234181e+03, 4.0168126721e+01, 5.1758530277e+02 },
	    { 6.0440922331e-01, 4.0168126721e+01, 5.2381364134e-01, 6.7303290492e+00 },
	    { 7.7468304727e+00, 5.1758530277e+02, 6.7303290492e+00, 8.6699307089e+01 }
	};

	//double gamma = 5.00e-02;
	double gamma = memmap.values.GAIN_OPTION1; // 実行中に外から変更可能

	/* --- static memories ------------------------------------- */
	static double xm[4]        = {0};   /* reference model state */
	static double theta_hat[4] = {0};   /* adaptive gain         */

	/* 1. initialize reference model at t = 0 ------------------ */
	if (first_call) {
		for (int i = 0; i < N; ++i) {
			xm[i] = x[i];
			theta_hat[i] = 0.0;
		}
		first_call = 0;
	}

	/* 2. error vector e = x − xm ------------------------------ */
	double e[4];
	for (int i = 0; i < N; ++i) e[i] = x[i] - xm[i];

	/* 3. compute s = Bᵀ P e   (scalar) ------------------------ */
	double s = 0.0;
	for (int i = 0; i < N; ++i) {
		double tmp = 0.0;
		for (int k = 0; k < N; ++k) tmp += P[i][k] * e[k];
		s += B[i] * tmp;
	}

	/* 4. update adaptive parameters  θ̂ ← θ̂ + θ̂̇·Ts ----------- */
	for (int i = 0; i < N; ++i)
		theta_hat[i] += (-gamma * s * x[i]) * TS;

	/* 5. control input  u = −Kx + θ̂ᵀ x ----------------------- */
	double u_k = 0.0, u_t = 0.0;
	for (int i = 0; i < N; ++i) {
		u_k += -K[i] * x[i];
		u_t += theta_hat[i] * x[i];
	}
	u = u_k + u_t;

	/* 6. propagate reference model  ẋ_m = A_c x_m  (Euler) ---- */
	double dxm[4] = {0};
	for (int i = 0; i < N; ++i)
		for (int k = 0; k < N; ++k)
			dxm[i] += Ac[i][k] * xm[k];

	for (int i = 0; i < N; ++i)
		xm[i] += dxm[i] * TS;

	// USB経由で変数参照できるようにセット
	memmap.values.USER_ARIA1 = u;
	memmap.values.USER_ARIA2 = u_k;
	memmap.values.USER_ARIA3 = u_t;

	return u;
}

/*
 * 倒立制御
 */
void Control(){
	volatile double outL = 0.0, outR = 0.0;
	// 状態量
	double x[4];
	x[0] = -(memmap.values.WHEEL_ANGLE_L + memmap.values.WHEEL_ANGLE_R)/2.0;
	x[1] = memmap.values.BODY_ANGLE;
	x[2] = -(memmap.values.WHEEL_ANGULAR_SPD_L + memmap.values.WHEEL_ANGULAR_SPD_R)/2.0;
	x[3] = memmap.values.BODY_ANGULAR_SPD;

	double u;

	// u = Control_PID(x);
	//u = Control_Feedback(x);
	u = Control_Adaptive(x);

	outL += u;
	outR += u;

	//最大指令値規制
	if(outL > 32767.0){
		outL = 32767.0;
	}
	if(outR > 32767.0){
		outR = 32767.0;
	}
	if(outL < -32767.0){
		outL = -32767.0;
	}
	if(outR < -32767.0){
		outR = -32767.0;
	}

	//電流指令値設定
	//memmap.values.T_CURRENT_L = (short)outL*1000;
	//memmap.values.T_CURRENT_R = (short)outR*1000;
	double gain = memmap.values.USER_ARIA8;
	memmap.values.T_CURRENT_L = (short)(outL*gain);
	memmap.values.T_CURRENT_R = (short)(outR*gain);

}

//VS-C3操縦の最大速度
#define VS_C3_MOVE_MAX_SPEED 9.42477796076937972 //rad/sec = 540deg/sec

//VS-C3操縦の速度設定
void VS_C3Control(){
	double out_L = 0.0,out_R = 0.0;

	out_L = -(double)(signed char)(memmap.values.PAD_AN_LY) * VS_C3_MOVE_MAX_SPEED/128.0;
	out_R = -(double)(signed char)(memmap.values.PAD_AN_LY) * VS_C3_MOVE_MAX_SPEED/128.0;
	out_L += (double)(signed char)(memmap.values.PAD_AN_RX) * VS_C3_MOVE_MAX_SPEED/128.0;
	out_R -= (double)(signed char)(memmap.values.PAD_AN_RX) * VS_C3_MOVE_MAX_SPEED/128.0;

	memmap.values.T_SPD_L = out_L;
	memmap.values.T_SPD_R = out_R;
}

//最大速度規制値
#define WHEELANGURARPSD_MAX (20.0*m_PI)	//3600deg/sec

//最大速度超過していないか確認（転倒検出）
unsigned char isWhleeOverSpeed(){
	unsigned char overspd = 0;
	static double old_L = 0.0,old_R = 0.0;
	if((memmap.values.WHEEL_ANGLE_L - old_L) > WHEELANGURARPSD_MAX/MODE_CYCLE){
		overspd++;
	}
	else if((memmap.values.WHEEL_ANGLE_L - old_L) < -WHEELANGURARPSD_MAX/MODE_CYCLE){
		overspd++;
	}
	if((memmap.values.WHEEL_ANGLE_R - old_R) > WHEELANGURARPSD_MAX/MODE_CYCLE){
		overspd++;
	}
	else if((memmap.values.WHEEL_ANGLE_R - old_R) < -WHEELANGURARPSD_MAX/MODE_CYCLE){
		overspd++;
	}
	if(overspd != 0){
		old_L = 0.0;
		old_R = 0.0;
		return 1;
	}
	old_L = memmap.values.WHEEL_ANGLE_L;
	old_R = memmap.values.WHEEL_ANGLE_R;
	return 0;
}

