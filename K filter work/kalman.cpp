#define i10mult(a,b) (a*b)>>10
#include "kalman.h"

void inverse3x3(int, int, int, int, int, int, int, int, int);

void KalmanFilter::assignSensorValues(int ax, int ay, int az,
			int gx, int gy, int gz,
			int Cx, int Cy, int Cz, // cz not used
			int Px, int Py, int Pz) {

	z_ax = ax * 1;
	z_ay = ay * 1;
	z_az = az * 1;
	z_gx = gx * 1;
	z_gy = gy * 1;
	z_gz = gz * 1;
	z_Px = Px * 1;
	z_Py = Py * 1;
	z_Pz = Pz * 1;

	z_Ox = (int)(atan2(z_ay, z_az) * 1024);
	z_Oy = (int)(atan2(z_ax, z_az) * 1024);
	z_Oz = (int)(atan2(Cx, Cy) * 1024);
}


// this function sets the new state variables from the previous ones and the
// updated sensor values
void KalmanFilter::predictAndUpdate() {
	nTimesRan++;
	unsigned int currentTime = 0; // TODO: get current time 
	unsigned int dT = currentTime - previousTimeRan;
	// expect time in ms

	// predict stage

	// Predicted (a priori) state estimate
	// ====================================================================

int xp_xp, xp_xv, xp_xa, xp_yp, xp_yv, xp_ya, xp_zp, xp_zv, xp_za;
	predictStateEstimateForPositions(
		xp_xp, xp_xv, xp_xa, x_xp, x_xv, x_xa, dT);
	predictStateEstimateForPositions(
		xp_yp, xp_yv, xp_ya, x_yp, x_yv, x_ya, dT);
	predictStateEstimateForPositions(
		xp_zp, xp_zv, xp_za, x_zp, x_zv, x_za, dT);

int xp_Xa, xp_Xr, xp_Xb, xp_Ya, xp_Yr, xp_Yb, xp_Za, xp_Zr, xp_Zb;
	predictStateEstimateForPositions(
		xp_Xa, xp_Xr, xp_Xb, x_Xa, x_Xr, x_Xb, dT);
	predictStateEstimateForPositions(
		xp_Ya, xp_Yr, xp_Yb, x_Ya, x_Yr, x_Yb, dT);
	predictStateEstimateForPositions(
		xp_Za, xp_Zr, xp_Zb, x_Za, x_Zr, x_Zb, dT);


//P_k = F_k * P_(k-1) * F_k'T + Q_k

int Pp_xpp, Pp_xpv, Pp_xpa, Pp_xvv, Pp_xva, Pp_xaa;
int Pp_ypp, Pp_ypv, Pp_ypa, Pp_yvv, Pp_yva, Pp_yaa;
int Pp_zpp, Pp_zpv, Pp_zpa, Pp_zvv, Pp_zva, Pp_zaa;
int Pp_Xaa, Pp_Xar, Pp_Xab, Pp_Xrr, Pp_Xrb, Pp_Xbb;
int Pp_Yaa, Pp_Yar, Pp_Yab, Pp_Yrr, Pp_Yrb, Pp_Ybb;
int Pp_Zaa, Pp_Zar, Pp_Zab, Pp_Zrr, Pp_Zrb, Pp_Zbb;

predictStateCovarianceForPosition(
		Pp_xpp, Pp_xpv, Pp_xpa, Pp_xvv, Pp_xva, Pp_xaa,
		P_xpp, P_xpv, P_xpa, P_xvv, P_xva, P_xaa,
		dT);
predictStateCovarianceForPosition(
		Pp_ypp, Pp_ypv, Pp_ypa, Pp_yvv, Pp_yva, Pp_yaa,
		P_ypp, P_ypv, P_ypa, P_yvv, P_yva, P_yaa,
		dT);
predictStateCovarianceForPosition(
		Pp_zpp, Pp_zpv, Pp_zpa, Pp_zvv, Pp_zva, Pp_zaa,
		P_zpp, P_zpv, P_zpa, P_zvv, P_zva, P_zaa,
		dT);
predictStateCovarianceForRotation(
		Pp_Xaa, Pp_Xar, Pp_Xab, Pp_Xrr, Pp_Xrb, Pp_Xbb,
		P_Xaa, P_Xar, P_Xab, P_Xrr, P_Xrb, P_Xbb,
		dT);
predictStateCovarianceForRotation(
		Pp_Yaa, Pp_Yar, Pp_Yab, Pp_Yrr, Pp_Yrb, Pp_Ybb,
		P_Yaa, P_Yar, P_Yab, P_Yrr, P_Yrb, P_Ybb,
		dT);
predictStateCovarianceForRotation(
		Pp_Zaa, Pp_Zar, Pp_Zab, Pp_Zrr, Pp_Zrb, Pp_Zbb,
		P_Zaa, P_Zar, P_Zab, P_Zrr, P_Zrb, P_Zbb,
		dT);

// degrade confidence by noise
Pp_xpp += Q_xpp; Pp_xvv += Q_xvv; Pp_xaa += Q_xaa;
Pp_ypp += Q_ypp; Pp_yvv += Q_yvv; Pp_yaa += Q_yaa;
Pp_zpp += Q_zpp; Pp_zvv += Q_zvv; Pp_zaa += Q_zaa;

Pp_Xaa += Q_Xaa; Pp_Xrr += Q_Xrr; Pp_Xbb += Q_Xbb;
Pp_Yaa += Q_Yaa; Pp_Yrr += Q_Yrr; Pp_Ybb += Q_Ybb;
Pp_Zaa += Q_Zaa; Pp_Zrr += Q_Zrr; Pp_Zbb += Q_Zbb;

// UPDATE =====================================================================
// Innovation or Measurement Residual

int y_ax = i10mult(H_ax_xa, xp_xa) + 
	i10mult(H_ax_xa, xp_xa) + 
	i10mult(H_ax_xa, xp_xa) + 

int y_ay = 
int y_az = 




}
// END OF FILTER =============================================================


void KalmanFilter::predictStateEstimateForPositions(
		int& xp_p, int& xp_v, int& xp_a,
		int& x_p, int& x_v, int& x_a,
		int& dT) {
	// state transition matrix is:
	// 	xp	xv	xa
	// xp	1	t	t^2/2
	// xv	0	1	t
	// xa	0	0	1

	xp_p = x_p + (x_v * dT >> 10) + ((x_a * dT >> 10) * dT >> 11 /* /2 */);
	xp_v = x_v + (x_a * dT >> 10);
	xp_a = x_a;
}

void KalmanFilter::predictStateEstimateForRotations(
		int& xp_a, int& xp_r, int& xp_b,
		int& x_a, int& x_r, int& x_b,
		int& dT) {
	// state transition matrix (rotations):
	// 	Xa	Xr	Xb
	// Xa	1	dT	-dT	
	// Xr	0	1	0
	// Xb	0	0	1

	xp_a = x_a + i10mult((x_r - x_b), dT);
	xp_r = x_r;
	xp_b = x_b;
}

void KalmanFilter::predictStateCovarianceForPosition(
		int& op1, int& op2, int& op3, int& op4, int& op5, int& op6,
		int& p1, int& p2, int& p3, int& p4, int& p5, int& p6,
		int& dT) {
// [[1,t,t^2/2],[0,1,t],[0,0,1]] * [[p1,p2,p3],[p4,p5,p6],[p7,p8,p9]] * [[1,0,0],[t,1,0],[t^2/2,t,1]]
// [[1,t,t^2/2],[0,1,t],[0,0,1]] * [[p1,p2,p3],[p2,p4,p5],[p3,p5,p6]] * [[1,0,0],[t,1,0],[t^2/2,t,1]]

	op6 = p6;

	int p6t = i10mult(p6, dT);
	op5 = p6t + p5;

	int p6t2 = i10mult(p6t, dT);
	int p5t = i10mult(p5, t);
	op4 = p6t2 + 2 * p5t + p4;

	int p6t2d2 = (p6t2 >> 1);
	op3 = p6t2d2 + p5t + p3;

	int p6t3d2 = i10mult(p6t2d2, dT);
	int p5t2 = i10mult(p5t,dT);
	int p3t = i10mult(p3, dT);
	int p4t = i10mult(p4, dT);
	op2 = p6t3d2 + p5t2 * 3 / 2 + p3t + p4t + p2;

	int p6t4d4 = i10mult(p6t3d2, dT) / 2;
	op1 = p6t4d4 + i10mult(p5t2 + p3t + p4t + 2 * p2, dT) + p1;
}

void KalmanFilter::predictStateCovarianceForRotations(
		int& op1, int& op2, int& op3, int& op4, int& op5, int& op6,
		int& p1, int& p2, int& p3, int& p4, int& p5, int& p6,
		int& dT) {
	 // [[1,t,-t],[0,1,0],[0,0,1]] * [[p1,p2,p3],[p2,p4,p5],[p3,p5,p6]] * [[1,0,0],[t,1,0],[-t,0,1]]

	op6 = p6; op5 = p5; op4 = p4;
	op3 = p3 + i10mult(p5 - p6, dT);
	op2 = p2 + i10mult(p4 - p5, dT);
	op1 = p1 + i10mult(p2 - p3 + op2 - op3, dT);
}


void inverse3x3(int& a_0_0, int& a_0_1, int& a_0_2,
		int& a_1_0, int& a_1_1, int& a_1_2,
		int& a_2_0, int& a_2_1, int& a_2_2) {
	double determinant = 	+ a_0_0 *( a_1_1 * a_2_2 - a_2_1 * a_1_2 )
		- a_0_1 *( a_1_0 * a_2_2 - a_1_2 * a_2_0 )
		+ a_0_2 *( a_1_0 * a_2_1 - a_1_1 * a_2_0 );
	double invdet = 1/determinant;
	int result_0_0 =  ( a_1_1 * a_2_2 - a_2_1 * a_1_2 )*invdet;
	int result_1_0 = -( a_0_1 * a_2_2 - a_0_2 * a_2_1 )*invdet;
	int result_2_0 =  ( a_0_1 * a_1_2 - a_0_2 * a_1_1 )*invdet;
	int result_0_1 = -( a_1_0 * a_2_2 - a_1_2 * a_2_0 )*invdet;
	int result_1_1 =  ( a_0_0 * a_2_2 - a_0_2 * a_2_0 )*invdet;
	int result_2_1 = -( a_0_0 * a_1_2 - a_1_0 * a_0_2 )*invdet;
	int result_0_2 =  ( a_1_0 * a_2_1 - a_2_0 * a_1_1 )*invdet;
	int result_1_2 = -( a_0_0 * a_2_1 - a_2_0 * a_0_1 )*invdet;
	int result_2_2 =  ( a_0_0 * a_1_1 - a_1_0 * a_0_1 )*invdet;

	// transposed inverse calculated, store actual inverse

	a_0_0 = result_0_0;
	a_1_0 = result_0_1;
	a_2_0 = result_0_2;
	a_0_1 = result_1_0;
	a_1_1 = result_1_1;
	a_2_1 = result_1_2;
	a_0_2 = result_2_0;
	a_1_2 = result_2_1;
	a_2_2 = result_2_2;

}
