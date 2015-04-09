#ifndef KALMAN_CPP
#define KALMAN_CPP
#include "Kalman.h"
inline int i10m(int a, int b) {
    return (((a)*(b))>>10);
}

inline int i10ml(int a, int b) {
    return (((a>>3)*(b>>3))>>4);
}

inline int i10m3(int a, int b, int c) {
    return (((a)*(b)*(c))>>10);
}
using namespace std;

const int gravity = 9192; // 8.98 * 1024, just the average of the magnitude







// Intermediate functions - used at several points in the kalman filter's
// predict and update process



KalmanFilter::KalmanFilter(ofstream& out) {
    fout = &out;
}


void KalmanFilter::initialize(int xBias, int yBias, int zBias) {

    x_Xb = xBias; x_Yb = yBias; x_Zb = zBias;

    x_xp = 60000; x_xv = 0; x_xa = 0;
    x_yp = 0; x_yv = 0; x_ya = 0;
    x_zp = 0; x_zv = 0; x_za = 0;

    x_Xa = 20000; x_Xr = 0;
    x_Ya = 20000; x_Yr = 0;
    x_Za = 20000; x_Zr = 0;

    P_xpp = 50; P_xpv = 50; P_xpa = 50; P_xvv = 50; P_xva = 50; P_xaa = 50;
    P_ypp = 50; P_ypv = 50; P_ypa = 50; P_yvv = 50; P_yva = 50; P_yaa = 50;
    P_zpp = 50; P_zpv = 50; P_zpa = 50; P_zvv = 50; P_zva = 50; P_zaa = 50;
    P_Xaa = 50; P_Xar = 50; P_Xrr = 50;
    P_Yaa = 50; P_Yar = 50; P_Yrr = 50;
    P_Zaa = 50; P_Zar = 50; P_Zrr = 50;


    // Sensor error covariance - Diagonal Matrix
    // (from lab)
    R_ax = 75; R_ay = 75; R_az = 75;
    // 0.000349 r/s^2
    R_gx = 10; R_gy = 10; R_gz = 10;
    R_Px = 500000; R_Py = 500000; R_Pz = 500000;
    // large enough to promote steady state correctness without going overboard
    R_Ox = 600000; R_Oy = 600000; R_Oz = 600000;
}

void KalmanFilter::assignSensorValues(
		int ax, int ay, int az,
		int gx, int gy, int gz,
		int Cx, int Cy,  // cz not used
		int Px, int Py, int Pz,
		bool useGPS)
{

	usingGPS = useGPS;

    // adjust with scaling factor of 0.038307
	z_ax = (ax * 642685) >> 14;
	z_ay = (ay * 642685) >> 14;
	z_az = (az * 642685) >> 14;
    // adjust gyroscope by 0.038307
	z_gx = ((gx - x_Xb) * 1167107) >> 14;
	z_gy = ((gy - x_Yb) * 1167107) >> 14;
	z_gz = ((gz - x_Zb) * 1167107) >> 14;


	if (usingGPS)
	{
		z_Px = Px * 1;
		z_Py = Py * 1;
		z_Pz = Pz * 1;
	} else {
    }


    int cx = trig.cos(x_Xa);
    int cy = trig.cos(x_Ya);
    int cz = trig.cos(x_Za);
    int sx = trig.sin(x_Xa);
    int sy = trig.sin(x_Ya);
    int sz = trig.sin(x_Za);

    int dropX = (int)(trig.atan2(-z_az, -z_ay));
    int dropY = (int)(trig.atan2(-z_ax,  z_az));
	z_Ox = i10m( dropX , cz ) + i10m ( dropY , sz );
	z_Oy = i10m( dropY , cz ) + i10m ( dropX ,-sz );
	z_Oz = (int)(trig.atan2(Cx, Cy));

    H_gx_Xr = H_ax_xa = i10m( cy , cz );
    H_gy_Yr = H_ay_ya = i10m( cz , cx );
    H_gz_Zr = H_az_za = i10m( cx , cy );

    H_gx_Yr = H_ax_ya = i10m( sz , cx );
    H_gy_Xr = H_ay_xa = i10m(-sz , cy );

    H_gx_Zr = H_ax_za = i10m(-sy , cx );
    H_gy_Zr = H_ay_za = i10m( sx , cy );

    H_gz_Xr = H_az_xa = i10m( sy , cz );
    H_gz_Yr = H_az_ya = i10m(-sx , cz );

}


// this function sets the new state variables from the previous ones and the
// updated sensor values.
void KalmanFilter::predictAndUpdate()
{
    nTimesRan++;
    unsigned int currentTime = 0; // TODO: get current time
    int dT = (int)(currentTime - previousTimeRan);

    dT = (1<<10) / 100;

    // Process noise Q matrix
    const int aNoise = 50;
    const int gNoise = 50;

    // Position
    int dT_2 = i10m(dT, dT);
    int dT_3 = i10m(dT_2, dT);
    int dT_4 = i10m(dT_3, dT);

    Q_xpp = Q_ypp = Q_zpp = i10m(dT_4 / 4 , aNoise);
    Q_xpv = Q_ypv = Q_zpv = i10m(dT_3 / 2 , aNoise);
    Q_xpa = Q_ypa = Q_zpa = i10m(dT_2 / 4 , aNoise);
    Q_xvv = Q_yvv = Q_zvv = i10m(dT_2 , aNoise);
    Q_xva = Q_yva = Q_zva = i10m(dT , aNoise);
    Q_xaa = Q_yaa = Q_zaa = i10m(1 , aNoise);

    // Rotation
    Q_Xaa = Q_Yaa = Q_Zaa = i10m(dT_4 / 4 , gNoise);
    Q_Xar = Q_Yar = Q_Zar = i10m(dT_3 / 2 , gNoise);
    Q_Xrr = Q_Yrr = Q_Zrr = i10m(dT_2 , gNoise);

    // predict stage

    // Step 1: PREDICTED (A PRIORI) STATE ESTIMATE =============================

    int xp_xp, xp_xv, xp_xa, xp_yp, xp_yv, xp_ya, xp_zp, xp_zv, xp_za;
    predictStateEstimateForPosition(
            xp_xp, xp_xv, xp_xa, x_xp, x_xv, x_xa, dT);
    predictStateEstimateForPosition(
            xp_yp, xp_yv, xp_ya, x_yp, x_yv, x_ya, dT);
    predictStateEstimateForPosition(
            xp_zp, xp_zv, xp_za, x_zp, x_zv, x_za, dT);

    int xp_Xa, xp_Xr, xp_Xb, xp_Ya, xp_Yr, xp_Yb, xp_Za, xp_Zr, xp_Zb;
    predictStateEstimateForRotation(
            xp_Xa, xp_Xr, xp_Xb, x_Xa, x_Xr, x_Xb, dT);
    predictStateEstimateForRotation(
            xp_Ya, xp_Yr, xp_Yb, x_Ya, x_Yr, x_Yb, dT);
    predictStateEstimateForRotation(
            xp_Za, xp_Zr, xp_Zb, x_Za, x_Zr, x_Zb, dT);


    // Step 2: PREDICTED (A PRIORI) ESTIMATE COVARIANCE ========================
    // P_k = F_k * P_(k-1) * F_k'T + Q_k

    int Pp_xpp, Pp_xpv, Pp_xpa, Pp_xvv, Pp_xva, Pp_xaa;
    int Pp_ypp, Pp_ypv, Pp_ypa, Pp_yvv, Pp_yva, Pp_yaa;
    int Pp_zpp, Pp_zpv, Pp_zpa, Pp_zvv, Pp_zva, Pp_zaa;
    int Pp_Xaa, Pp_Xar, Pp_Xrr; //  Pp_Xbb;
    int Pp_Yaa, Pp_Yar, Pp_Yrr; //  Pp_Ybb;
    int Pp_Zaa, Pp_Zar, Pp_Zrr; //  Pp_Zbb;

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
            Pp_Xaa, Pp_Xar, Pp_Xrr, //Pp_Xbb,
            P_Xaa, P_Xar, P_Xrr,// P_Xbb,
            dT);
    predictStateCovarianceForRotation(
            Pp_Yaa, Pp_Yar, Pp_Yrr, //Pp_Ybb,
            P_Yaa, P_Yar, P_Yrr, //P_Ybb,
            dT);
    predictStateCovarianceForRotation(
            Pp_Zaa, Pp_Zar, Pp_Zrr, //Pp_Zbb,
            P_Zaa, P_Zar, P_Zrr, //P_Zbb,
            dT);

    // degrade confidence by noise
    Pp_xpp += Q_xpp;
    Pp_xpv += Q_xpv;
    Pp_xpa += Q_xpa;
    Pp_xvv += Q_xvv;
    Pp_xva += Q_xva;
    Pp_xaa += Q_xaa;
    Pp_ypp += Q_ypp;
    Pp_ypv += Q_ypv;
    Pp_ypa += Q_ypa;
    Pp_yvv += Q_yvv;
    Pp_yva += Q_yva;
    Pp_yaa += Q_yaa;
    Pp_zpp += Q_zpp;
    Pp_zpv += Q_zpv;
    Pp_zpa += Q_zpa;
    Pp_zvv += Q_zvv;
    Pp_zva += Q_zva;
    Pp_zaa += Q_zaa;
    // rotation
    Pp_Xaa += Q_Xaa;
    Pp_Xrr += Q_Xrr;
    //Pp_Xbb += Q_Xbb;
    Pp_Yaa += Q_Yaa;
    Pp_Yrr += Q_Yrr;
    //Pp_Ybb += Q_Ybb;
    Pp_Zaa += Q_Zaa;
    Pp_Zrr += Q_Zrr;
    // Pp_Zbb += Q_Zbb;

    // UPDATE ==================================================================
    // Step 3: INNOVATION OR MEASUREMENT RESIDUAL ==============================

    int y_ax = i10m(H_ax_xa, xp_xa) +
        i10m(H_ax_ya, xp_ya) +
        i10m(H_ax_za, xp_za - gravity) -
        z_ax;
    int y_ay = i10m(H_ay_xa, xp_xa) +
        i10m(H_ay_ya, xp_ya) +
        i10m(H_ay_za, xp_za - gravity) -
        z_ay;
    int y_az = i10m(H_az_xa, xp_xa) +
        i10m(H_az_ya, xp_ya) +
        i10m(H_az_za, xp_za - gravity) -
        z_az;
    // rotations
    int y_gx = -z_gx +
        // xp_Xb +
        i10m(H_gx_Xr, xp_Xr) +
        i10m(H_gx_Yr, xp_Yr) +
        i10m(H_gx_Zr, xp_Zr);
    int y_gy = -z_gy +
        // xp_Xb +
        i10m(H_gy_Xr, xp_Xr) +
        i10m(H_gy_Yr, xp_Yr) +
        i10m(H_gy_Zr, xp_Zr);
    int y_gz = -z_gz +
        // xp_Zb +
        i10m(H_gz_Xr, xp_Xr) +
        i10m(H_gz_Yr, xp_Yr) +
        i10m(H_gz_Zr, xp_Zr);

    int y_Px = xp_xp - z_Px;
    int y_Py = xp_yp - z_Py;
    int y_Pz = xp_zp - z_Pz;
    int y_Ox = xp_Xa - z_Ox;
    int y_Oy = xp_Ya - z_Oy;
    int y_Oz = xp_Za - z_Oz;



    // Step 4: INNOVATION COVARIANCE ===========================================
    //  step: S_k = H_k * P_k|k-1 * H_k' + R_k
    //
    //  need to figure out for the sensors, how much can we trust them
    //  (given as covariance)
    //  (maybe covarance between the gyroscope and accelerometer variables
    //  is necessary)
    //  - also, R_k is added

    // accelerometer
    int S_axx, S_axy, S_axz, S_ayy, S_ayz, S_azz;
    innovationCovariance(S_axx, S_axy, S_axz,
            S_ayy, S_ayz, S_azz,
            Pp_xaa, Pp_yaa, Pp_zaa, // 0, 0, 0, // no biases
            H_ax_xa, H_ax_ya, H_ax_za,
            H_ay_xa, H_ay_ya, H_ay_za,
            H_az_xa, H_az_ya, H_az_za);

    S_axx += R_ax; S_ayy += R_ay; S_azz += R_az;

    // gyroscope
    int S_gxx, S_gxy, S_gxz, S_gyy, S_gyz, S_gzz;
    innovationCovariance(S_gxx, S_gxy, S_gxz,
            S_gyy, S_gyz, S_gzz,
            Pp_Xrr, Pp_Yrr, Pp_Zrr, // Pp_Xbb, Pp_Ybb, Pp_Zbb,
            H_gx_Xr, H_gx_Yr, H_gx_Zr,
            H_gy_Xr, H_gy_Yr, H_gy_Zr,
            H_gz_Xr, H_gz_Yr, H_gz_Zr);

    S_gxx += R_gx; S_gyy += R_gy; S_gzz += R_gz;

    // GPS
    int S_Pxx = Pp_xpp + R_Px;
    int S_Pyy = Pp_ypp + R_Py;
    int S_Pzz = Pp_zpp + R_Pz;

    // Orientation (pseudo sensor)
    int S_Oxx = Pp_Xaa + R_Ox;
    int S_Oyy = Pp_Yaa + R_Oy;
    int S_Ozz = Pp_Zaa + R_Oz;


    // Step 5: KALMAN GAIN =====================================================
    // computed from:
    // K = P_k|k-1 * H_k'*S_k^-1
    // see step 4.wxm for maxima analysis

    // format: K_<state variable>_<sensor variable>
    // intermediate inverse of S matrix for acceleration
    // format given as: [[i1, i2, i3],[i2, i4, i5],[i3, i5, i6]]
    int Si_axx, Si_axy, Si_axz, Si_ayy, Si_ayz, Si_azz;
    invertSymmetric3x3(Si_axx, Si_axy, Si_axz, Si_ayy, Si_ayz, Si_azz,
            S_axx, S_axy, S_axz, S_ayy, S_ayz, S_azz);

    // gyroscope
    // for differences between this and acceleration, since we now incorporate
    // biases, compare previous maxima matrix multiplication with:
    // step 5-7 gyro.wxm
    // int K_Xr_gx, K_Xr_gy, K_Xr_gz;
    // int K_Yr_gx, K_Yr_gy, K_Yr_gz;
    // int K_Zr_gx, K_Zr_gy, K_Zr_gz;

    int Si_gxx, Si_gxy, Si_gxz, Si_gyy, Si_gyz, Si_gzz;
    invertSymmetric3x3(Si_gxx, Si_gxy, Si_gxz, Si_gyy, Si_gyz, Si_gzz,
            S_gxx, S_gxy, S_gxz, S_gyy, S_gyz, S_gzz);
    // kalmanGain3x3x3(
    //         // outputs
    //         K_Xr_gx, K_Xr_gy, K_Xr_gz,
    //         K_Yr_gx, K_Yr_gy, K_Yr_gz,
    //         K_Zr_gx, K_Zr_gy, K_Zr_gz,
    //         // three Pp's needed
    //         Pp_Xrr, Pp_Yrr, Pp_Zrr,
    //         // transpose of H matrix
    //         H_gx_Xr, H_gy_Xr, H_gz_Xr,
    //         H_gx_Yr, H_gy_Yr, H_gz_Yr,
    //         H_gx_Zr, H_gy_Zr, H_gz_Zr,
    //         // 6 unique elements of symmetrical inverse matrix
    //         ig1, ig2, ig3, ig4, ig5, ig6);

    // GPS and Orientation-pseudo-sensors

    // intermediates for multiplying by covariance
    int Si_Pxx = ((1<<20) / S_Pxx );
    int Si_Pyy = ((1<<20) / S_Pyy );
    int Si_Pzz = ((1<<20) / S_Pzz );

    // intermediates
    int Ht_SiP1 = H_az_xa * Si_axz + H_ay_xa * Si_axy + H_ax_xa * Si_axx ;
    int Ht_SiP2 = H_az_xa * Si_ayz + H_ay_xa * Si_ayy + H_ax_xa * Si_axy ;
    int Ht_SiP3 = H_az_xa * Si_azz + H_ay_xa * Si_ayz + H_ax_xa * Si_axz ;

    int Ht_SiP5 = H_az_ya * Si_ayz + H_ay_ya * Si_ayy + H_ax_ya * Si_axy ;
    int Ht_SiP4 = H_az_ya * Si_axz + H_ay_ya * Si_axy + H_ax_ya * Si_axx ;
    int Ht_SiP6 = H_az_ya * Si_azz + H_ay_ya * Si_ayz + H_ax_ya * Si_axz ;

    int Ht_SiP7 = H_az_za * Si_axz + H_ay_za * Si_axy + H_ax_za * Si_axx ;
    int Ht_SiP8 = H_az_za * Si_ayz + H_ay_za * Si_ayy + H_ax_za * Si_axy ;
    int Ht_SiP9 = H_az_za * Si_azz + H_ay_za * Si_ayz + H_ax_za * Si_axz ;

    // actual kalman gains
    int K_xp_ax = (P_xpa * (Ht_SiP1 >> 4)) >> 16;
    int K_xp_ay = (P_xpa * (Ht_SiP2 >> 4)) >> 16;
    int K_xp_az = (P_xpa * (Ht_SiP3 >> 4)) >> 16;
    int K_yp_ax = (P_ypa * (Ht_SiP4 >> 4)) >> 16;
    int K_yp_ay = (P_ypa * (Ht_SiP5 >> 4)) >> 16;
    int K_yp_az = (P_ypa * (Ht_SiP6 >> 4)) >> 16;
    int K_zp_ax = (P_zpa * (Ht_SiP7 >> 4)) >> 16;
    int K_zp_ay = (P_zpa * (Ht_SiP8 >> 4)) >> 16;
    int K_zp_az = (P_zpa * (Ht_SiP9 >> 4)) >> 16;

    int K_xv_ax = (P_xva * (Ht_SiP1 >> 4)) >> 16;
    int K_xv_ay = (P_xva * (Ht_SiP2 >> 4)) >> 16;
    int K_xv_az = (P_xva * (Ht_SiP3 >> 4)) >> 16;
    int K_yv_ax = (P_yva * (Ht_SiP4 >> 4)) >> 16;
    int K_yv_ay = (P_yva * (Ht_SiP5 >> 4)) >> 16;
    int K_yv_az = (P_yva * (Ht_SiP6 >> 4)) >> 16;
    int K_zv_ax = (P_zva * (Ht_SiP7 >> 4)) >> 16;
    int K_zv_ay = (P_zva * (Ht_SiP8 >> 4)) >> 16;
    int K_zv_az = (P_zva * (Ht_SiP9 >> 4)) >> 16;

    int K_xa_ax = (P_xaa * (Ht_SiP1 >> 4)) >> 16;
    int K_xa_ay = (P_xaa * (Ht_SiP2 >> 4)) >> 16;
    int K_xa_az = (P_xaa * (Ht_SiP3 >> 4)) >> 16;
    int K_ya_ax = (P_yaa * (Ht_SiP4 >> 4)) >> 16;
    int K_ya_ay = (P_yaa * (Ht_SiP5 >> 4)) >> 16;
    int K_ya_az = (P_yaa * (Ht_SiP6 >> 4)) >> 16;
    int K_za_ax = (P_zaa * (Ht_SiP7 >> 4)) >> 16;
    int K_za_ay = (P_zaa * (Ht_SiP8 >> 4)) >> 16;
    int K_za_az = (P_zaa * (Ht_SiP9 >> 4)) >> 16;

    // position
    int K_xp_Px; int K_yp_Py; int K_zp_Pz;
    int K_xv_Px; int K_yv_Py; int K_zv_Pz;
    int K_xa_Px; int K_ya_Py; int K_za_Pz;

    if (usingGPS) {
        K_xp_Px = i10m( P_xpp , Si_Pxx );
        K_yp_Py = i10m( P_ypp , Si_Pyy );
        K_zp_Pz = i10m( P_zpp , Si_Pzz );
        K_xv_Px = i10m( P_xpv , Si_Pxx );
        K_yv_Py = i10m( P_ypv , Si_Pyy );
        K_zv_Pz = i10m( P_zpv , Si_Pzz );
        K_xa_Px = i10m( P_xpa , Si_Pxx );
        K_ya_Py = i10m( P_ypa , Si_Pyy );
        K_za_Pz = i10m( P_zpa , Si_Pzz );
    } else {
        K_xp_Px = 0; K_yp_Py = 0; K_zp_Pz = 0;
        K_xv_Px = 0; K_yv_Py = 0; K_zv_Pz = 0;
        K_xa_Px = 0; K_ya_Py = 0; K_za_Pz = 0;
    }


    // Orientation

    // intermediates for orientation correcting
    int Si_Oxx = ((1<<20) / S_Oxx );
    int Si_Oyy = ((1<<20) / S_Oyy );
    int Si_Ozz = ((1<<20) / S_Ozz );


    // intermediates from gyroscope, UNSCALED
    int Ht_SiO1 = H_gz_Xr * Si_gxz + H_gy_Xr * Si_gxy + H_gx_Xr * Si_gxx;
    int Ht_SiO2 = H_gz_Xr * Si_gyz + H_gy_Xr * Si_gyy + H_gx_Xr * Si_gxy;
    int Ht_SiO3 = H_gz_Xr * Si_gzz + H_gy_Xr * Si_gyz + H_gx_Xr * Si_gxz;

    int Ht_SiO5 = H_gz_Yr * Si_gyz + H_gy_Yr * Si_gyy + H_gx_Yr * Si_gxy;
    int Ht_SiO4 = H_gz_Yr * Si_gxz + H_gy_Yr * Si_gxy + H_gx_Yr * Si_gxx;
    int Ht_SiO6 = H_gz_Yr * Si_gzz + H_gy_Yr * Si_gyz + H_gx_Yr * Si_gxz;

    int Ht_SiO7 = H_gz_Zr * Si_gxz + H_gy_Zr * Si_gxy + H_gx_Zr * Si_gxx;
    int Ht_SiO8 = H_gz_Zr * Si_gyz + H_gy_Zr * Si_gyy + H_gx_Zr * Si_gxy;
    int Ht_SiO9 = H_gz_Zr * Si_gzz + H_gy_Zr * Si_gyz + H_gx_Zr * Si_gxz;


    int K_Xa_gx = i10m( P_Xar , Ht_SiO1 ) >> 10;
    int K_Xa_gy = i10m( P_Xar , Ht_SiO2 ) >> 10;
    int K_Xa_gz = i10m( P_Xar , Ht_SiO3 ) >> 10;

    int K_Ya_gx = i10m( P_Yar , Ht_SiO5 ) >> 10;
    int K_Ya_gy = i10m( P_Yar , Ht_SiO4 ) >> 10;
    int K_Ya_gz = i10m( P_Yar , Ht_SiO6 ) >> 10;

    int K_Za_gx = i10m( P_Zar , Ht_SiO7 ) >> 10;
    int K_Za_gy = i10m( P_Zar , Ht_SiO8 ) >> 10;
    int K_Za_gz = i10m( P_Zar , Ht_SiO9 ) >> 10;

    int K_Xr_gx = i10m( P_Xrr , Ht_SiO1 ) >> 10;
    int K_Xr_gy = i10m( P_Xrr , Ht_SiO2 ) >> 10;
    int K_Xr_gz = i10m( P_Xrr , Ht_SiO3 ) >> 10;

    int K_Yr_gx = i10m( P_Yrr , Ht_SiO5 ) >> 10;
    int K_Yr_gy = i10m( P_Yrr , Ht_SiO4 ) >> 10;
    int K_Yr_gz = i10m( P_Yrr , Ht_SiO6 ) >> 10;

    int K_Zr_gx = i10m( P_Zrr , Ht_SiO7 ) >> 10;
    int K_Zr_gy = i10m( P_Zrr , Ht_SiO8 ) >> 10;
    int K_Zr_gz = i10m( P_Zrr , Ht_SiO9 ) >> 10;

    int K_Xa_Ox = (P_Xaa*Si_Oxx) >> 10;
    int K_Ya_Oy = (P_Yaa*Si_Oyy) >> 10;
    int K_Za_Oz = (P_Zaa*Si_Ozz) >> 10;
    int K_Xr_Ox = (P_Xar*Si_Oxx) >> 10;
    int K_Yr_Oy = (P_Yar*Si_Oyy) >> 10;
    int K_Zr_Oz = (P_Zar*Si_Ozz) >> 10;

    log(P_Xaa);

    logr();

    // Step 6: UPDATED STATE ESTIMATE ==========================================
    // bit simpler: x_k|k = x_x|x-1 + K_k * y_k
    // acceleration gets updates from 3,
    // angular velocity get updates from 3,
    // bias gets updates from three (surprisingly. might need fixing.
    //  TODO: recheck why each bias depends on values from all axes,
    // position and angle each get one from each sensor

    // from accelerometer
    x_xp = xp_xp + (K_xp_ax * y_ax + K_xp_ay * y_ay + K_xp_az * y_az + K_xp_Px * y_Px) >> 10;
    x_yp = xp_yp + (K_yp_ax * y_ax + K_yp_ay * y_ay + K_yp_az * y_az + K_yp_Py * y_Py) >> 10;
    x_zp = xp_zp + (K_zp_ax * y_ax + K_zp_ay * y_ay + K_zp_az * y_az + K_zp_Pz * y_Pz) >> 10;

    x_xv = xp_xv + (K_xv_ax * y_ax + K_xv_ay * y_ay + K_xv_az * y_az + K_xv_Px * y_Px) >> 10;
    x_yv = xp_yv + (K_yv_ax * y_ax + K_yv_ay * y_ay + K_yv_az * y_az + K_yv_Py * y_Py) >> 10;
    x_zv = xp_zv + (K_zv_ax * y_ax + K_zv_ay * y_ay + K_zv_az * y_az + K_zv_Pz * y_Pz) >> 10;

    x_xa = xp_xa + (K_xa_ax * y_ax + K_xa_ay * y_ay + K_xa_az * y_az + K_xa_Px * y_Px) >> 10;
    x_ya = xp_ya + (K_ya_ax * y_ax + K_ya_ay * y_ay + K_ya_az * y_az + K_ya_Py * y_Py) >> 10;
    x_za = xp_za + (K_za_ax * y_ax + K_za_ay * y_ay + K_za_az * y_az + K_za_Pz * y_Pz) >> 10;


    // gyroscope updates to both x_*r
    x_Xa = xp_Xa + (K_Xa_gx * y_gx + K_Xa_gy * y_gy + K_Xa_gz * y_gz + K_Xa_Ox * y_Ox) >> 10;
    x_Ya = xp_Ya + (K_Ya_gx * y_gx + K_Ya_gy * y_gy + K_Ya_gz * y_gz + K_Ya_Oy * y_Oy) >> 10;
    x_Za = xp_Za + (K_Za_gx * y_gx + K_Za_gy * y_gy + K_Za_gz * y_gz + K_Za_Oz * y_Oz) >> 10;

    x_Xr = xp_Xr + (K_Xr_gx * y_gx + K_Xr_gy * y_gy + K_Xr_gz * y_gz + K_Xr_Ox * y_Ox) >> 10;
    x_Yr = xp_Yr + (K_Yr_gx * y_gx + K_Yr_gy * y_gy + K_Yr_gz * y_gz + K_Yr_Oy * y_Oy) >> 10;
    x_Zr = xp_Zr + (K_Zr_gx * y_gx + K_Zr_gy * y_gy + K_Zr_gz * y_gz + K_Zr_Oz * y_Oz) >> 10;



    // Step 7: UPDATED ESTIMATE COVARIANCE =====================================
    // P_k|k = (<I> - K_k * H_k) * P_k|k-1
    // see "step 5.wxm" last line for analysis

    // intermediates for optimizing
    int Pi_1_Kx = (1 << 10) - K_xp_Px;
    int Pi_1_Ky = (1 << 10) - K_yp_Py;
    int Pi_1_Kz = (1 << 10) - K_zp_Pz;

    int Pi_mess_xa = i10m( -H_az_xa , K_xp_az ) - i10m( H_ay_xa , K_xp_ay ) - i10m( H_ax_xa , K_xp_ax );
    int Pi_mess_ya = i10m( -H_az_ya , K_yp_az ) - i10m( H_ay_ya , K_yp_ay ) - i10m( H_ax_ya , K_yp_ax );
    int Pi_mess_za = i10m( -H_az_za , K_zp_az ) - i10m( H_ay_za , K_zp_ay ) - i10m( H_ax_za , K_zp_ax );

    P_xpp = i10m( Pi_1_Kx , Pp_xpp ) + i10m( Pi_mess_xa , Pp_xpa );
    P_xpv = i10m( Pi_1_Kx , Pp_xpv ) + i10m( Pi_mess_xa , Pp_xva );
    P_xpa = i10m( Pi_1_Kx , Pp_xpa ) + i10m( Pi_mess_xa , Pp_xaa );

    P_ypp = i10m( Pi_1_Ky , Pp_ypp ) + i10m( Pi_mess_ya , Pp_ypa );
    P_ypv = i10m( Pi_1_Ky , Pp_ypv ) + i10m( Pi_mess_ya , Pp_yva );
    P_ypa = i10m( Pi_1_Ky , Pp_ypa ) + i10m( Pi_mess_ya , Pp_yaa );

    P_zpp = i10m( Pi_1_Kz , Pp_zpp ) + i10m( Pi_mess_za , Pp_zpa );
    P_zpv = i10m( Pi_1_Kz , Pp_zpv ) + i10m( Pi_mess_za , Pp_zva );
    P_zpa = i10m( Pi_1_Kz , Pp_zpa ) + i10m( Pi_mess_za , Pp_zaa );

    // more intermediates that are the same
    int Pi_mess_xv = i10m( -H_az_xa , K_xv_az ) - i10m( H_ay_xa , K_xv_ay ) - i10m( H_ax_xa , K_xv_ax );
    int Pi_mess_yv = i10m( -H_az_ya , K_yv_az ) - i10m( H_ay_ya , K_yv_ay ) - i10m( H_ax_ya , K_yv_ax );
    int Pi_mess_zv = i10m( -H_az_za , K_zv_az ) - i10m( H_ay_za , K_zv_ay ) - i10m( H_ax_za , K_zv_ax );

    P_xvv = Pp_xvv - i10m( K_xv_Px , Pp_xpv ) + i10m( Pi_mess_xv , Pp_xva );
    P_xva = Pp_xva - i10m( K_xv_Px , Pp_xpa ) + i10m( Pi_mess_xv , Pp_xaa );

    P_yvv = Pp_yvv - i10m( K_yv_Py , Pp_ypv ) + i10m( Pi_mess_yv , Pp_yva );
    P_yva = Pp_yva - i10m( K_yv_Py , Pp_ypa ) + i10m( Pi_mess_yv , Pp_yaa );

    P_zvv = Pp_zvv - i10m( K_zv_Pz , Pp_zpv ) + i10m( Pi_mess_zv , Pp_zva );
    P_zva = Pp_zva - i10m( K_zv_Pz , Pp_zpa ) + i10m( Pi_mess_zv , Pp_zaa );


    P_xaa = i10m( (i10m( -H_az_xa , K_xa_az ) - i10m( H_ay_xa , K_xa_ay ) - i10m( H_ax_xa , K_xa_ax+(1 << 10) )) , Pp_xaa - i10m( K_xa_Px , Pp_xpa ) );
    P_yaa = i10m( (i10m( -H_az_ya , K_ya_az ) - i10m( H_ay_ya , K_ya_ay ) - i10m( H_ax_ya , K_ya_ax+(1 << 10) )) , Pp_yaa - i10m( K_ya_Py , Pp_ypa ) );
    P_zaa = i10m( (i10m( -H_az_za , K_za_az ) - i10m( H_ay_za , K_za_ay ) - i10m( H_ax_za , K_za_ax+(1 << 10) )) , Pp_zaa - i10m( K_za_Pz , Pp_zpa ) );

    // ORIENTATION

    P_Xaa = i10m( ( -i10m( H_gz_Xr , K_Xa_gz ) - i10m( H_gy_Xr , K_Xa_gy ) - i10m( H_gx_Xr , K_Xa_gx ) ) , Pp_Xar) + i10m( (1-K_Xa_Ox) , Pp_Xaa );
    P_Xar = i10m( ( -i10m( H_gz_Xr , K_Xa_gz ) - i10m( H_gy_Xr , K_Xa_gy ) - i10m( H_gx_Xr , K_Xa_gx ) ) , Pp_Xrr) + i10m( (1-K_Xa_Ox) , Pp_Xar );
    P_Yaa = i10m( ( -i10m( H_gz_Yr , K_Ya_gz ) - i10m( H_gy_Yr , K_Ya_gy ) - i10m( H_gx_Yr , K_Ya_gx ) ) , Pp_Yar) + i10m( (1-K_Ya_Oy) , Pp_Yaa );
    P_Yar = i10m( ( -i10m( H_gz_Yr , K_Ya_gz ) - i10m( H_gy_Yr , K_Ya_gy ) - i10m( H_gx_Yr , K_Ya_gx ) ) , Pp_Yrr) + i10m( (1-K_Ya_Oy) , Pp_Yar );
    P_Zaa = i10m( ( -i10m( H_gz_Zr , K_Za_gz ) - i10m( H_gy_Zr , K_Za_gy ) - i10m( H_gx_Zr , K_Za_gx ) ) , Pp_Zar) + i10m( (1-K_Za_Oz) , Pp_Zaa );
    P_Zar = i10m( ( -i10m( H_gz_Zr , K_Za_gz ) - i10m( H_gy_Zr , K_Za_gy ) - i10m( H_gx_Zr , K_Za_gx ) ) , Pp_Zrr) + i10m( (1-K_Za_Oz) , Pp_Zar );

    P_Xrr = i10m( ( -i10m( H_gz_Xr , K_Xr_gz ) - i10m( H_gy_Xr , K_Xr_gy ) - i10m( H_gx_Xr , K_Xr_gx ) +1 ) , Pp_Xrr) - i10m( K_Xr_Ox , Pp_Xar );
    P_Yrr = i10m( ( -i10m( H_gz_Yr , K_Yr_gz ) - i10m( H_gy_Yr , K_Yr_gy ) - i10m( H_gx_Yr , K_Yr_gx ) +1 ) , Pp_Yrr) - i10m( K_Yr_Oy , Pp_Yar );
    P_Zrr = i10m( ( -i10m( H_gz_Zr , K_Zr_gz ) - i10m( H_gy_Zr , K_Zr_gy ) - i10m( H_gx_Zr , K_Zr_gx ) +1 ) , Pp_Zrr) - i10m( K_Zr_Oz , Pp_Zar );






} // END OF FILTER =============================================================






quadState_t KalmanFilter::getQuadState() {
    quadState_t state;
    state.xPosition = x_xp;
    state.yPosition = x_yp;
    state.zPosition = x_zp;
    state.xVelocity = x_xv;
    state.yVelocity = x_yv;
    state.zVelocity = x_zv;
    state.xAcceleration = x_xa;
    state.yAcceleration = x_ya;
    state.zAcceleration = x_za;
    state.xAngle = x_Xa;
    state.yAngle = x_Ya;
    state.zAngle = x_Za;
    state.xRotation = x_Xr;
    state.yRotation = x_Yr;
    state.zRotation = x_Zr;
    return state;
}

quadState_t KalmanFilter::getCovariance() {
    quadState_t cov;
    cov.xPosition = P_xpp;
    cov.yPosition = P_ypp;
    cov.zPosition = P_zpp;
    cov.xVelocity = P_xvv;
    cov.yVelocity = P_yvv;
    cov.zVelocity = P_zvv;
    cov.xAcceleration = P_xaa;
    cov.yAcceleration = P_yaa;
    cov.zAcceleration = P_zaa;
    cov.xAngle = P_Xaa;
    cov.yAngle = P_Yaa;
    cov.zAngle = P_Zaa;
    cov.xRotation = P_Xrr;
    cov.yRotation = P_Yrr;
    cov.zRotation = P_Zrr;
    return cov;
}


void KalmanFilter::predictStateEstimateForPosition(
		int& xp_p, int& xp_v, int& xp_a,
		int& x_p, int& x_v, int& x_a,
		int& dT)
{
	// state transition matrix is:
	// 	xp	xv	xa
	// xp	1	t	t^2/2
	// xv	0	1	t
	// xa	0	0	1

	xp_p = x_p + i10m(x_v, dT) + i10m(i10m(x_a, dT), dT) / 2;
	xp_v = x_v + i10m(x_a, dT);
	xp_a = x_a;
}

void KalmanFilter::predictStateEstimateForRotation(
		int& xp_a, int& xp_r, int& xp_b,
		int& x_a, int& x_r, int& x_b,
		int& dT)
{
	// state transition matrix (rotations):
	// 	Xa	Xr	Xb
	// Xa	1	dT	0
	// Xr	0	1	0
	// Xb	0	0	1

	xp_a = x_a + i10m(x_r, dT);
	xp_r = x_r;
	xp_b = x_b;
}

void KalmanFilter::predictStateCovarianceForPosition(
		int& op1, int& op2, int& op3, int& op4, int& op5, int& op6,
		int& p1, int& p2, int& p3, int& p4, int& p5, int& p6,
		int& dT)
{
// maxima: matrix([1,t,t^2/2],[0,1,t],[0,0,1]) . matrix([p1,p2,p3],[p2,p4,p5],[p3,p5,p6]) . matrix([1,0,0],[t,1,0],[t^2/2,t,1]);
// (then simplify)

	op6 = p6;

	int p6t = i10m(p6, dT);
	op5 = p6t + p5;

	int p6t2 = i10m(p6t, dT);
	int p5t = i10m(p5, dT);
	op4 = p6t2 + 2 * p5t + p4;

	int p6t2d2 = (p6t2 / 2);
	op3 = p6t2d2 + p5t + p3;

	int p6t3d2 = i10m(p6t2d2, dT);
	int p5t2 = i10m(p5t,dT);
	int p3t = i10m(p3, dT);
	int p4t = i10m(p4, dT);
	op2 = p6t3d2 + p5t2 * 3 / 2 + p3t + p4t + p2;

	int p6t4d4 = i10m(p6t3d2, dT) / 2;
	op1 = p6t4d4 + i10m(p5t2 + p3t + p4t + 2 * p2, dT) + p1;
}

void KalmanFilter::predictStateCovarianceForRotation(
		int& op1, int& op2, int& op3, // int& op4,
		int& p1, int& p2, int& p3, // int& p4,
		int& dT)
{
	// [[1,t,0],[0,1,0],[0,0,1]] * [[p1,p2,0],[p2,p3,0],[0,0,p4]] * [[1,0,0],[t,1,0],[0,0,1]]
	// op4 = p4;
	op3 = p3;
	op2 = p2 + i10m(p3, dT);
	op1 = p1 + i10m(op2 + p2, dT);
}


void KalmanFilter::innovationCovariance(
        int& s1, int& s2, int& s3, int& s4, int& s5, int& s6,
        int p1, int p2, int p3,//  int p4, int p5, int p6,
        int h1, int h2, int h3,
        int h4, int h5, int h6,
        int h7, int h8, int h9)
{
    /**
     * covar
     * represents multiplication of:
     * hmatrix : matrix([h1,h2,h3,1,0,0],[h4,h5,h6,0,1,0],[h7,h8,h9,0,0,1]);
     * pmatrix : diagonal of p1 - p6
     * hmatrix . pmatrix . transpose(hmatrix);
     * stored into:
     * [s1 s2 s3]
     * [s2 s4 s5]
     * [s3 s5 s6]
     */

    s1 = /* p4+ */ i10m3(h3,h3,p3)+i10m3(h2,h2,p2)+i10m3(h1,h1,p1);
    s2 = /*     */ i10m3(h3,h6,p3)+i10m3(h2,h5,p2)+i10m3(h1,h4,p1);
    s3 = /*     */ i10m3(h3,h9,p3)+i10m3(h2,h8,p2)+i10m3(h1,h7,p1);
    s4 = /* p5+ */ i10m3(h6,h6,p3)+i10m3(h5,h5,p2)+i10m3(h4,h4,p1);
    s5 = /*     */ i10m3(h6,h9,p3)+i10m3(h5,h8,p2)+i10m3(h4,h7,p1);
    s6 = /* p6+ */ i10m3(h9,h9,p3)+i10m3(h8,h8,p2)+i10m3(h7,h7,p1);
}

void KalmanFilter::kalmanGain3x3x3(
        int& k1, int& k2, int& k3,
        int& k4, int& k5, int& k6,
        int& k7, int& k8, int& k9,
        int p1, int p2, int p3,
        int h1, int h2, int h3,
        int h4, int h5, int h6,
        int h7, int h8, int h9,
        int i1, int i2, int i3, int i4, int i5, int i6)
{
    // verify through maxima:
    // matrix([p1,0,0],[0,p2,0],[0,0,p3]) . matrix([h1,h2,h3],[h4,h5,h6],[h7,h8,h9]) . matrix([i1,i2,i3],[i2,i4,i5],[i3,i5,i6]);
    //
    //
    // TODO: add in p2, p3 for each of xyz variables
    // to account for the fact that there is covariance between position,
    // velocity and acceleration that will take the y and apply it to the
    // estimate for velocity and acceleration as well.
    // this is where velocity is corrected.

    k1 = i10m( ( i10m(h3,i3) + i10m(h2,i2) + i10m(h1,i1) ) , p1 );
    k2 = i10m( ( i10m(h3,i5) + i10m(h2,i4) + i10m(h1,i2) ) , p1 );
    k3 = i10m( ( i10m(h3,i6) + i10m(h2,i5) + i10m(h1,i3) ) , p1 );
    k4 = i10m( ( i10m(h6,i3) + i10m(h5,i2) + i10m(h4,i1) ) , p2 );
    k5 = i10m( ( i10m(h6,i5) + i10m(h5,i4) + i10m(h4,i2) ) , p2 );
    k6 = i10m( ( i10m(h6,i6) + i10m(h5,i5) + i10m(h4,i3) ) , p2 );
    k7 = i10m( ( i10m(h9,i3) + i10m(h8,i2) + i10m(h7,i1) ) , p3 );
    k8 = i10m( ( i10m(h9,i5) + i10m(h8,i4) + i10m(h7,i2) ) , p3 );
    k9 = i10m( ( i10m(h9,i6) + i10m(h8,i5) + i10m(h7,i3) ) , p3 );
}



void KalmanFilter::invertSymmetric3x3(
        int& o1, int& o2, int& o3, int& o4, int& o5, int& o6,
        int i1, int i2, int i3, int i4, int i5, int i6)
{
    // to check: wolfram alpha:
    //  invert([[s1,s2,s3],[s2,s4,s5],[s3,s5,s6]])
    int determinant =
        +i1*(i4*i6-i5*i5)
        -i2*(i2*i6-i5*i3)
        +i3*(i2*i5-i4*i3);

    int invdet = ((1<<30) / determinant );

    // log(i1); log(i2); log(i3); log(i4); log(i5); log(i6);

    // log(determinant);
    // log(invdet);
    // logr();

    bool scaleInv = invdet > (1 << 16);
    if (scaleInv) { invdet = invdet >> 10; }
    invdet = -invdet;

    o1 = (( (i5*i5) - (i4*i6) ) * invdet);
    o2 = (( (i2*i6) - (i3*i5) ) * invdet);
    o3 = (( (i3*i4) - (i2*i5) ) * invdet);
    o4 = (( (i3*i3) - (i1*i6) ) * invdet);
    o5 = (( (i1*i5) - (i2*i3) ) * invdet);
    o6 = (( (i2*i2) - (i1*i4) ) * invdet);

    if (scaleInv) {
        o1 = o1 >> 10; o2 = o2 >> 10; o3 = o3 >> 10;
        o4 = o4 >> 10; o5 = o5 >> 10; o6 = o6 >> 10;
    } else {
        o1 = o1 >> 20; o2 = o2 >> 20; o3 = o3 >> 20;
        o4 = o4 >> 20; o5 = o5 >> 20; o6 = o6 >> 20;
    }

    // log(o1); log(o2); log(o3); log(o4); log(o5); log(o6); logr();

}
#endif
