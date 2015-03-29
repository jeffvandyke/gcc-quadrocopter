#ifndef KALMAN_CPP
#define KALMAN_CPP
#include "kalman.h"
inline int i10mult(int a, int b) {
    return (((a)*(b))>>10);
}

inline int i10multl(int a, int b) {
    return (((a>>3)*(b>>3))>>4);
}

inline int i10mult3(int a, int b, int c) {
    return (((a)*(b)*(c))>>10);
}
using namespace std;

const int gravity = 10035; // 9.8 * 1024







// Intermediate functions - used at several points in the kalman filter's
// predict and update process
void predictStateEstimateForPosition(
		int& xp_p, int& xp_v, int& xp_a,
		int& x_p, int& x_v, int& x_a,
		int& dT);
void predictStateEstimateForRotation(
		int& xp_a, int& xp_r, int& xp_b,
		int& x_a, int& x_r, int& x_b,
		int& dT);
void predictStateCovarianceForPosition(
		int& op1, int& op2, int& op3, int& op4, int& op5, int& op6,
		int& p1, int& p2, int& p3, int& p4, int& p5, int& p6,
		int& dT);
void predictStateCovarianceForRotation(
		int& op1, int& op2, int& op3, int& op4,
		int& p1, int& p2, int& p3, int& p4,
		int& dT);
void innovationCovariance(
        int& s1, int& s2, int& s3, int& s4, int& s5, int& s6,
        int p1, int p2, int p3, int p4, int p5, int p6,
        int h1, int h2, int h3,
        int h4, int h5, int h6,
        int h7, int h8, int h9);
void kalmanGain3x3x3(
        int& k1, int& k2, int& k3,
        int& k4, int& k5, int& k6,
        int& k7, int& k8, int& k9,
        int p1, int p2, int p3,
        int h1, int h2, int h3,
        int h4, int h5, int h6,
        int h7, int h8, int h9,
        int i1, int i2, int i3, int i4, int i5, int i6);


void invert3x3(int&, int&, int&, int&, int&, int&, int&, int&, int&,
        int, int, int, int, int, int, int, int, int);
void invertSymmetric3x3(int& o1, int& o2, int& o3, int& o4, int& o5, int& o6,
        int i1, int i2, int i3, int i4, int i5, int i6);



KalmanFilter::KalmanFilter() {

}


void KalmanFilter::initialize(int xBias, int yBias, int zBias) {

    x_Xb = xBias; x_Yb = yBias; x_Zb = zBias;
    x_Xa = 0; x_Xr = 0;
    x_Ya = 0; x_Yr = 0;
    x_Za = 0; x_Zr = 0;

    x_xp = 0; x_xv = 0; x_xa = 0;
    x_yp = 0; x_yv = 0; x_ya = 0;
    x_zp = 0; x_zv = 0; x_za = 0;


    // Sensor error covariance - Diagonal Matrix
    // (from lab)
    R_ax = 75; R_ay = 75; R_az = 75;
    // 0.000349 r/s^2
    R_gx = 1; R_gy = 1; R_gz = 1;
    R_Px = 5000; R_Py = 5000; R_Pz = 5000;
    // large enough to promote steady state correctness without going overboard
    R_Ox = 6000; R_Oy = 6000; R_Oz = 6000;
}

void KalmanFilter::assignSensorValues(
		int ax, int ay, int az,
		int gx, int gy, int gz,
		int Cx, int Cy,  // cz not used
		int Px, int Py, int Pz,
		// int Vx, int Vy, int Vz,
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
	}

	z_Ox = (int)(trig.atan2(z_ay, z_az) * 1024);
	z_Oy = (int)(trig.atan2(z_ax, z_az) * 1024);
	z_Oz = (int)(trig.atan2(Cx, Cy) * 1024);

    // TODO: adjust H matrix for acceleration and gyroscope

    int cx = trig.cos(x_Xa);
    int cy = trig.cos(x_Ya);
    int cz = trig.cos(x_Za);
    int sx = trig.sin(x_Xa);
    int sy = trig.sin(x_Ya);
    int sz = trig.sin(x_Za);

    H_gx_Xr = H_ax_xa = i10mult( cy , cz );
    H_gy_Yr = H_ay_ya = i10mult( cz , cx );
    H_gz_Zr = H_az_za = i10mult( cx , cy );
          
    H_gx_Yr = H_ax_ya = i10mult( sz , cx );
    H_gy_Xr = H_ay_xa = i10mult(-sz , cy );
          
    H_gx_Zr = H_ax_za = i10mult(-sy , cx );
    H_gy_Zr = H_ay_za = i10mult( sx , cy );
          
    H_gz_Xr = H_az_xa = i10mult( sy , cz );
    H_gz_Yr = H_az_ya = i10mult(-sx , cz );
                             
    
    
    
    
    
    
    
    
    
    
    
    

}


// this function sets the new state variables from the previous ones and the
// updated sensor values. 
void KalmanFilter::predictAndUpdate()
{
    nTimesRan++;
    unsigned int currentTime = 0; // TODO: get current time 
    int dT = (int)(currentTime - previousTimeRan);

    // Process noise Q matrix
    const int aNoise = 50;
    const int gNoise = 50;
    // Position
    Q_xpp = 64;
    Q_xvv = 64;
    Q_xaa = 64;

    Q_ypp = 64;
    Q_yvv = 64;
    Q_yaa = 64;

    Q_zpp = 64;
    Q_zvv = 64;
    Q_zaa = 64;

    //Rotation
    Q_Xaa = 64;
    Q_Xrr = 64;
    // Q_Xbb = 0;

    Q_Yaa = 64;
    Q_Yrr = 64;
    // Q_Ybb = 0;

    Q_Zaa = 64;
    Q_Zrr = 64;
    // Q_Zbb = 0;
    // expect time in ms

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

    int y_ax = i10mult(H_ax_xa, xp_xa) + 
        i10mult(H_ax_ya, xp_ya) + 
        i10mult(H_ax_za, xp_za - gravity) -
        z_ax;
    int y_ay = i10mult(H_ay_xa, xp_xa) + 
        i10mult(H_ay_ya, xp_ya) + 
        i10mult(H_ay_za, xp_za - gravity) -
        z_ay;
    int y_az = i10mult(H_az_xa, xp_xa) + 
        i10mult(H_az_ya, xp_ya) + 
        i10mult(H_az_za, xp_za - gravity) -
        z_az;
    // rotations
    int y_gx = -z_gx +
        // xp_Xb +
        i10mult(H_gx_Xr, xp_Xr) + 
        i10mult(H_gx_Yr, xp_Yr) +
        i10mult(H_gx_Zr, xp_Zr);
    int y_gy = -z_gy +
        // xp_Xb +
        i10mult(H_gy_Xr, xp_Xr) + 
        i10mult(H_gy_Yr, xp_Yr) +
        i10mult(H_gy_Zr, xp_Zr);
    int y_gz = -z_gz +
        // xp_Zb +
        i10mult(H_gz_Xr, xp_Xr) + 
        i10mult(H_gz_Yr, xp_Yr) +
        i10mult(H_gz_Zr, xp_Zr);

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
            Pp_xaa, Pp_yaa, Pp_zaa, 0, 0, 0, // no biases
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

    // acceleration (format: K_<state var>_<sensor var> )
    int K_xa_ax, K_xa_ay, K_xa_az; 
    int K_ya_ax, K_ya_ay, K_ya_az; 
    int K_za_ax, K_za_ay, K_za_az; 

    /** transpose of the H matrix looks like:
     * H_ax_xa, H_ay_xa, H_az_xa;
     * H_ay_xa, ...
     */
    // intermediate inverse of S matrix for acceleration
    // format given as: [[i1, i2, i3],[i2, i4, i5],[i3, i5, i6]]
    int ia1, ia2, ia3, ia4, ia5, ia6;
    invertSymmetric3x3(ia1, ia2, ia3, ia4, ia5, ia6,
            S_axx, S_axy, S_axz, S_ayy, S_ayz, S_azz);
    kalmanGain3x3x3(
            // outputs
            K_xa_ax, K_xa_ay, K_xa_az,
            K_ya_ax, K_ya_ay, K_ya_az,
            K_za_ax, K_za_ay, K_za_az,
            // three Pp's needed
            Pp_xaa, Pp_yaa, Pp_zaa,
            // transpose of H matrix
            H_ax_xa, H_ay_xa, H_az_xa,
            H_ax_ya, H_ay_ya, H_az_ya,
            H_ax_za, H_ay_za, H_az_za,
            // 6 unique elements of symmetrical inverse matrix
            ia1, ia2, ia3, ia4, ia5, ia6);

    // gyroscope
    // for differences between this and acceleration, since we now incorporate
    // biases, compare previous maxima matrix multiplication with:
    //  matrix([p1,0,0,0,0,0],[0,p2,0,0,0,0],[0,0,p3,0,0,0],[0,0,0,p4,0,0],[0,0,0,0,p5,0],[0,0,0,0,0,p6]) . matrix([h1,h2,h3],[h4,h5,h6],[h7,h8,h9],[1,0,0],[0,1,0],[0,0,1]) . matrix([i1,i2,i3],[i2,i4,i5],[i3,i5,i6]);
    int K_Xr_gx, K_Xr_gy, K_Xr_gz; 
    int K_Yr_gx, K_Yr_gy, K_Yr_gz; 
    int K_Zr_gx, K_Zr_gy, K_Zr_gz; 

    int ig1, ig2, ig3, ig4, ig5, ig6;
    invertSymmetric3x3(ig1, ig2, ig3, ig4, ig5, ig6,
            S_gxx, S_gxy, S_gxz, S_gyy, S_gyz, S_gzz);
    kalmanGain3x3x3(
            // outputs
            K_Xr_gx, K_Xr_gy, K_Xr_gz,
            K_Yr_gx, K_Yr_gy, K_Yr_gz,
            K_Zr_gx, K_Zr_gy, K_Zr_gz,
            // three Pp's needed
            Pp_Xrr, Pp_Yrr, Pp_Zrr,
            // transpose of H matrix
            H_gx_Xr, H_gy_Xr, H_gz_Xr,
            H_gx_Yr, H_gy_Yr, H_gz_Yr,
            H_gx_Zr, H_gy_Zr, H_gz_Zr,
            // 6 unique elements of symmetrical inverse matrix
            ig1, ig2, ig3, ig4, ig5, ig6);

    // biases from gyroscope values - easy, bottom half of previous
    // maxima calculation
    // int K_Xb_gx = i10mult(ig1, Pp_Xbb);
    // int K_Xb_gy = i10mult(ig2, Pp_Xbb);
    // int K_Xb_gz = i10mult(ig3, Pp_Xbb);
    // int K_Yb_gx = i10mult(ig2, Pp_Ybb);
    // int K_Yb_gy = i10mult(ig4, Pp_Ybb);
    // int K_Yb_gz = i10mult(ig5, Pp_Ybb);
    // int K_Zb_gx = i10mult(ig3, Pp_Zbb);
    // int K_Zb_gy = i10mult(ig5, Pp_Zbb);
    // int K_Zb_gz = i10mult(ig6, Pp_Zbb);

    // GPS and Orientation-pseudo-sensors

    // intermediates for multiplying by covariance
    int H_S_Pxx = ((1<<20) / S_Pxx );
    int H_S_Pyy = ((1<<20) / S_Pyy );
    int H_S_Pzz = ((1<<20) / S_Pzz );

    int H_S_Oxx = ((1<<20) / S_Oxx );
    int H_S_Oyy = ((1<<20) / S_Oyy );
    int H_S_Ozz = ((1<<20) / S_Ozz );






    int K_xp_Px = i10mult(Pp_xpp , H_S_Pxx);
    int K_yp_Py = i10mult(Pp_ypp , H_S_Pyy);
    int K_zp_Pz = i10mult(Pp_zpp , H_S_Pzz);
                                 ,        
    int K_Xa_Ox = i10mult(Pp_Xaa , H_S_Oxx);
    int K_Ya_Oy = i10mult(Pp_Yaa , H_S_Oyy);
    int K_Za_Oz = i10mult(Pp_Zaa , H_S_Ozz);

    int K_xv_Px = i10mult(Pp_xpv , H_S_Pxx);
    int K_yv_Py = i10mult(Pp_ypv , H_S_Pyy);
    int K_zv_Pz = i10mult(Pp_zpv , H_S_Pzz);

    // int K_Xa_Ox = i10mult(Pp_Xaa , H_S_Oxx);
    // int K_Ya_Oy = i10mult(Pp_Yaa , H_S_Oyy);
    // int K_Za_Oz = i10mult(Pp_Zaa , H_S_Ozz);


    // Step 6: UPDATED STATE ESTIMATE ==========================================
    // bit simpler: x_k|k = x_x|x-1 + K_k * y_k
    // acceleration gets updates from 3,
    // angular velocity get updates from 3,
    // bias gets updates from three (surprisingly. might need fixing.
    //  TODO: recheck why each bias depends on values from all axes,
    // position and angle each get one from each sensor
    
    // from accelerometer
    x_xa = xp_xa
        + i10mult(K_xa_ax , y_ax)
        + i10mult(K_xa_ay , y_ay)
        + i10mult(K_xa_az , y_az);
    x_ya = xp_ya           
        + i10mult(K_ya_ax , y_ax)
        + i10mult(K_ya_ay , y_ay)
        + i10mult(K_ya_az , y_az);
    x_za = xp_za           
        + i10mult(K_za_ax , y_ax)
        + i10mult(K_za_ay , y_ay)
        + i10mult(K_za_az , y_az);

    // gyroscope updates to both x_*r and x_*b
    x_Xr = xp_Xr
        + i10mult(K_Xr_gx , y_gx)
        + i10mult(K_Xr_gy , y_gy)
        + i10mult(K_Xr_gz , y_gz);
    x_Yr = xp_Yr           
        + i10mult(K_Yr_gx , y_gx)
        + i10mult(K_Yr_gy , y_gy)
        + i10mult(K_Yr_gz , y_gz);
    x_Zr = xp_Zr           
        + i10mult(K_Zr_gx , y_gx)
        + i10mult(K_Zr_gy , y_gy)
        + i10mult(K_Zr_gz , y_gz);

    // x_Xb = xp_Xb
    //     + i10mult(K_Xb_gx , y_gx)
    //     + i10mult(K_Xb_gy , y_gy)
    //     + i10mult(K_Xb_gz , y_gz);
    // x_Yb = xp_Yb           
    //     + i10mult(K_Yb_gx , y_gx)
    //     + i10mult(K_Yb_gy , y_gy)
    //     + i10mult(K_Yb_gz , y_gz);
    // x_Zb = xp_Zb           
    //     + i10mult(K_Zb_gx , y_gx)
    //     + i10mult(K_Zb_gy , y_gy)
    //     + i10mult(K_Zb_gz , y_gz);

    // GPS and Orientation's updates:
    x_xp = xp_xp + i10mult(K_xp_Px , y_Px);
    x_yp = xp_yp + i10mult(K_yp_Py , y_Py);
    x_zp = xp_zp + i10mult(K_zp_Pz , y_Pz);

    x_Xa = xp_Xa + i10mult(K_Xa_Ox , y_Ox);
    x_Ya = xp_Ya + i10mult(K_Ya_Oy , y_Oy);
    x_Za = xp_Za + i10mult(K_Za_Oz , y_Oz);
    
    // directly assign all variables not updated by the sensors
    x_xv = xp_xv + i10mult(K_xv_Px , y_Px);
    x_yv = xp_yv + i10mult(K_yv_Py , y_Px);
    x_zv = xp_zv + i10mult(K_zv_Pz , y_Px); 

    // Step 7: UPDATED ESTIMATE COVARIANCE =====================================
    // P_k|k = (<I> - K_k * H_k) * P_k|k-1



    P_xpp = i10mult(Pp_xpp, (1<<10) - K_xp_Px );
    P_ypp = i10mult(Pp_ypp, (1<<10) - K_yp_Py );
    P_zpp = i10mult(Pp_zpp, (1<<10) - K_zp_Pz );

    P_xvv = i10mult(Pp_xvv, (1<<10) - K_xv_Px );
    P_yvv = i10mult(Pp_yvv, (1<<10) - K_yv_Py );
    P_zvv = i10mult(Pp_zvv, (1<<10) - K_zv_Pz );

    P_xaa = i10mult(Pp_xaa, (1<<10) - (
                i10mult(K_xa_ax, H_ax_xa) +
                i10mult(K_xa_ay, H_ay_xa) +
                i10mult(K_xa_az, H_az_xa) ));

    P_yaa = i10mult(Pp_yaa, (1<<10) - (
                i10mult(K_ya_ax, H_ax_ya) +
                i10mult(K_ya_ay, H_ay_ya) +
                i10mult(K_ya_az, H_az_ya) ));

    P_zaa = i10mult(Pp_zaa, (1<<10) - (
                i10mult(K_za_ax, H_ax_za) +
                i10mult(K_za_ay, H_ay_za) +
                i10mult(K_za_az, H_az_za) ));

    // rotations
    P_Xaa = i10mult(Pp_Xaa, (1<<10) - K_Xa_Ox );
    P_Yaa = i10mult(Pp_Yaa, (1<<10) - K_Ya_Oy );
    P_Zaa = i10mult(Pp_Zaa, (1<<10) - K_Za_Oz );

    P_Xrr = i10mult(Pp_Xrr, (1<<10) - (
                i10mult(K_Xr_gx, H_gx_Xr) +
                i10mult(K_Xr_gy, H_gy_Xr) +
                i10mult(K_Xr_gz, H_gz_Xr) ));

    P_Yrr = i10mult(Pp_Yrr, (1<<10) - (
                i10mult(K_Yr_gx, H_gx_Yr) +
                i10mult(K_Yr_gy, H_gy_Yr) +
                i10mult(K_Yr_gz, H_gz_Yr) ));

    P_Zrr = i10mult(Pp_Zrr, (1<<10) - (
                i10mult(K_Zr_gx, H_gx_Zr) +
                i10mult(K_Zr_gy, H_gy_Zr) +
                i10mult(K_Zr_gz, H_gz_Zr) ));

    // P_Xbb = i10mult(Pp_Xbb, (1<<10) - K_Xb_gx);
    // P_Ybb = i10mult(Pp_Ybb, (1<<10) - K_Yb_gx);
    // P_Zbb = i10mult(Pp_Zbb, (1<<10) - K_Zb_gx);

    P_xpv = Pp_xpv;
    P_xpa = Pp_xpa;
    P_xva = Pp_xva;
    P_ypv = Pp_ypv;
    P_ypa = Pp_ypa;
    P_yva = Pp_yva;
    P_zpv = Pp_zpv;
    P_zpa = Pp_zpa;
    P_zva = Pp_zva;
    P_Xar = Pp_Xar;
    P_Yar = Pp_Yar;
    P_Zar = Pp_Zar;

} // END OF FILTER =============================================================






quadState_t getQuadState() {
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


void predictStateEstimateForPosition(
		int& xp_p, int& xp_v, int& xp_a,
		int& x_p, int& x_v, int& x_a,
		int& dT)
{
	// state transition matrix is:
	// 	xp	xv	xa
	// xp	1	t	t^2/2
	// xv	0	1	t
	// xa	0	0	1

	xp_p = x_p + i10mult(x_v, dT) + i10mult(i10mult(x_a, dT), dT) / 2;
	xp_v = x_v + i10mult(x_a, dT);
	xp_a = x_a;
}

void predictStateEstimateForRotation(
		int& xp_a, int& xp_r, int& xp_b,
		int& x_a, int& x_r, int& x_b,
		int& dT)
{
	// state transition matrix (rotations):
	// 	Xa	Xr	Xb
	// Xa	1	dT	0
	// Xr	0	1	0
	// Xb	0	0	1

	xp_a = x_a + i10mult(x_r, dT);
	xp_r = x_r;
	xp_b = x_b;
}

void predictStateCovarianceForPosition(
		int& op1, int& op2, int& op3, int& op4, int& op5, int& op6,
		int& p1, int& p2, int& p3, int& p4, int& p5, int& p6,
		int& dT)
{
// maxima: matrix([1,t,t^2/2],[0,1,t],[0,0,1]) . matrix([p1,p2,p3],[p2,p4,p5],[p3,p5,p6]) . matrix([1,0,0],[t,1,0],[t^2/2,t,1]);
// (then simplify)

	op6 = p6;

	int p6t = i10mult(p6, dT);
	op5 = p6t + p5;

	int p6t2 = i10mult(p6t, dT);
	int p5t = i10mult(p5, dT);
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

void predictStateCovarianceForRotation(
		int& op1, int& op2, int& op3, // int& op4,
		int& p1, int& p2, int& p3, // int& p4,
		int& dT)
{
	// [[1,t,0],[0,1,0],[0,0,1]] * [[p1,p2,0],[p2,p3,0],[0,0,p4]] * [[1,0,0],[t,1,0],[0,0,1]]
	// op4 = p4;
	op3 = p3;
	op2 = p2 + i10mult(p3, dT);
	op1 = p1 + i10mult(op2 + p2, dT);
}


void innovationCovariance(
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

    s1 = /* p4+ */ i10mult3(h3,h3,p3)+i10mult3(h2,h2,p2)+i10mult3(h1,h1,p1);
    s2 = /*     */ i10mult3(h3,h6,p3)+i10mult3(h2,h5,p2)+i10mult3(h1,h4,p1);
    s3 = /*     */ i10mult3(h3,h9,p3)+i10mult3(h2,h8,p2)+i10mult3(h1,h7,p1);
    s4 = /* p5+ */ i10mult3(h6,h6,p3)+i10mult3(h5,h5,p2)+i10mult3(h4,h4,p1);
    s5 = /*     */ i10mult3(h6,h9,p3)+i10mult3(h5,h8,p2)+i10mult3(h4,h7,p1);
    s6 = /* p6+ */ i10mult3(h9,h9,p3)+i10mult3(h8,h8,p2)+i10mult3(h7,h7,p1);
}

void kalmanGain3x3x3(
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

    k1 = i10mult( ( i10mult(h3,i3) + i10mult(h2,i2) + i10mult(h1,i1) ) , p1 );
    k2 = i10mult( ( i10mult(h3,i5) + i10mult(h2,i4) + i10mult(h1,i2) ) , p1 );
    k3 = i10mult( ( i10mult(h3,i6) + i10mult(h2,i5) + i10mult(h1,i3) ) , p1 );
    k4 = i10mult( ( i10mult(h6,i3) + i10mult(h5,i2) + i10mult(h4,i1) ) , p2 );
    k5 = i10mult( ( i10mult(h6,i5) + i10mult(h5,i4) + i10mult(h4,i2) ) , p2 );
    k6 = i10mult( ( i10mult(h6,i6) + i10mult(h5,i5) + i10mult(h4,i3) ) , p2 );
    k7 = i10mult( ( i10mult(h9,i3) + i10mult(h8,i2) + i10mult(h7,i1) ) , p3 );
    k8 = i10mult( ( i10mult(h9,i5) + i10mult(h8,i4) + i10mult(h7,i2) ) , p3 );
    k9 = i10mult( ( i10mult(h9,i6) + i10mult(h8,i5) + i10mult(h7,i3) ) , p3 );
}



void invert3x3(int& o_0_0, int& o_0_1, int& o_0_2,
        int& o_1_0, int& o_1_1, int& o_1_2,
        int& o_2_0, int& o_2_1, int& o_2_2,
        int a_0_0, int a_0_1, int a_0_2,
        int a_1_0, int a_1_1, int a_1_2,
        int a_2_0, int a_2_1, int a_2_2)
{
    int determinant =
        + i10mult(a_0_0 ,( i10mult(a_1_1 , a_2_2) - i10mult(a_2_1 , a_1_2 )))
        - i10mult(a_0_1 ,( i10mult(a_1_0 , a_2_2) - i10mult(a_1_2 , a_2_0 )))
        + i10mult(a_0_2 ,( i10mult(a_1_0 , a_2_1) - i10mult(a_1_1 , a_2_0 )));
    int invdet = (1<<20)/determinant;
    int r_0_0 = i10mult( (i10mult(a_1_1,a_2_2) - i10mult(a_2_1,a_1_2)),invdet);
    int r_1_0 = i10mult(-(i10mult(a_0_1,a_2_2) - i10mult(a_0_2,a_2_1)),invdet);
    int r_2_0 = i10mult( (i10mult(a_0_1,a_1_2) - i10mult(a_0_2,a_1_1)),invdet);
    int r_0_1 = i10mult(-(i10mult(a_1_0,a_2_2) - i10mult(a_1_2,a_2_0)),invdet);
    int r_1_1 = i10mult( (i10mult(a_0_0,a_2_2) - i10mult(a_0_2,a_2_0)),invdet);
    int r_2_1 = i10mult(-(i10mult(a_0_0,a_1_2) - i10mult(a_1_0,a_0_2)),invdet);
    int r_0_2 = i10mult( (i10mult(a_1_0,a_2_1) - i10mult(a_2_0,a_1_1)),invdet);
    int r_1_2 = i10mult(-(i10mult(a_0_0,a_2_1) - i10mult(a_2_0,a_0_1)),invdet);
    int r_2_2 = i10mult( (i10mult(a_0_0,a_1_1) - i10mult(a_1_0,a_0_1)),invdet);

	// transposed inverse calculated, store actual inverse
	o_0_0 = r_0_0; o_1_0 = r_0_1; o_2_0 = r_0_2;
	o_0_1 = r_1_0; o_1_1 = r_1_1; o_2_1 = r_1_2;
	o_0_2 = r_2_0; o_1_2 = r_2_1; o_2_2 = r_2_2;
}

void invertSymmetric3x3(int& o1, int& o2, int& o3, int& o4, int& o5, int& o6,
        int i1, int i2, int i3, int i4, int i5, int i6)
{
    // to check: wolfram alpha:
    //  invert([[s1,s2,s3],[s2,s4,s5],[s3,s5,s6]])
    int determinant =
        - i10mult3(i1,i4,i6) + i10mult3(i1,i5,i5)
        + i10mult3(i2,i2,i6) - 2 * i10mult3(i2,i3,i5)
        + i10mult3(i3,i3,i4);
    int invdet = (1<<20) / determinant;

    o1 = i10mult(i10mult(i5,i5) - i10mult(i4,i6), invdet);
    o2 = i10mult(i10mult(i2,i6) - i10mult(i3,i5), invdet);
    o3 = i10mult(i10mult(i3,i4) - i10mult(i2,i5), invdet);
    o4 = i10mult(i10mult(i3,i3) - i10mult(i1,i6), invdet);
    o5 = i10mult(i10mult(i1,i5) - i10mult(i2,i3), invdet);
    o6 = i10mult(i10mult(i2,i2) - i10mult(i1,i4), invdet);
}
#endif
