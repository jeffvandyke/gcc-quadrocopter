#ifndef KALMAN_CPP
#define KALMAN_CPP
#include "Kalman.h"



const float aProcessNoise = 1;
const float gProcessNoise = 6.f;

inline float i10m(float a, float b) {
    return ((a)*(b));
}

inline float i10ml(float a, float b) {
    return a * b;
}

inline float i10m3(float a, float b, float c) {
    return a * b * c;
}
using namespace std;

// 8.98 * 1024, just the average of the magnitude
const float gravity = 8.9639096;







// Intermediate functions - used at several points in the kalman filter's
// predict and update process

KalmanFilter::KalmanFilter() {
}

#if DEBUGK
KalmanFilter::KalmanFilter(ofstream& out) {
    fout = &out;
}
#endif


void KalmanFilter::initialize(float xBias, float yBias, float zBias) {

    x_Xb = xBias; x_Yb = yBias; x_Zb = zBias;

    x_xp = 0.f; x_xv = 0.f; x_xa = 0.f;
    x_yp = 0.f; x_yv = 0.f; x_ya = 0.f;
    x_zp = 0.f; x_zv = 0.f; x_za = 0.f;

    x_Xa = 0; x_Xr = 0;
    x_Ya = 0; x_Yr = 0;
    x_Za = 0; x_Zr = 0;

    P_xpp = 0.0; P_xpv = 0.0; P_xpa = 0.0;
    P_xvv = 0.0; P_xva = 0.0; P_xaa = 0.0;
    P_ypp = 0.0; P_ypv = 0.0; P_ypa = 0.0;
    P_yvv = 0.0; P_yva = 0.0; P_yaa = 0.0;
    P_zpp = 0.0; P_zpv = 0.0; P_zpa = 0.0;
    P_zvv = 0.0; P_zva = 0.0; P_zaa = 0.0;

    P_Xaa = 0.05; P_Xar = 0.05; P_Xrr = 0.05;
    P_Yaa = 0.05; P_Yar = 0.05; P_Yrr = 0.05;
    // z rotation is the only real uncertainty
    P_Zaa = 9.0; P_Zar = 0.0; P_Zrr = 0.0;


    // Sensor error covariance - Diagonal Matrix
    // (experimentation)
    R_ax = R_ay = R_az = 0.04;
    // 0.000349 (r/s^2)^2 = 1.1468 (°/s^2)^2
    R_gx = R_gy = R_gz = 1.1468 - .5; // (compensate for process noice)
    R_Px = R_Py = 1; R_Pz = 1;
    // large enough to promote steady state correctness without going overboard
    R_Ox = R_Oy = R_Oz = 10;
}

void KalmanFilter::assignSensorValues(
        int ax, int ay, int az,
        int gx, int gy, int gz,
        int Cx, int Cy,  // cz not used
        int Px, int Py, int Pz,
        bool useGPS)
{


    logh("===================================================================");

    usingGPS = useGPS;

    // adjust with scaling factor of 0.038307
    z_ax = (static_cast<float>(ax) * 0.038307);
    z_ay = (static_cast<float>(ay) * 0.038307);
    z_az = (static_cast<float>(az) * 0.038307);
    // adjust gyroscope factor of 0.069565 °/sec
    z_gx = (static_cast<float>(gx - x_Xb) * 0.069565 );
    z_gy = (static_cast<float>(gy - x_Yb) * 0.069565 );
    z_gz = (static_cast<float>(gz - x_Zb) * 0.069565 );


    if (usingGPS)
    {
        log("using gps");
        z_Px = Px * 1.f;
        z_Py = Py * 1.f;
        z_Pz = Pz * 1.f;
    } else {
        log("not using gps");
    }


    float cx = cos(x_Xa * 3.14159 / 180);
    float cy = cos(x_Ya * 3.14159 / 180);
    float cz = cos(x_Za * 3.14159 / 180);
    float sx = sin(x_Xa * 3.14159 / 180);
    float sy = sin(x_Ya * 3.14159 / 180);
    float sz = sin(x_Za * 3.14159 / 180);

    float dropX = (float)(atan2(-z_ay, -z_az )) * 180 / 3.14159;
    float dropY = (float)(atan2( z_ax, -z_az )) * 180 / 3.14159;
    z_Ox = i10m( dropX , cz ) + i10m ( dropY , sz );
    z_Oy = i10m( dropY , cz ) + i10m ( dropX ,-sz );
    z_Oz = (float)(atan2(Cy, Cx)) * 180 / 3.14159;

    log("test for sin...\n");
    log("dropx"); log(dropX); log("dropy"); log(dropY);
    logr();

    H_gx_Xr = H_ax_xa = i10m( cy , cz );
    H_gy_Yr = H_ay_ya = i10m( cz , cx );
    H_gz_Zr = H_az_za = i10m( cx , cy );

    H_gx_Yr = H_ax_ya = i10m( sz , cx );
    H_gy_Xr = H_ay_xa = i10m(-sz , cy );

    H_gx_Zr = H_ax_za = i10m(-sy , cx );
    H_gy_Zr = H_ay_za = i10m( sx , cy );

    H_gz_Xr = H_az_xa = i10m( sy , cz );
    H_gz_Yr = H_az_ya = i10m(-sx , cz );




#if DEBUGK

    logh("z_ax\tz_ay\tz_az\tz_gx\tz_gy\tz_gz\t"
            "z_Px\tz_Py\tz_Pz\tz_Ox\tz_Oy\tz_Oz");
    log(z_ax); log(z_ay); log(z_az);
    log(z_gx); log(z_gy); log(z_gz);
    log(z_Px); log(z_Py); log(z_Pz);
    log(z_Ox); log(z_Oy); log(z_Oz);
    logr();
#endif

}


// this function sets the new state variables from the previous ones and the
// updated sensor values.
void KalmanFilter::predictAndUpdate()
{
    nTimesRan++;
    unsigned int currentTime = 0; // TODO: get current time
    float dT = (float)(currentTime - previousTimeRan);

    dT = 1.f / 100;

    // Process noise Q matrix

    // Position
    float dT_2 = i10m(dT, dT);
    float dT_3 = i10m(dT_2, dT);
    float dT_4 = i10m(dT_3, dT);

    Q_xpp = Q_ypp = Q_zpp = i10m(dT_4 / 4 , aProcessNoise);
    Q_xpv = Q_ypv = Q_zpv = i10m(dT_3 / 2 , aProcessNoise);
    Q_xpa = Q_ypa = Q_zpa = i10m(dT_2 / 4 , aProcessNoise);
    Q_xvv = Q_yvv = Q_zvv = i10m(dT_2 , aProcessNoise);
    Q_xva = Q_yva = Q_zva = i10m(dT , aProcessNoise);
    Q_xaa = Q_yaa = Q_zaa = i10m(1 , aProcessNoise);


    // Rotation
    Q_Xaa = Q_Yaa = Q_Zaa = i10m(dT_4 / 4 , gProcessNoise);
    Q_Xar = Q_Yar = Q_Zar = i10m(dT_3 / 2 , gProcessNoise);
    Q_Xrr = Q_Yrr = Q_Zrr = i10m(dT_2 , gProcessNoise);

    // predict stage

    // Step 1: PREDICTED (A PRIORI) STATE ESTIMATE =============================

    float xp_xp, xp_xv, xp_xa, xp_yp, xp_yv, xp_ya, xp_zp, xp_zv, xp_za;
    predictStateEstimateForPosition(
            xp_xp, xp_xv, xp_xa, x_xp, x_xv, x_xa, dT);
    predictStateEstimateForPosition(
            xp_yp, xp_yv, xp_ya, x_yp, x_yv, x_ya, dT);
    predictStateEstimateForPosition(
            xp_zp, xp_zv, xp_za, x_zp, x_zv, x_za, dT);

    float xp_Xa, xp_Xr, xp_Xb, xp_Ya, xp_Yr, xp_Yb, xp_Za, xp_Zr, xp_Zb;
    predictStateEstimateForRotation(
            xp_Xa, xp_Xr, xp_Xb, x_Xa, x_Xr, x_Xb, dT);
    predictStateEstimateForRotation(
            xp_Ya, xp_Yr, xp_Yb, x_Ya, x_Yr, x_Yb, dT);
    predictStateEstimateForRotation(
            xp_Za, xp_Zr, xp_Zb, x_Za, x_Zr, x_Zb, dT);


    // Step 2: PREDICTED (A PRIORI) ESTIMATE COVARIANCE ========================
    // P_k = F_k * P_(k-1) * F_k'T + Q_k

    float Pp_xpp, Pp_xpv, Pp_xpa, Pp_xvv, Pp_xva, Pp_xaa;
    float Pp_ypp, Pp_ypv, Pp_ypa, Pp_yvv, Pp_yva, Pp_yaa;
    float Pp_zpp, Pp_zpv, Pp_zpa, Pp_zvv, Pp_zva, Pp_zaa;
    float Pp_Xaa, Pp_Xar, Pp_Xrr; //  Pp_Xbb;
    float Pp_Yaa, Pp_Yar, Pp_Yrr; //  Pp_Ybb;
    float Pp_Zaa, Pp_Zar, Pp_Zrr; //  Pp_Zbb;

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


#if DEBUGK
    log("~ H_a ~"); log("_xa"); log("_ya"); log("_za");
    logh("\tPp_xpp\tPp_xpv\tPp_xpa\tPp_xvv\tPp_xva\tPp_xaa");
    log("    xa_"); log(H_ax_xa); log(H_ax_ya); log(H_ax_za); log();
    log(Pp_xpp); log(Pp_xpv); log(Pp_xpa);
    log(Pp_xvv); log(Pp_xva); log(Pp_xaa); logr();
    log("    ya_"); log(H_ay_xa); log(H_ay_ya); log(H_ay_za); log();
    log(Pp_ypp); log(Pp_ypv); log(Pp_ypa);
    log(Pp_yvv); log(Pp_yva); log(Pp_yaa); logr();
    log("    za_"); log(H_az_xa); log(H_az_ya); log(H_az_za); log();
    log(Pp_zpp); log(Pp_zpv); log(Pp_zpa);
    log(Pp_zvv); log(Pp_zva); log(Pp_zaa); logr();
#endif










    // UPDATE ==================================================================
    // Step 3: INNOVATION OR MEASUREMENT RESIDUAL ==============================

    float y_ax = -(i10m(H_ax_xa, xp_xa) +
            i10m(H_ax_ya, xp_ya) +
            i10m(H_ax_za, xp_za - gravity) )+
        z_ax;
    float y_ay = -(i10m(H_ay_xa, xp_xa) +
            i10m(H_ay_ya, xp_ya) +
            i10m(H_ay_za, xp_za - gravity) )+
        z_ay;
    float y_az = -(i10m(H_az_xa, xp_xa) +
            i10m(H_az_ya, xp_ya) +
            i10m(H_az_za, xp_za - gravity) )+
        z_az;

    // rotations
    float y_gx = z_gx - (
            // xp_Xb +
            i10m(H_gx_Xr, xp_Xr) +
            i10m(H_gx_Yr, xp_Yr) +
            i10m(H_gx_Zr, xp_Zr));
    float y_gy = z_gy - (
            // xp_Xb +
            i10m(H_gy_Xr, xp_Xr) +
            i10m(H_gy_Yr, xp_Yr) +
            i10m(H_gy_Zr, xp_Zr));
    float y_gz = z_gz - (
            // xp_Zb +
            i10m(H_gz_Xr, xp_Xr) +
            i10m(H_gz_Yr, xp_Yr) +
            i10m(H_gz_Zr, xp_Zr));

    float y_Px = -1;
    float y_Py = -1;
    float y_Pz = -1;
    if (usingGPS) {
        y_Px = z_Px - xp_xp;
        y_Py = z_Py - xp_yp;
        y_Pz = z_Pz - xp_zp;
    }

    float y_Ox = z_Ox - xp_Xa;
    float y_Oy = z_Oy - xp_Ya;
    float y_Oz = z_Oz - xp_Za;


#if DEBUGK
    logh("y_ax\ty_ay\ty_az\ty_gx\ty_gy\ty_gz\t"
            "y_Px\ty_Py\ty_Pz\ty_Ox\ty_Oy\ty_Oz");
    log(y_ax); log(y_ay); log(y_az);
    log(y_gx); log(y_gy); log(y_gz);
    log(y_Px); log(y_Py); log(y_Pz);
    log(y_Ox); log(y_Oy); log(y_Oz);
    logr();
#endif

    // Step 4: INNOVATION COVARIANCE ===========================================
    //  step: S_k = H_k * P_k|k-1 * H_k' + R_k
    //
    //  need to figure out for the sensors, how much can we trust them
    //  (given as covariance)
    //  (maybe covarance between the gyroscope and accelerometer variables
    //  is necessary)
    //  - also, R_k is added

    // accelerometer
    float S_axx, S_axy, S_axz, S_ayy, S_ayz, S_azz;
    innovationCovariance(S_axx, S_axy, S_axz,
            S_ayy, S_ayz, S_azz,
            Pp_xaa, Pp_yaa, Pp_zaa, // 0, 0, 0, // no biases
            H_ax_xa, H_ax_ya, H_ax_za,
            H_ay_xa, H_ay_ya, H_ay_za,
            H_az_xa, H_az_ya, H_az_za);

    S_axx += R_ax; S_ayy += R_ay; S_azz += R_az;

    // gyroscope
    float S_gxx, S_gxy, S_gxz, S_gyy, S_gyz, S_gzz;
    innovationCovariance(S_gxx, S_gxy, S_gxz,
            S_gyy, S_gyz, S_gzz,
            Pp_Xrr, Pp_Yrr, Pp_Zrr, // Pp_Xbb, Pp_Ybb, Pp_Zbb,
            H_gx_Xr, H_gx_Yr, H_gx_Zr,
            H_gy_Xr, H_gy_Yr, H_gy_Zr,
            H_gz_Xr, H_gz_Yr, H_gz_Zr);

    S_gxx += R_gx; S_gyy += R_gy; S_gzz += R_gz;



    // GPS
    float S_Pxx = Pp_xpp + R_Px;
    float S_Pyy = Pp_ypp + R_Py;
    float S_Pzz = Pp_zpp + R_Pz;

    // Orientation (pseudo sensor)
    float S_Oxx = Pp_Xaa + R_Ox;
    float S_Oyy = Pp_Yaa + R_Oy;
    float S_Ozz = Pp_Zaa + R_Oz;

#if DEBUGK
    logh("S_axx\t S_axy\t S_axz\t S_ayy\t S_ayz\t S_azz\t\t"
            "S_Pxx\tS_Pyy\tS_Pzz");
    log(S_axx);
    log(S_axy);
    log(S_axz);
    log(S_ayy);
    log(S_ayz);
    log(S_azz);
    log("- a - P -\t");
    log(S_Pxx);
    log(S_Pyy);
    log(S_Pzz);
    logr();

    log(S_gxx);
    log(S_gxy);
    log(S_gxz);
    log(S_gyy);
    log(S_gyz);
    log(S_gzz);
    log("- g - O -\t");
    log(S_Oxx);
    log(S_Oyy);
    log(S_Ozz);
    logr();
#endif

    // Step 5: KALMAN GAIN =====================================================
    // computed from:
    // K = P_k|k-1 * H_k'*S_k^-1
    // see step 4.wxm for maxima analysis

    // format: K_<state variable>_<sensor variable>
    // intermediate inverse of S matrix for acceleration
    // format given as: [[i1, i2, i3],[i2, i4, i5],[i3, i5, i6]]
    float Si_axx, Si_axy, Si_axz, Si_ayy, Si_ayz, Si_azz;
    invertSymmetric3x3(Si_axx, Si_axy, Si_axz, Si_ayy, Si_ayz, Si_azz,
            S_axx, S_axy, S_axz, S_ayy, S_ayz, S_azz);

    // gyroscope
    // for differences between this and acceleration, since we now incorporate
    // biases, compare previous maxima matrix multiplication with:
    // step 5-7 gyro.wxm

    float Si_gxx, Si_gxy, Si_gxz, Si_gyy, Si_gyz, Si_gzz;
    invertSymmetric3x3(Si_gxx, Si_gxy, Si_gxz, Si_gyy, Si_gyz, Si_gzz,
            S_gxx, S_gxy, S_gxz, S_gyy, S_gyz, S_gzz);

    // GPS and Orientation-pseudo-sensors

    // intermediates for multiplying by covariance
    float Si_Pxx = ((1.f) / S_Pxx );
    float Si_Pyy = ((1.f) / S_Pyy );
    float Si_Pzz = ((1.f) / S_Pzz );

    // intermediates
    float Ht_SiP1 = H_az_xa * Si_axz + H_ay_xa * Si_axy + H_ax_xa * Si_axx ;
    float Ht_SiP2 = H_az_xa * Si_ayz + H_ay_xa * Si_ayy + H_ax_xa * Si_axy ;
    float Ht_SiP3 = H_az_xa * Si_azz + H_ay_xa * Si_ayz + H_ax_xa * Si_axz ;

    float Ht_SiP5 = H_az_ya * Si_ayz + H_ay_ya * Si_ayy + H_ax_ya * Si_axy ;
    float Ht_SiP4 = H_az_ya * Si_axz + H_ay_ya * Si_axy + H_ax_ya * Si_axx ;
    float Ht_SiP6 = H_az_ya * Si_azz + H_ay_ya * Si_ayz + H_ax_ya * Si_axz ;

    float Ht_SiP7 = H_az_za * Si_axz + H_ay_za * Si_axy + H_ax_za * Si_axx ;
    float Ht_SiP8 = H_az_za * Si_ayz + H_ay_za * Si_ayy + H_ax_za * Si_axy ;
    float Ht_SiP9 = H_az_za * Si_azz + H_ay_za * Si_ayz + H_ax_za * Si_axz ;

    // actual kalman gains
    float K_xp_ax = (Pp_xpa * (Ht_SiP1));
    float K_xp_ay = (Pp_xpa * (Ht_SiP2));
    float K_xp_az = (Pp_xpa * (Ht_SiP3));
    float K_yp_ax = (Pp_ypa * (Ht_SiP4));
    float K_yp_ay = (Pp_ypa * (Ht_SiP5));
    float K_yp_az = (Pp_ypa * (Ht_SiP6));
    float K_zp_ax = (Pp_zpa * (Ht_SiP7));
    float K_zp_ay = (Pp_zpa * (Ht_SiP8));
    float K_zp_az = (Pp_zpa * (Ht_SiP9));

    float K_xv_ax = (Pp_xva * (Ht_SiP1));
    float K_xv_ay = (Pp_xva * (Ht_SiP2));
    float K_xv_az = (Pp_xva * (Ht_SiP3));
    float K_yv_ax = (Pp_yva * (Ht_SiP4));
    float K_yv_ay = (Pp_yva * (Ht_SiP5));
    float K_yv_az = (Pp_yva * (Ht_SiP6));
    float K_zv_ax = (Pp_zva * (Ht_SiP7));
    float K_zv_ay = (Pp_zva * (Ht_SiP8));
    float K_zv_az = (Pp_zva * (Ht_SiP9));

    float K_xa_ax = (Pp_xaa * (Ht_SiP1));
    float K_xa_ay = (Pp_xaa * (Ht_SiP2));
    float K_xa_az = (Pp_xaa * (Ht_SiP3));
    float K_ya_ax = (Pp_yaa * (Ht_SiP4));
    float K_ya_ay = (Pp_yaa * (Ht_SiP5));
    float K_ya_az = (Pp_yaa * (Ht_SiP6));
    float K_za_ax = (Pp_zaa * (Ht_SiP7));
    float K_za_ay = (Pp_zaa * (Ht_SiP8));
    float K_za_az = (Pp_zaa * (Ht_SiP9));

    // position
    float K_xp_Px; float K_yp_Py; float K_zp_Pz;
    float K_xv_Px; float K_yv_Py; float K_zv_Pz;
    float K_xa_Px; float K_ya_Py; float K_za_Pz;

    if (usingGPS) {
        K_xp_Px = i10m( Pp_xpp , Si_Pxx );
        K_yp_Py = i10m( Pp_ypp , Si_Pyy );
        K_zp_Pz = i10m( Pp_zpp , Si_Pzz );
        K_xv_Px = i10m( Pp_xpv , Si_Pxx );
        K_yv_Py = i10m( Pp_ypv , Si_Pyy );
        K_zv_Pz = i10m( Pp_zpv , Si_Pzz );
        K_xa_Px = i10m( Pp_xpa , Si_Pxx );
        K_ya_Py = i10m( Pp_ypa , Si_Pyy );
        K_za_Pz = i10m( Pp_zpa , Si_Pzz );
    } else {
        K_xp_Px = 0; K_yp_Py = 0; K_zp_Pz = 0;
        K_xv_Px = 0; K_yv_Py = 0; K_zv_Pz = 0;
        K_xa_Px = 0; K_ya_Py = 0; K_za_Pz = 0;
    }


    // Orientation

    // intermediates for orientation correcting
    float Si_Oxx = (1.f / S_Oxx );
    float Si_Oyy = (1.f / S_Oyy );
    float Si_Ozz = (1.f / S_Ozz );


    // intermediates from gyroscope, UNSCALED
    float Ht_SiO1 = H_gz_Xr * Si_gxz + H_gy_Xr * Si_gxy + H_gx_Xr * Si_gxx;
    float Ht_SiO2 = H_gz_Xr * Si_gyz + H_gy_Xr * Si_gyy + H_gx_Xr * Si_gxy;
    float Ht_SiO3 = H_gz_Xr * Si_gzz + H_gy_Xr * Si_gyz + H_gx_Xr * Si_gxz;

    float Ht_SiO5 = H_gz_Yr * Si_gyz + H_gy_Yr * Si_gyy + H_gx_Yr * Si_gxy;
    float Ht_SiO4 = H_gz_Yr * Si_gxz + H_gy_Yr * Si_gxy + H_gx_Yr * Si_gxx;
    float Ht_SiO6 = H_gz_Yr * Si_gzz + H_gy_Yr * Si_gyz + H_gx_Yr * Si_gxz;

    float Ht_SiO7 = H_gz_Zr * Si_gxz + H_gy_Zr * Si_gxy + H_gx_Zr * Si_gxx;
    float Ht_SiO8 = H_gz_Zr * Si_gyz + H_gy_Zr * Si_gyy + H_gx_Zr * Si_gxy;
    float Ht_SiO9 = H_gz_Zr * Si_gzz + H_gy_Zr * Si_gyz + H_gx_Zr * Si_gxz;


    float K_Xa_gx = i10m( Pp_Xar , Ht_SiO1 ) ;
    float K_Xa_gy = i10m( Pp_Xar , Ht_SiO2 ) ;
    float K_Xa_gz = i10m( Pp_Xar , Ht_SiO3 ) ;

    float K_Ya_gx = i10m( Pp_Yar , Ht_SiO5 ) ;
    float K_Ya_gy = i10m( Pp_Yar , Ht_SiO4 ) ;
    float K_Ya_gz = i10m( Pp_Yar , Ht_SiO6 ) ;

    float K_Za_gx = i10m( Pp_Zar , Ht_SiO7 ) ;
    float K_Za_gy = i10m( Pp_Zar , Ht_SiO8 ) ;
    float K_Za_gz = i10m( Pp_Zar , Ht_SiO9 ) ;

    float K_Xr_gx = i10m( Pp_Xrr , Ht_SiO1 ) ;
    float K_Xr_gy = i10m( Pp_Xrr , Ht_SiO2 ) ;
    float K_Xr_gz = i10m( Pp_Xrr , Ht_SiO3 ) ;

    float K_Yr_gx = i10m( Pp_Yrr , Ht_SiO5 ) ;
    float K_Yr_gy = i10m( Pp_Yrr , Ht_SiO4 ) ;
    float K_Yr_gz = i10m( Pp_Yrr , Ht_SiO6 ) ;

    float K_Zr_gx = i10m( Pp_Zrr , Ht_SiO7 ) ;
    float K_Zr_gy = i10m( Pp_Zrr , Ht_SiO8 ) ;
    float K_Zr_gz = i10m( Pp_Zrr , Ht_SiO9 ) ;

    float K_Xa_Ox = (Pp_Xaa*Si_Oxx);
    float K_Ya_Oy = (Pp_Yaa*Si_Oyy);
    float K_Za_Oz = (Pp_Zaa*Si_Ozz);
    float K_Xr_Ox = (Pp_Xar*Si_Oxx);
    float K_Yr_Oy = (Pp_Yar*Si_Oyy);
    float K_Zr_Oz = (Pp_Zar*Si_Ozz);



    // Step 6: UPDATED STATE ESTIMATE ==========================================
    // bit simpler: x_k|k = x_x|x-1 + K_k * y_k
    // acceleration gets updates from 3,
    // angular velocity get updates from 3,
    // bias gets updates from three (surprisingly. might need fixing.
    //  TODO: recheck why each bias depends on values from all axes,
    // position and angle each get one from each sensor

    // from accelerometer
    x_xp = xp_xp +
        (K_xp_ax * y_ax + K_xp_ay * y_ay + K_xp_az * y_az + K_xp_Px * y_Px);
    x_yp = xp_yp +
        (K_yp_ax * y_ax + K_yp_ay * y_ay + K_yp_az * y_az + K_yp_Py * y_Py);
    x_zp = xp_zp +
        (K_zp_ax * y_ax + K_zp_ay * y_ay + K_zp_az * y_az + K_zp_Pz * y_Pz);

    x_xv = xp_xv +
        (K_xv_ax * y_ax + K_xv_ay * y_ay + K_xv_az * y_az + K_xv_Px * y_Px);
    x_yv = xp_yv +
        (K_yv_ax * y_ax + K_yv_ay * y_ay + K_yv_az * y_az + K_yv_Py * y_Py);
    x_zv = xp_zv +
        (K_zv_ax * y_ax + K_zv_ay * y_ay + K_zv_az * y_az + K_zv_Pz * y_Pz);

    x_xa = xp_xa +
        (K_xa_ax * y_ax + K_xa_ay * y_ay + K_xa_az * y_az + K_xa_Px * y_Px);
    x_ya = xp_ya +
        (K_ya_ax * y_ax + K_ya_ay * y_ay + K_ya_az * y_az + K_ya_Py * y_Py);
    x_za = xp_za +
        (K_za_ax * y_ax + K_za_ay * y_ay + K_za_az * y_az + K_za_Pz * y_Pz);


    // gyroscope updates to both x_*r
    x_Xa = xp_Xa +
        (K_Xa_gx * y_gx + K_Xa_gy * y_gy + K_Xa_gz * y_gz + K_Xa_Ox * y_Ox);
    x_Ya = xp_Ya +
        (K_Ya_gx * y_gx + K_Ya_gy * y_gy + K_Ya_gz * y_gz + K_Ya_Oy * y_Oy);
    x_Za = xp_Za +
        (K_Za_gx * y_gx + K_Za_gy * y_gy + K_Za_gz * y_gz + K_Za_Oz * y_Oz);

    x_Xr = xp_Xr +
        (K_Xr_gx * y_gx + K_Xr_gy * y_gy + K_Xr_gz * y_gz + K_Xr_Ox * y_Ox);
    x_Yr = xp_Yr +
        (K_Yr_gx * y_gx + K_Yr_gy * y_gy + K_Yr_gz * y_gz + K_Yr_Oy * y_Oy);
    x_Zr = xp_Zr +
        (K_Zr_gx * y_gx + K_Zr_gy * y_gy + K_Zr_gz * y_gz + K_Zr_Oz * y_Oz);



    // Step 7: UPDATED ESTIMATE COVARIANCE =====================================
    // P_k|k = (<I> - K_k * H_k) * P_k|k-1
    // see "step 5.wxm" last line for analysis

    // intermediates for optimizing
    float Pi_1_Kx = 1 - K_xp_Px;
    float Pi_1_Ky = 1 - K_yp_Py;
    float Pi_1_Kz = 1 - K_zp_Pz;

    float Pi_mess_xa =
        -H_az_xa * K_xp_az - H_ay_xa * K_xp_ay - H_ax_xa * K_xp_ax;
    float Pi_mess_ya =
        -H_az_ya * K_yp_az - H_ay_ya * K_yp_ay - H_ax_ya * K_yp_ax;
    float Pi_mess_za =
        -H_az_za * K_zp_az - H_ay_za * K_zp_ay - H_ax_za * K_zp_ax;

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
    float Pi_mess_xv =
        -H_az_xa * K_xv_az - H_ay_xa * K_xv_ay - H_ax_xa * K_xv_ax;
    float Pi_mess_yv =
        -H_az_ya * K_yv_az - H_ay_ya * K_yv_ay - H_ax_ya * K_yv_ax;
    float Pi_mess_zv =
        -H_az_za * K_zv_az - H_ay_za * K_zv_ay - H_ax_za * K_zv_ax;

    P_xvv = Pp_xvv - i10m( K_xv_Px , Pp_xpv ) + i10m( Pi_mess_xv , Pp_xva );
    P_xva = Pp_xva - i10m( K_xv_Px , Pp_xpa ) + i10m( Pi_mess_xv , Pp_xaa );

    P_yvv = Pp_yvv - i10m( K_yv_Py , Pp_ypv ) + i10m( Pi_mess_yv , Pp_yva );
    P_yva = Pp_yva - i10m( K_yv_Py , Pp_ypa ) + i10m( Pi_mess_yv , Pp_yaa );

    P_zvv = Pp_zvv - i10m( K_zv_Pz , Pp_zpv ) + i10m( Pi_mess_zv , Pp_zva );
    P_zva = Pp_zva - i10m( K_zv_Pz , Pp_zpa ) + i10m( Pi_mess_zv , Pp_zaa );


    P_xaa = i10m( (i10m( -H_az_xa , K_xa_az ) - i10m( H_ay_xa , K_xa_ay ) - i10m( H_ax_xa , K_xa_ax ) + 1 ) , Pp_xaa ) - i10m( K_xa_Px , Pp_xpa );
    P_yaa = i10m( (i10m( -H_az_ya , K_ya_az ) - i10m( H_ay_ya , K_ya_ay ) - i10m( H_ax_ya , K_ya_ax ) + 1 ) , Pp_yaa ) - i10m( K_ya_Py , Pp_ypa );
    P_zaa = i10m( (i10m( -H_az_za , K_za_az ) - i10m( H_ay_za , K_za_ay ) - i10m( H_ax_za , K_za_ax ) + 1 ) , Pp_zaa ) - i10m( K_za_Pz , Pp_zpa );

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


    logh("P_xpp\t P_xpv\t P_xpa\t P_xvv\t P_xva\t P_xaa\t \t P_Xaa\t P_Xar\t P_Xrr");

    log(P_xpp); log(P_xpv); log(P_xpa); log(P_xvv); log(P_xva); log(P_xaa);
    log("- X -"); log(P_Xaa); log(P_Xar); log(P_Xrr);
    logr();

    log(P_ypp); log(P_ypv); log(P_ypa); log(P_yvv); log(P_yva); log(P_yaa);
    log("- Y -"); log(P_Yaa); log(P_Yar); log(P_Yrr);
    logr();

    log(P_zpp); log(P_zpv); log(P_zpa); log(P_zvv); log(P_zva); log(P_zaa);
    log("- Z -"); log(P_Zaa); log(P_Zar); log(P_Zrr);
    logr();



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
        float& xp_p, float& xp_v, float& xp_a,
        float& x_p, float& x_v, float& x_a,
        float& dT)
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
        float& xp_a, float& xp_r, float& xp_b,
        float& x_a, float& x_r, float& x_b,
        float& dT)
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
        float& op1, float& op2, float& op3, float& op4, float& op5, float& op6,
        float& p1, float& p2, float& p3, float& p4, float& p5, float& p6,
        float& dT)
{
    // maxima: matrix([1,t,t^2/2],[0,1,t],[0,0,1]) . matrix([p1,p2,p3],[p2,p4,p5],[p3,p5,p6]) . matrix([1,0,0],[t,1,0],[t^2/2,t,1]);
    // (then simplify)

    op6 = p6;

    float p6t = i10m(p6, dT);
    op5 = p6t + p5;

    float p6t2 = i10m(p6t, dT);
    float p5t = i10m(p5, dT);
    op4 = p6t2 + 2 * p5t + p4;

    float p6t2d2 = (p6t2 / 2);
    op3 = p6t2d2 + p5t + p3;

    float p6t3d2 = i10m(p6t2d2, dT);
    float p5t2 = i10m(p5t,dT);
    float p3t = i10m(p3, dT);
    float p4t = i10m(p4, dT);
    op2 = p6t3d2 + p5t2 * 3 / 2 + p3t + p4t + p2;

    float p6t4d4 = i10m(p6t3d2, dT) / 2;
    op1 = p6t4d4 + i10m(p5t2 + p3t + p4t + 2 * p2, dT) + p1;
}

void KalmanFilter::predictStateCovarianceForRotation(
        float& op1, float& op2, float& op3, // float& op4,
        float& p1, float& p2, float& p3, // float& p4,
        float& dT)
{
    // [[1,t,0],[0,1,0],[0,0,1]] * [[p1,p2,0],[p2,p3,0],[0,0,p4]] * [[1,0,0],[t,1,0],[0,0,1]]
    // op4 = p4;
    op3 = p3;
    op2 = p2 + i10m(p3, dT);
    op1 = p1 + i10m(op2 + p2, dT);
}


void KalmanFilter::innovationCovariance(
        float& s1, float& s2, float& s3, float& s4, float& s5, float& s6,
        float p1, float p2, float p3,//  float p4, float p5, float p6,
        float h1, float h2, float h3,
        float h4, float h5, float h6,
        float h7, float h8, float h9)
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

    s1 = i10m3(h3,h3,p3)+i10m3(h2,h2,p2)+i10m3(h1,h1,p1);
    s2 = i10m3(h3,h6,p3)+i10m3(h2,h5,p2)+i10m3(h1,h4,p1);
    s3 = i10m3(h3,h9,p3)+i10m3(h2,h8,p2)+i10m3(h1,h7,p1);
    s4 = i10m3(h6,h6,p3)+i10m3(h5,h5,p2)+i10m3(h4,h4,p1);
    s5 = i10m3(h6,h9,p3)+i10m3(h5,h8,p2)+i10m3(h4,h7,p1);
    s6 = i10m3(h9,h9,p3)+i10m3(h8,h8,p2)+i10m3(h7,h7,p1);
}

void KalmanFilter::kalmanGain3x3x3(
        float& k1, float& k2, float& k3,
        float& k4, float& k5, float& k6,
        float& k7, float& k8, float& k9,
        float p1, float p2, float p3,
        float h1, float h2, float h3,
        float h4, float h5, float h6,
        float h7, float h8, float h9,
        float i1, float i2, float i3, float i4, float i5, float i6)
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
        float& o1, float& o2, float& o3, float& o4, float& o5, float& o6,
        float i1, float i2, float i3, float i4, float i5, float i6)
{
    // to check: wolfram alpha:
    //  invert([[s1,s2,s3],[s2,s4,s5],[s3,s5,s6]])
    float determinant =
        +i1*(i4*i6-i5*i5)
        -i2*(i2*i6-i5*i3)
        +i3*(i2*i5-i4*i3);

    float invdet = (1.f / determinant );

    // log(i1); log(i2); log(i3); log(i4); log(i5); log(i6);

    // log(determinant);
    // log(invdet);
    // logr();

    // bool scaleInv = invdet > (1 << 16);
    // if (scaleInv) { invdet = invdet >> 10; }
    invdet = -invdet;

    o1 = (( (i5*i5) - (i4*i6) ) * invdet);
    o2 = (( (i2*i6) - (i3*i5) ) * invdet);
    o3 = (( (i3*i4) - (i2*i5) ) * invdet);
    o4 = (( (i3*i3) - (i1*i6) ) * invdet);
    o5 = (( (i1*i5) - (i2*i3) ) * invdet);
    o6 = (( (i2*i2) - (i1*i4) ) * invdet);

    // if (scaleInv) {
    //     o1 = o1 >> 10; o2 = o2 >> 10; o3 = o3 >> 10;
    //     o4 = o4 >> 10; o5 = o5 >> 10; o6 = o6 >> 10;
    // } else {
    //     o1 = o1 >> 20; o2 = o2 >> 20; o3 = o3 >> 20;
    //     o4 = o4 >> 20; o5 = o5 >> 20; o6 = o6 >> 20;
    // }

    // log(o1); log(o2); log(o3); log(o4); log(o5); log(o6); logr();

}
#endif
