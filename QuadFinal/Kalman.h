#ifndef KALMAN_H
#define KALMAN_H

#define SLOG 1
#define DEBUGK 0
#define S_BLUETOOTH 0

#include "math.h"
#if !(DEBUGK)
#include <Wire.h>
#include "I2C.h"
#endif


#if DEBUGK

#include <iostream>
#include <fstream>
#include <string>
#endif

struct quadState_t {
    float xPosition, yPosition, zPosition;
    float xVelocity, yVelocity, zVelocity;
    float xAcceleration, yAcceleration, zAcceleration;

    float xAngle, yAngle, zAngle;
    float xRotation, yRotation, zRotation;
};

using namespace std;

class KalmanFilter {
	public:
        KalmanFilter() {}
#if DEBUGK
		KalmanFilter(ofstream& resultsOut, ofstream& out);
        ofstream* rout;
        ofstream* fout;

        void log(float number) { *fout << number << '\t'; }
        void logr() { *fout << endl; }
        void log() { *fout << '\t'; }
        void log(string text) {*fout << text << '\t';}
        void rlog(float number) { *rout << number << '\t'; }
        void rlograw(string text) {*rout << text;}
        void logh(string text) {*fout << text << '\n';}
#endif
#if SLOG
        void slog(String header, float number){
#if S_BLUETOOTH
            Serial1.print(header);
            Serial1.print(",");
            Serial1.print(number);
            Serial1.print("\t");
#else
            Serial.print(header);
            Serial.print(",");
            Serial.print(number);
            Serial.print("\t");
#endif
        }
#endif

		void initialize(float biasAccX, float biasAccY, float biasAccZ,
                float biasGyroX, float biasGyroY, float biasGyroZ,
                int biasGpsX, int biasGpsY, int biasGpsZ);

		void assignSensorValues(
				int, int, int,	// acceleration
				int, int, int,	// gyroscope
				int, int, int,	// Compass
				int, int, int,	// GPS
				bool); // use gps value

		void predictAndUpdate();
        quadState_t getQuadState();
        quadState_t getCovariance();
	// private:
		// repeated functions over variaus dimensions

        void predictStateEstimateForPosition(
                float& xp_p, float& xp_v, float& xp_a,
                float& x_p, float& x_v, float& x_a,
                float& dT);
        void predictStateEstimateForRotation(
                float& xp_a, float& xp_r,
                float& x_a, float& x_r,
                float& dT);
        void predictStateCovarianceForPosition(
                float& op1, float& op2, float& op3, float& op4, float& op5, float& op6,
                float& p1, float& p2, float& p3, float& p4, float& p5, float& p6,
                float& dT);
        void predictStateCovarianceForRotation(
                float& op1, float& op2, float& op3, // float& op4,
                float& p1, float& p2, float& p3, // float& p4,
                float& dT);
        void innovationCovariance(
                float& s1, float& s2, float& s3, float& s4, float& s5, float& s6,
                float p1, float p2, float p3, // float p4, float p5, float p6,
                float h1, float h2, float h3,
                float h4, float h5, float h6,
                float h7, float h8, float h9);
        void kalmanGain3x3x3(
                float& k1, float& k2, float& k3,
                float& k4, float& k5, float& k6,
                float& k7, float& k8, float& k9,
                float p1, float p2, float p3,
                float h1, float h2, float h3,
                float h4, float h5, float h6,
                float h7, float h8, float h9,
                float i1, float i2, float i3, float i4, float i5, float i6);


        void invertSymmetric3x3(
                float& o1, float& o2, float& o3, float& o4, float& o5, float& o6,
                float i1, float i2, float i3, float i4, float i5, float i6);

        bool usingGPS;
        // biases:

        float bias_ax, bias_ay, bias_az;
        float bias_gx, bias_gy, bias_gz;
        int bias_Gx, bias_Gy, bias_Gz;

		// Constant Matrixes
		//
		// Process Noise - variance associated with each state variable
		// from a global point of view - the higher these values, the
		// less we should trust the estimate, and the more we should
		// trust the sensors.
		//
		// Position
		float Q_xpp, Q_xpv, Q_xpa, Q_xvv, Q_xva, Q_xaa;
		float Q_ypp, Q_ypv, Q_ypa, Q_yvv, Q_yva, Q_yaa;
		float Q_zpp, Q_zpv, Q_zpa, Q_zvv, Q_zva, Q_zaa;
		// Rotation
		float Q_Xaa, Q_Xar, Q_Xrr, Q_Xbb;
		float Q_Yaa, Q_Yar, Q_Yrr, Q_Ybb;
		float Q_Zaa, Q_Zar, Q_Zrr, Q_Zbb;

		// Sensor error covariance - Diagonal Matrix
		// (from lab)
		float R_axx, R_axy, R_axz, R_ayy, R_ayz, R_azz;
		float R_gxx, R_gxy, R_gxz, R_gyy, R_gyz, R_gzz;
		// 0.000349 r/s^2
		float R_Px, R_Py, R_Pz;
		float R_Ox, R_Oy, R_Oz;


		// state variables

		// position
		float x_xp, x_xv, x_xa;
		float x_yp, x_yv, x_ya;
		float x_zp, x_zv, x_za;

		// orientation
		// a: angle, r: rotation (speed), b: bias
		float x_Xa, x_Xr;
		float x_Ya, x_Yr;
		float x_Za, x_Zr;

		// P matrix
		float P_xpp, P_xpv, P_xpa, P_xvv, P_xva, P_xaa;
		float P_ypp, P_ypv, P_ypa, P_yvv, P_yva, P_yaa;
		float P_zpp, P_zpv, P_zpa, P_zvv, P_zva, P_zaa;
		float P_Xaa, P_Xar, P_Xrr;
		float P_Yaa, P_Yar, P_Yrr;
		float P_Zaa, P_Zar, P_Zrr;

		// sensor values
		// a: accelerometer
		// g: gyroscope
		// C: compass
		// P: GPS
		// O:	(computed orientation in degrees, used for correcting
		// 	orientation)
		// * compass only used for orientation
		// computed, useful values from accelerometer to correct angle

		float z_ax, z_ay, z_az;
		float z_gx, z_gy, z_gz;
		float z_Px, z_Py, z_Pz;
		float z_Ox, z_Oy, z_Oz;

		unsigned int previousTimeRan;
		unsigned int nTimesRan;


		/* H mapping: to sensor values from state variables
		accelerometer x, y, z map to xa, ya, and za
		gyroscope
		*/

		// H OBSERVATION MATRIX (dynamic) - updated on sensor update
		// mapping: H_<target sensor value>_<source state variable>
		float H_ax_xa, H_ax_ya, H_ax_za;
		float H_ay_xa, H_ay_ya, H_ay_za;
		float H_az_xa, H_az_ya, H_az_za;

		float H_gx_Xr, H_gx_Yr, H_gx_Zr;
		float H_gy_Xr, H_gy_Yr, H_gy_Zr;
		float H_gz_Xr, H_gz_Yr, H_gz_Zr;

		// biases just add
		// constants
////	static const int H_gx_Xb, H_gy_Yb, H_gz_Zb;
////	static const int H_Px_xp, H_Px_yp, H_Px_zp;
////	static const int H_Ox_Xa, H_Oy_Ya, H_Oz_Za;
};
#endif
