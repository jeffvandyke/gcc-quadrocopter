#ifndef KALMAN_H
#define KALMAN_H
#include "Trig.h"
#include <iostream>
#include <fstream>

struct quadState_t {
    int xPosition, yPosition, zPosition;
    int xVelocity, yVelocity, zVelocity;
    int xAcceleration, yAcceleration, zAcceleration;

    int xAngle, yAngle, zAngle;
    int xRotation, yRotation, zRotation;
};

// struct quadStateCovariance_t {
//     int xPosition, yPosition, zPosition;
//     int xVelocity, yVelocity, zVelocity;
//     int xAcceleration, yAcceleration, zAcceleration;
//
//     int xAngle, yAngle, zAngle;
//     int xRotation, yRotation, zRotation;
// };


class KalmanFilter {
	public:
		KalmanFilter(ofstream& out);
        ofstream* fout;

        void log(int number) { *fout << number << ','; }
        void logr() { *fout << endl; }

		void initialize(int, int, int);

		void assignSensorValues(
				int, int, int,	// acceleration
				int, int, int,	// gyroscope
				int, int, 	// Compass
				int, int, int,	// GPS
				// int, int, int,  // GPS velocity
				bool); // use gps value

		void predictAndUpdate();
        quadState_t getQuadState();
        quadState_t getCovariance();
	private:
		// repeated functions over variaus dimensions

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
                int& op1, int& op2, int& op3, // int& op4,
                int& p1, int& p2, int& p3, // int& p4,
                int& dT);
        void innovationCovariance(
                int& s1, int& s2, int& s3, int& s4, int& s5, int& s6,
                int p1, int p2, int p3, // int p4, int p5, int p6,
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


        void invertSymmetric3x3(
                int& o1, int& o2, int& o3, int& o4, int& o5, int& o6,
                int i1, int i2, int i3, int i4, int i5, int i6);

        bool usingGPS;
		// Constant Matrixes
		//
		// Process Noise - variance associated with each state variable
		// from a global point of view - the higher these values, the
		// less we should trust the estimate, and the more we should
		// trust the sensors.
		//
		// Position
		int Q_xpp, Q_xpv, Q_xpa, Q_xvv, Q_xva, Q_xaa;
		int Q_ypp, Q_ypv, Q_ypa, Q_yvv, Q_yva, Q_yaa;
		int Q_zpp, Q_zpv, Q_zpa, Q_zvv, Q_zva, Q_zaa;
		// Rotation
		int Q_Xaa, Q_Xar, Q_Xrr, Q_Xbb;
		int Q_Yaa, Q_Yar, Q_Yrr, Q_Ybb;
		int Q_Zaa, Q_Zar, Q_Zrr, Q_Zbb;

		// Sensor error covariance - Diagonal Matrix
		// (from lab)
		int R_ax, R_ay, R_az;
		int R_gx, R_gy, R_gz;
		// 0.000349 r/s^2
		int R_Px, R_Py, R_Pz;
		int R_Ox, R_Oy, R_Oz;


		// state variables

		// position
		int x_xp, x_xv, x_xa;
		int x_yp, x_yv, x_ya;
		int x_zp, x_zv, x_za;

		// orientation
		// a: angle, r: rotation (speed), b: bias
		int x_Xa, x_Xr, x_Xb;
		int x_Ya, x_Yr, x_Yb;
		int x_Za, x_Zr, x_Zb;

		// P matrix
		int P_xpp, P_xpv, P_xpa, P_xvv, P_xva, P_xaa;
		int P_ypp, P_ypv, P_ypa, P_yvv, P_yva, P_yaa;
		int P_zpp, P_zpv, P_zpa, P_zvv, P_zva, P_zaa;
		int P_Xaa, P_Xar, P_Xrr/*, P_Xbb*/;
		int P_Yaa, P_Yar, P_Yrr/*, P_Ybb*/;
		int P_Zaa, P_Zar, P_Zrr/*, P_Zbb*/;

		// sensor values
		// a: accelerometer
		// g: gyroscope
		// C: compass
		// P: GPS
		// O:	(computed orientation in degrees, used for correcting
		// 	orientation)
		// * compass only used for orientation
		// computed, useful values from accelerometer to correct angle

		int z_ax, z_ay, z_az;
		int z_gx, z_gy, z_gz;
		int z_Px, z_Py, z_Pz;
		int z_Ox, z_Oy, z_Oz;

		unsigned int previousTimeRan;
		unsigned int nTimesRan;


		/* H mapping: to sensor values from state variables
		accelerometer x, y, z map to xa, ya, and za
		gyroscope
		*/

		// H OBSERVATION MATRIX (dynamic) - updated on sensor update
		// mapping: H_<target sensor value>_<source state variable>
		int H_ax_xa, H_ax_ya, H_ax_za;
		int H_ay_xa, H_ay_ya, H_ay_za;
		int H_az_xa, H_az_ya, H_az_za;

		int H_gx_Xr, H_gx_Yr, H_gx_Zr;
		int H_gy_Xr, H_gy_Yr, H_gy_Zr;
		int H_gz_Xr, H_gz_Yr, H_gz_Zr;

		// biases just add
		// constants
////	static const int H_gx_Xb, H_gy_Yb, H_gz_Zb;
////	static const int H_Px_xp, H_Px_yp, H_Px_zp;
////	static const int H_Ox_Xa, H_Oy_Ya, H_Oz_Za;
};
#endif
