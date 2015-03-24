#ifndef KALMAN_H
#define KALMAN_H
#include "Trig.h"
#include <math.h>

struct quadState_t {
    int xPosition, yPosition, zPosition;
    int xVelocity, yVelocity, zVelocity;
    int xAcceleration, yAcceleration, zAcceleration;

    int xAngle, yAngle, zAngle;
    int xRotation, yRotation, zRotation;
}


class KalmanFilter {
	public:
		KalmanFilter();

		void initialize();

		void assignSensorValues(
				int, int, int,	// acceleration
				int, int, int,	// gyroscope
				int, int, 	// Compass
				int, int, int,	// GPS
				// int, int, int,  // GPS velocity
				bool); // use gps value

		void predictAndUpdate();
        quadState_t getQuadState();
	private:
		// repeated functions over variaus dimensions

		bool usingGPS;
		// Constant Matrixes
		//
		// Process Noise - variance associated with each state variable
		// from a global point of view - the higher these values, the 
		// less we should trust the estimate, and the more we should
		// trust the sensors.
		//
		// Position
		int Q_xpp, Q_xvv, Q_xaa;
		int Q_ypp, Q_yvv, Q_yaa;
		int Q_zpp, Q_zvv, Q_zaa;
		// Rotation
		int Q_Xaa, Q_Xrr, Q_Xbb;
		int Q_Yaa, Q_Yrr, Q_Ybb;
		int Q_Zaa, Q_Zrr, Q_Zbb;

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
		int P_Xaa, P_Xar, P_Xrr, P_Xbb;
		int P_Yaa, P_Yar, P_Yrr, P_Ybb;
		int P_Zaa, P_Zar, P_Zrr, P_Zbb;

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
