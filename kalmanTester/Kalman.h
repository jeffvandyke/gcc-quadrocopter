#include <math.h>

class KalmanFilter {
	private:
		KalmanFilter();
	public:
		static void assignSensorValues(
				int, int, int,	// acceleration
				int, int, int,	// gyroscope
				int, int, int,	// Compass
				int, int, int);	// GPS

		static void predictAndUpdate();
	private:
		// line by line functions

		void predictStateEstimateForPositions(
				int&, int&, int&, int&, int&, int&, int&);
		void predictStateEstimateForRotations(
				int&, int&, int&, int&, int&, int&, int&);

		void predictStateCovarianceForPositions(
				int&, int&, int&, int&, int&, int&,
				int&, int&, int&, int&, int&, int&,
				int&);
		void predictStateCovarianceForRotations(
				int&, int&, int&, int&, int&, int&,
				int&, int&, int&, int&, int&, int&,
				int&);
		// Constant Matrixes

		static const int Q_xpp = 64, Q_xvv = 64, Q_xaa = 64;
		static const int Q_ypp = 64, Q_yvv = 64, Q_yaa = 64;
		static const int Q_zpp = 64, Q_zvv = 64, Q_zaa = 64;

		static const int Q_Xaa = 64, Q_Xrr = 64, Q_Xbb = 64;
		static const int Q_Yaa = 64, Q_Yrr = 64, Q_Ybb = 64;
		static const int Q_Zaa = 64, Q_Zrr = 64, Q_Zbb = 64;


		// Sensor error covariance (diagonals only)


		// state variables

		// position
		static int x_xp, x_xv, x_xa;
		static int x_yp, x_yv, x_ya;
		static int x_zp, x_zv, x_za;

		// orientation
		// a: angle, r: rotation (speed), b: bias
		static int x_Xa, x_Xr, x_Xb;
		static int x_Ya, x_Yr, x_Yb;
		static int x_Za, x_Zr, x_Zb;

		// P matrix
		static int P_xpp, P_xpv, P_xpa, P_xvv, P_xva, Paa;
		static int P_ypp, P_ypv, P_ypa, P_yvv, P_yva, Paa;
		static int P_zpp, P_zpv, P_zpa, P_zvv, P_zva, Paa;
		static int P_Xaa, P_Xar, P_Xab, P_Xrr, P_Xrb, abb;
		static int P_Yaa, P_Yar, P_Yab, P_Yrr, P_Yrb, abb;
		static int P_Zaa, P_Zar, P_Zab, P_Zrr, P_Zrb, abb;

		// sensor values
		// a: accelerometer
		// g: gyroscope
		// C: compass
		// P: GPS
		// O:	(computed orientation in degrees, used for correcting
		// 	orientation)

		static int z_ax, z_ay, z_az;

		static int z_gx, z_gy, z_gz;

		// * compass only used for orientation
		static int z_Px, z_Py, z_Pz;

		// *computed, useful values from accelerometer to correct angle
		static int z_Ox, z_Oy, z_Oz;

		static unsigned int previousTimeRan;
		static unsigned int nTimesRan;


		/* H mapping: to sensor values from state variables
		accelerometer x, y, z map to xa, ya, and za
		gyroscope
		*/

		// DYNAMIC MATRIX - updated on sensor update
		// mapping: H_<target sensor value>_<source state variable>
		static int H_ax_xa, H_ax_ya, H_ax_za;
		static int H_ay_xa, H_ay_ya, H_ay_za;
		static int H_az_xa, H_az_ya, H_az_za;

		// since their all one, probably can replace with assignments
		// in some places
		static const int H_gx_Xr = 1, H_gy_Yr = 1, H_gz_Zr = 1;
		static const int H_gx_Xr = 1, H_gy_Yr = 1, H_gz_Zr = 1;
		static const int H_Px_xp = 1, H_Px_yp = 1, H_Px_zp = 1;





};
