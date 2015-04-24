#include <iostream>
#include <fstream>
#include "Kalman.h"
#include <iomanip>
using namespace std;

void outputStateToStream(ofstream & out, quadState_t & state) {
    out << state.xPosition << '\t' ;
    out << state.yPosition << '\t' ;
    out << state.zPosition << '\t' ;
    out << state.xVelocity << '\t' ;
    out << state.yVelocity << '\t' ;
    out << state.zVelocity << '\t' ;
    out << state.xAcceleration << '\t' ;
    out << state.yAcceleration << '\t' ;
    out << state.zAcceleration << '\t' ;
    out << state.xAngle << '\t' ;
    out << state.yAngle << '\t' ;
    out << state.zAngle << '\t' ;
    out << state.xRotation << '\t' ;
    out << state.yRotation << '\t' ;
    out << state.zRotation << endl;
}

int main()
{
	ifstream datFile;

    quadState_t results;
    quadState_t covariance;

	datFile.open("data");
    ofstream outFile;
    outFile.open("results.csv");
    ofstream debugFile;
    debugFile.open("debug.csv");

    KalmanFilter kalman(outFile, debugFile);
    kalman.initialize( 6.9158f, 9.9410f, 21.515f,
            -16.027f, 0.9157f, 0.6185f,
            -12504.f, -13316.f, -24942.f );

    outFile << "x_xp\tx_yp\tx_zp\tx_xv\tx_yv\tx_zv\tx_xa\tx_ya\tx_za\t";
    outFile << "x_Xa\tx_Ya\tx_Za\tx_Xr\tx_Yr\tx_Zr\n";

    int i, j, k, vals[12];
    while(!datFile.eof()) {
		for(j=0;j<12;j++){
			datFile >> vals[j];
		}
            results = kalman.getQuadState();
			kalman.assignSensorValues(
				vals[0], vals[1], vals[2],	// acceleration
				vals[3], vals[4], vals[5],	// gyroscope
				0, 100, // Compass facing north - initial Z angle = 0
				vals[8], vals[9], vals[10], static_cast<bool>(vals[11]));
            // GPS and use (y/n)
            kalman.predictAndUpdate();
            // covariance = kalman.getCovariance();

            outputStateToStream(outFile, results);
	}
}
