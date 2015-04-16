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

	datFile.open("variance test.csv");
    ofstream outFile;
    outFile.open("results.csv");
    ofstream debugFile;
    debugFile.open("debug.csv");

    KalmanFilter kalman(debugFile);
    kalman.initialize(-15.24991969, 1.94282043, 1.4240282686);

    outFile << "xPo\tyPo\tzPo\txVe\tyVe\tzVe\txAc\tyAc\tzAc\txAn\tyAn\tzAn\txRo\tyRo\tzRo\n";
	int i, j, k, vals[9];
	for(i=0;i<3110;i++){
		for(j=0;j<8;j++){
			datFile >> vals[j];
		}
			kalman.assignSensorValues(
				vals[0], vals[1], vals[2],	// acceleration
				vals[3], vals[4], vals[5],	// gyroscope
				vals[6], vals[7], // Compass
				0, 0, 0, (i + 2) % 60 == 0);	// GPS
            kalman.predictAndUpdate();
            results = kalman.getQuadState();
            covariance = kalman.getCovariance();

            outputStateToStream(outFile, results);
            debugFile << "xPo\tyPo\tzPo\txVe\tyVe\tzVe\txAc\tyAc\tzAc\txAn\tyAn\tzAn\txRo\tyRo\tzRo\n";
            outputStateToStream(debugFile, results);
	}
}
