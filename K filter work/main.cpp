#include <iostream>
#include <fstream>
#include "Kalman.h"
using namespace std;

void outputStateToStream(ofstream & out, quadState_t & state) {
    out << state.xPosition << ',' ;
    out << state.yPosition << ',' ;
    out << state.zPosition << ',' ;
    out << state.xVelocity << ',' ;
    out << state.yVelocity << ',' ;
    out << state.zVelocity << ',' ;
    out << state.xAcceleration << ',' ;
    out << state.yAcceleration << ',' ;
    out << state.zAcceleration << ',' ;
    out << state.xAngle << ',' ;
    out << state.yAngle << ',' ;
    out << state.zAngle << ',' ;
    out << state.xRotation << ',' ;
    out << state.yRotation << ',' ;
    out << state.zRotation << endl;
}

int main()
{
	KalmanFilter kalman;
	ifstream datFile;
    quadState_t results;

	datFile.open("variance test.csv");
    ofstream outFile;
    outFile.open("results.csv");

	int i, j, k, vals[9];
	for(i=0;i<60;i++){
		for(j=0;j<8;j++){
			datFile >> vals[j];
		}
			kalman.assignSensorValues(
				vals[0], vals[1], vals[2],	// acceleration
				vals[3], vals[4], vals[5],	// gyroscope
				vals[6], vals[7], // Compass
				0, 0, 0, false);	// GPS
            kalman.predictAndUpdate();
            results = kalman.getQuadState();
            outputStateToStream(outFile, results);
	}
}
