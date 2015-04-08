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
	ifstream datFile;

    quadState_t results;
    quadState_t covariance;

	datFile.open("variance test.csv");
    ofstream outFile;
    outFile.open("results.csv");

    KalmanFilter kalman(outFile);
    kalman.initialize(-15, 2, 1);

	int i, j, k, vals[9];
	for(i=0;i<10000;i++){
		for(j=0;j<8;j++){
			datFile >> vals[j];
            cout << vals[j];
		}
			kalman.assignSensorValues(
				vals[0], vals[1], vals[2],	// acceleration
				vals[3], vals[4], vals[5],	// gyroscope
				vals[6], vals[7], // Compass
				0, 0, 0, i % 60 == 0);	// GPS
            kalman.predictAndUpdate();
            results = kalman.getQuadState();
            covariance = kalman.getCovariance();
            // separator
            outFile << " ,";
            outputStateToStream(outFile, results);
            outputStateToStream(outFile, covariance);
	}
}
