#include <iostream>
#include <fstream>
#include "Kalman.h"
using namespace std;

int main()
{
	KalmanFilter Kalman;
	ifstream datFile;

	Kalman();
	datFile.open("bogusData.txt");
	int i, j, k, vals[9];
	for(i=0;i<60;i++){
		for(j=0;j<8,j++){
			datFile >> vals[j];
		}
			Kalman.assignSensorValues(
				vals[0], vals[1], vals[2],	// acceleration
				vals[3], vals[4], vals[5],	// gyroscope
				vals[6], vals[7], vals[8],	// Compass
				0, 0, 0);	// GPS
	}
}