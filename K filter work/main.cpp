#include <iostream>
#include "kalman.h"

using namespace std;

int main() {
    KalmanFilter kf;
    kf.initialize(-15, 2, 1);
    for(int i = 0; i < 700; i++) {
        kf.assignSensorValues(...);
    	kf.predictAndUpdate();
    }
	return 0;
}
