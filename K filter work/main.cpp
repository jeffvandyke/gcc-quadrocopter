#include <iostream>
#include "kalman.h"

using namespace std;

int main() {
    KalmanFilter kf;
    kf.initialize();
	kf.predictAndUpdate();
	return 0;
}
