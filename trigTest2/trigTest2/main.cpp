#include "Trig.h"
#include <iostream>
using namespace std;

Trig trig;
int i;

int main()
{
	for(i=1;i<3;i++){
		cout << trig.atan2(46080,20480*i) << "\n";
	}
	cin >> i;
}
