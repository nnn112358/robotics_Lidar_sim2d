
#include <math.h>
#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
using namespace std;


#include "geometory.h"


double str_double(string str){
	istringstream is;
	is.str(str);
	double x;
	is >> x;
	return x;
}
