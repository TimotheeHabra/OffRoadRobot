#ifdef __cplusplus

#include "test_hybrid_code.h"
#include <iostream>

using namespace std;

void print_cpp()
{
	cout << "force value = " << 10 << endl;
	cout << "I used a C++ functions"<< endl;
}

int get_force()
{
	print_cpp();
	return 10;
}

#endif