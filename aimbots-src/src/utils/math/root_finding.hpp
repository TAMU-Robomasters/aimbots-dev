#pragma once

#include "utils/common_types.hpp"
#include "src/subsystems/gimbal/gimbal.hpp"
//#include "gimbal.hpp"
//#include "utils/common_types.hpp"
#include <float.h>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include <cassert>

using namespace src::Gimbal;

#define DEBUG false  //sets print statements

#define ACCEPTED_ERROR 1e-10  //how far the root can deviate from the x-axis
#define PRECISION_OF_DERIVATIVE 1e-10  //the precision used when finding slope using the def of a derivative
#define ALLOWED_ITERATIONS 50  //the number of allowed_iterations until divergency is assumed
#define UPPER_ACCEPTED_BOUND 30  //the upper limit in seconds that a trajectory intersection will be looked for, we will not expect bullets to have 30 seconds of airtime

using namespace std;

double find_root(vector<double> coeffs);
double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate);
complex<double> unit_func4(vector<double> coeffs, complex<double> time);
complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots);
complex<double> find_next_root(complex<double> (*func)(complex<double>), complex<double> estimate, vector<complex<double>> roots, bool &all_found);
void Swap(double *a, double *b);
void BubbleSort(vector<double> &array);
double get_priority_root(vector<complex<double>> roots);
