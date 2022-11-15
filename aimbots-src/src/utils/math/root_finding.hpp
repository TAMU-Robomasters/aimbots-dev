#pragma once

#include "utils/common_types.hpp"
#include <complex>
#include <vector>


#define DEBUG false  //sets print statements

#define ACCEPTED_ERROR (double)1e-10  //how far the root can deviate from the x-axis
#define PRECISION_OF_DERIVATIVE (double)1e-10  //the precision used when finding slope using the def of a derivative
#define ALLOWED_ITERATIONS 50  //the number of allowed_iterations until divergency is assumed
#define UPPER_ACCEPTED_BOUND 30  //the upper limit in seconds that a trajectory intersection will be looked for, we will not expect bullets to have 30 seconds of airtime

using namespace std;

double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate);

// missing decalrations (added below)?
// double get_priority_root(vector<complex<double>> roots);
// void BubbleSort(vector<double> &array);
// void Swap(double *a, double *b);
// complex<double> find_next_root(complex<double> (*func)(complex<double>), complex<double> estimate, vector<complex<double>> roots, bool &all_found);
// complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots);
// complex<double> example_func(complex<double> time);
// complex<double> unit_func(complex<double> time);
// complex<double> unit_func4(vector<int> coeffs, complex<double> time);
