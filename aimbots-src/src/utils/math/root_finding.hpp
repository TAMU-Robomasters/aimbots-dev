#pragma once

#include "utils/common_types.hpp"
#include <complex>
#include <vector>

double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate);

// missing decalrations (added below)?
double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate);
double get_priority_root(vector<complex<double>> roots);
void BubbleSort(vector<double> &array);
void Swap(double *a, double *b);
complex<double> find_next_root(complex<double> (*func)(complex<double>), complex<double> estimate, vector<complex<double>> roots, bool &all_found);
complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots);
complex<double> example_func(complex<double> time);
complex<double> unit_func(complex<double> time);
complex<double> unit_func4(vector<int> coeffs, complex<double> time);
