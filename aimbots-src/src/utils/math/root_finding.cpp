#include <float.h>
#include <math.h>

#include <algorithm>
#include <chrono>
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
using namespace std::chrono;

#define DEBUG false

#define ACCEPTED_ERROR 1e-10                                     // how far the root can deviate from the x-axis
#define PRECISION_OF_DERIVATIVE (complex<double>)(1e-10, 1e-10)  // the precision used when finding slope using the def of a derivative
#define ALLOWED_ITERATIONS 50                                    // the number of allowed_iterations until divergency is assumed
#define UPPER_ACCEPTED_BOUND \
    (complex<double>)30  // the upper limit in seconds that a trajectory intersection will be looked for, we will not expect bullets to have 30
                         // seconds of airtime
#define POLY_ORDER 5

using std::cout, std::endl;

complex<double> co[] = {1, 2, 3, 4, 1};

complex<double> unit_func4(complex<double> *coeffs, complex<double> time) {
    return (
        coeffs[0] * (time) * (time) * (time) * (time) + coeffs[1] * (time) * (time) * (time) + coeffs[2] * (time) * (time) + coeffs[3] * time +
        coeffs[4]);
}

complex<double> unit_func(complex<double> time) { return unit_func4(co, time); }

complex<double> example_func(complex<double> time) {  // 4x^4 + -2x^3 + -4x^2 + x - 3
    return (time - (complex<double>)3) * (time + (complex<double>)2) * (time + (complex<double>)3) + (complex<double>)100;  //-3, -2, 3
    // return (4 * pow(time, 4)) - (2 * pow(time, 3)) - (4 * pow(time, 2)) + time - 3;
}

complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots) {
    complex<double> factor = 1;
    for (unsigned int i = 0; i < roots.size(); i++) {
        factor *= (input - roots.at(i));
    }
    return func(input) / factor;
}

complex<double> find_next_root(
    complex<double> (*func)(complex<double>),
    complex<double> estimate,
    vector<complex<double>> roots,
    bool &all_found) {  // takes as arg a function to evaluate roots for
    complex<double> x = estimate;
    int iterations = 0;
    complex<double> factor = 1;
    while (abs(real(modified_function(x, func, roots))) > ACCEPTED_ERROR && iterations < ALLOWED_ITERATIONS) {
        complex<double> slope =
            (modified_function(x + PRECISION_OF_DERIVATIVE, func, roots) - modified_function(x, func, roots)) / PRECISION_OF_DERIVATIVE;
        x = x - (modified_function(x, func, roots) / slope);
        if (DEBUG) cout << "function returned " << modified_function(x, func, roots) << " on iteration " << iterations << endl;
        iterations++;
    }
    if ((iterations >= ALLOWED_ITERATIONS - 1) || (real(x) > real(UPPER_ACCEPTED_BOUND)) || (real(x) < -real(UPPER_ACCEPTED_BOUND))) {
        all_found = true;
        return DBL_MAX;
    }
    if (DEBUG) cout << endl << "number of iterations: " << iterations << endl;
    return x;
}

void Swap(double *a, double *b) {
    double temp = *a;
    *a = *b;
    *b = temp;
}

void BubbleSort(vector<double> &array) {
    for (unsigned int i = 0; i < array.size(); i++) {
        for (unsigned int j = 0; j < array.size() - 1; j++) {
            if (array[j] > array[j + 1]) Swap(&array[j], &array[j + 1]);
        }
    }
}

double get_priority_root(vector<complex<double>> roots) {
    // return the lowest, positive, real root
    vector<double> reals;
    for (unsigned int i = 0; i < roots.size(); i++) {
        if (abs(imag(roots.at(i))) > ACCEPTED_ERROR || real(roots.at(i)) < 0) {
            reals.push_back(30);
        } else if (real(roots.at(i)) <= ACCEPTED_ERROR) {
            reals.push_back(real(roots.at(i)));
        }
    }
    BubbleSort(reals);
    if (DEBUG) cout << "reals size: " << reals.size() << endl;
    if (reals.size() == 0 || reals.at(0) >= 30) {
        return -1;
    }

    return reals.at(0);
}

double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate) {
    vector<complex<double>> roots;
    bool all_found = false;

    /*
    for(int i = 0; i < 5; i++){
        roots.push_back(find_next_root(func, estimate, roots, all_found));
        cout << roots.back() << " is a root and all_found is " << all_found << endl;
    }
    */

    while (all_found == false) {
        complex<double> root = find_next_root(func, estimate, roots, all_found);
        if (DEBUG) cout << "root found: " << root << endl << endl;
        if (all_found == true) {
            break;
        } else {
            roots.push_back(root);
        }
    }

    double priority = get_priority_root(roots);
    return priority;
    if (priority == -1) {
        perror("No valid trajectory");
        exit(1);
    } else {
        return priority;
    }

    return 1;
}

// int main() {
//     auto start = high_resolution_clock::now();

//     double root = deep_impact(&unit_func, (complex<double>)(1, 0));

//     auto stop = high_resolution_clock::now();

//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//     cout << "runtime in microseconds " << duration.count() << endl;

//     cout << "the final root is " << root << endl;
//     return 0;
// }

/*
cannot self determine the order of the polynomial
    could return an all found if too many iterations pass

ideally needs to find imaginary solutions
    WIP, the complex number system should converge on the solution though it does not seem to

needs to know to exit with error if there is no real positive solution
    exits when it takes too long to find a solution and returns an invalid solution
    if the only solutions are all invalid, the invalid solution is returned to be handled by the caller




deep_impact recieves an estimate and the function

deep impact calls root_finder and keeps a list of all found roots

root_finder iteratively finds roots of polynomials using newtons algorithm

root_finder is sped up by dividing out already found roots

root_finder returns a found root to deep_impact

deep_impact runs until the root_finder method takes too long to find a root

deep_impact calls get_priority_root to find the prefered output

get_priority_root filters out negative values then calls BubbleSort

BubbleSort sorts the list for get_priority_root

get_priority_root returns the lowest value, or an error if no valid time was found

deep_impact passes on that lowest value as its output



*/