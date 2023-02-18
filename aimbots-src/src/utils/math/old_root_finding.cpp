#include "root_finding.hpp"

using namespace std;
using namespace std::chrono;

// using std::cout, std::endl;

vector<double> global_coeffs;

int curr_unit_test = 0;
vector<vector<int>> testing_coeffs {
    {-9, -3, 4, -10, -1},
    {-6, 8, 8, -8, -6},
    {-5, -5, -9, -3, -9}, 
    {1, 5, -8, -3, 6},
    {1, -6, -8, 3, 2},
    {-8, -9, 6, 8, 5},
    {-3, -4, 1, 8, -1},
    {2, -3, 9, 5, 4},
    {-7, 1, -8, 3, 3},
    {-6, -9, 1, 3, -2}
};
vector<vector<complex<double>>> unit_outputs;

complex<double> unit_func4(vector<double> coeffs, complex<double> time){
    return (((complex<double>)coeffs.at(0))*(time)*(time)*(time)*(time) + ((complex<double>)coeffs.at(1)*(time)*(time)*(time)) + ((complex<double>)coeffs.at(2))*(time)*(time) + ((complex<double>)coeffs.at(3))*time + ((complex<double>)coeffs.at(4)));
}



complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots) {
    complex<double> factor = 1;
    for(unsigned int i = 0; i < roots.size(); i++){
        factor *= (input - roots.at(i));
    }
    return func(input) / factor;
}

complex<double> find_next_root(complex<double> (*func)(complex<double>), complex<double> estimate, vector<complex<double>> roots, bool &all_found) {  // takes as arg a function to evaluate roots for
    complex<double> x = estimate;
    int iterations = 0;
    //complex<double> factor = 1;
    while (abs(real(modified_function(x, func, roots))) > ACCEPTED_ERROR && iterations < ALLOWED_ITERATIONS) {
        complex<double> slope =
            (modified_function(x + PRECISION_OF_DERIVATIVE, func, roots) - modified_function(x, func, roots)) / PRECISION_OF_DERIVATIVE;
        x = x - (modified_function(x, func, roots) / slope);
        // if (DEBUG) cout << "function returned " << modified_function(x, func, roots) << " on iteration " << iterations << endl;
        iterations++;
    }
    if ((iterations >= ALLOWED_ITERATIONS - 1) || (real(x) > real(UPPER_ACCEPTED_BOUND)) || (real(x) < -real(UPPER_ACCEPTED_BOUND))) {
        all_found = true;
        return DBL_MAX;
    }
    // if (DEBUG) cout << endl << "number of iterations: " << iterations << endl;
    return x;
}

void Swap(double *a, double *b) {
    double temp = *a;
    *a = *b;
    *b = temp;
}

void BubbleSort(vector<double> &array) {
    for ( unsigned int i = 0; i < array.size(); i++)
    {
        for( unsigned int j = 0; j < array.size() - 1; j++)
        {   
            if (array[j] > array[j+1])
                Swap(&array[j], &array[j+1]);
        }
    }
}

double get_priority_root(vector<complex<double>> roots) {
    // return the lowest, positive, real root
    vector<double> reals;
    for (unsigned int i = 0; i < roots.size(); i++) {
        if (abs(imag(roots.at(i))) > ACCEPTED_ERROR || real(roots.at(i)) < 0) {
            reals.push_back(30);
        //} else if (real(roots.at(i)) <= ACCEPTED_ERROR) {
        } else{
            reals.push_back(real(roots.at(i)));
        }
    }
    // if(DEBUG){
    //     for(long long unsigned int i = 0; i < reals.size(); i++){
    //         cout << "reals at " << i << " = " << reals.at(i) << endl;
    //     }
    // }
    BubbleSort(reals);
    // if (DEBUG) cout << "reals size: " << reals.size() << endl;
    if (reals.size() == 0 || reals.at(0) >= 30) {
        return -1;
    }

    return reals.at(0);
}

complex<double> equation(complex<double> time){
    // cout << "global_coeffs size = " << global_coeffs.size() << endl;
    assert(global_coeffs.size() >= 5);
    return (((complex<double>)global_coeffs.at(0))*(time)*(time)*(time)*(time) + ((complex<double>)global_coeffs.at(1)*(time)*(time)*(time)) + ((complex<double>)global_coeffs.at(2))*(time)*(time) + ((complex<double>)global_coeffs.at(3))*time + ((complex<double>)global_coeffs.at(4)));
}


double deep_impact(complex<double> (*func)(complex<double>), complex<double> estimate) {
    vector<complex<double>> roots;
    bool all_found = false;
    //complex<double> estimate = 1;

    /*
    for(int i = 0; i < 5; i++){
        roots.push_back(find_next_root(func, estimate, roots, all_found));
        cout << roots.back() << " is a root and all_found is " << all_found << endl;
    }
    */

    while (all_found == false) {
        complex<double> root = find_next_root(func, estimate, roots, all_found);
        // if (DEBUG) cout << "root found: " << root << endl << endl;
        if (all_found == true) {
            break;
        } else {
            roots.push_back(root);
        }
        // if(DEBUG){
        //     for(size_t i = 0; i < roots.size(); i++){
        //         cout << roots.at(i) << " ";
        //     } cout << endl;
        // }
    }
    //unit_outputs.push_back(roots);

    // if(DEBUG){

    // for(long long unsigned int i = 0; i < roots.size(); i++){
    //     cout << "roots at " << i << " = " << roots.at(i) << endl;
    // }

    // }
    
    return get_priority_root(roots);
    
}


float find_root(vector<float> coeffs){
    for(size_t i = 0; i < coeffs.size(); i++){
        global_coeffs.push_back(coeffs.at(i));
    }

    return deep_impact(&equation, 1);
}

/*
int main(){

    vector<double> vec = {-8, -9, 6, 8, 5};
    cout << find_root(vec) << endl;

    return 1;
}
*/

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
    returns an all found boolean if too many iterations pass

ideally needs to find imaginary solutions
    the complex number system converges on both real and imaginary solutions

needs to know to exit with error if there is no real positive solution
    exits when it takes too long to find a solution and returns an invalid solution
    if the only solutions are all invalid, the invalid solution is returned to be handled by the caller
    note: the invalid solution is hardcoded as -1, because this represents a time it should be clear that is not valid


High level explainer:

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