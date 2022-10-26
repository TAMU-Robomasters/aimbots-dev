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

#define DEBUG false  //sets print statements

#define ACCEPTED_ERROR 1e-10  //how far the root can deviate from the x-axis
#define PRECISION_OF_DERIVATIVE 1e-10  //the precision used when finding slope using the def of a derivative
#define ALLOWED_ITERATIONS 50  //the number of allowed_iterations until divergency is assumed
#define UPPER_ACCEPTED_BOUND 30  //the upper limit in seconds that a trajectory intersection will be looked for, we will not expect bullets to have 30 seconds of airtime

using std::cout, std::endl;

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

complex<double> unit_func4(vector<int> coeffs, complex<double> time){
    return (((complex<double>)coeffs.at(0))*(time)*(time)*(time)*(time) + ((complex<double>)coeffs.at(1)*(time)*(time)*(time)) + ((complex<double>)coeffs.at(2))*(time)*(time) + ((complex<double>)coeffs.at(3))*time + ((complex<double>)coeffs.at(4)));
}

complex<double> unit_func(complex<double> time){
    return unit_func4(testing_coeffs.at(curr_unit_test), time);
    //return 0;
    //return (((complex<double>)coeffs.at(0))*(time)*(time)*(time)*(time) + ((complex<double>)coeffs.at(1)*(time)*(time)*(time)) + ((complex<double>)coeffs.at(2))*(time)*(time) + ((complex<double>)coeffs.at(3))*time + ((complex<double>)coeffs.at(4)));
}

complex<double> example_func(complex<double> time){ // 4x^4 + -2x^3 + -4x^2 + x - 3
    return (time - (complex<double>)3)*(time + (complex<double>)2)*(time + (complex<double>)3) + (complex<double>)100; //-3, -2, 3
}

complex<double> modified_function(complex<double> input, complex<double> (*func)(complex<double>), vector<complex<double>> roots) {
    complex<double> factor = 1;
    for (unsigned int i = 0; i < roots.size(); i++) {
        factor *= (input - roots.at(i));
    }
    return func(input) / factor;
}


complex<double> find_next_root(complex<double> (*func)(complex<double>), vector<complex<double>> roots, bool &all_found){ //takes as arg a function to evaluate roots for
    complex<double> x (rand(), rand());
    int iterations = 0;
    complex<double> precision (PRECISION_OF_DERIVATIVE, PRECISION_OF_DERIVATIVE);

    if(DEBUG) cout << "entered find_next_root" << endl;
    if(DEBUG) cout << abs((double)real(modified_function(x, func, roots))) << endl;

    while(abs((double)real(modified_function(x, func, roots))) > (double)ACCEPTED_ERROR && iterations < ALLOWED_ITERATIONS){ 

        complex<double> slope = (modified_function(x + precision, func, roots) - modified_function(x, func, roots)) / precision;
        x = x - (modified_function(x, func, roots) / slope);

        if(DEBUG) cout  << "function returned " << modified_function(x, func, roots) << " on iteration " << iterations << endl; 

        iterations++;
    }
    if((iterations >= ALLOWED_ITERATIONS - 1) || (real(x) > UPPER_ACCEPTED_BOUND) || (real(x) < -UPPER_ACCEPTED_BOUND)){
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

double get_priority_root(vector<complex<double>> roots){ //return the lowest, positive, real root
    vector<double> reals;
    for(size_t i = 0; i < roots.size(); i++){
        if((abs((double)imag(roots.at(i))) > ((double)ACCEPTED_ERROR * 2)) || (real(roots.at(i)) < 0)){
            reals.push_back(30);
        } else{
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

double deep_impact(complex<double> (*func)(complex<double>)) {
    vector<complex<double>> roots;
    bool all_found = false;

    while(all_found == false){
        complex<double> root = find_next_root(func, roots, all_found);
        if(DEBUG) cout << "root found: " << root << endl << endl;
        if(all_found == true){
            break;
        } else {
            roots.push_back(root);
        }
        if(DEBUG){
            for(size_t i = 0; i < roots.size(); i++){
                cout << roots.at(i) << " ";
            } cout << endl;
        }
    }
    unit_outputs.push_back(roots);
    
    return get_priority_root(roots);
    
}

// int main() {
//     auto start = high_resolution_clock::now();


/*
int main(){
    double r;
    auto start = high_resolution_clock::now();

    for(int i = 0; i < 10; i++){
        auto start1 = high_resolution_clock::now();
        r = deep_impact(&unit_func);
        auto stop1 = high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(stop1 - start1);
        cout << "runtime in microseconds " << duration1.count() << " on run " << i << endl;
        //cout << deep_impact(&unit_func) << " was the found root on test " << curr_unit_test << endl;
        curr_unit_test++;
    }

    auto stop = high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "runtime in microseconds " << duration.count() << endl;
    cout << r << endl;
    
    for(int i = 0; i < unit_outputs.size(); i++){
        cout << i << endl;
        for(int j = 0; j < unit_outputs.at(i).size(); j++){
            cout << unit_outputs.at(i).at(j) << " ";
        } cout << endl << endl;
    }
    
    //if(DEBUG) cout << "the final root is " << root << endl;
    return 0;
}
*/

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