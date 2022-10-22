#include <iostream>
#include <iomanip>
#include <math.h>
#include <float.h>
#include <algorithm>
#include <chrono>

#include <vector>

using namespace std;
using namespace std::chrono;

#define DEBUG true

#define ACCEPTED_ERROR 1e-10  //how far the root can deviate from the x-axis
#define PRECISION_OF_DERIVATIVE 1e-10  //the precision used when finding slope using the def of a derivative
#define ALLOWED_ITERATIONS 30  //the number of allowed_iterations until divergency is assumed
#define UPPER_ACCEPTED_BOUND 30  //the upper limit in seconds that a trajectory intersection will be looked for, we will not expect bullets to have 30 seconds of airtime
#define POLY_ORDER 5

using std::cout, std::endl, std::sort;

double example_func(double time){ // 4x^4 + -2x^3 + -4x^2 + x - 3
    return (time - 3)*(time + 2)*(time + 3) + 9; //-3, -2, 3
    //return (4 * pow(time, 4)) - (2 * pow(time, 3)) - (4 * pow(time, 2)) + time - 3;
}


double modified_function(double input, double (*func)(double), vector<double> roots){
    double factor = 1;
    for(int i = 0; i < roots.size(); i++){
        factor *= (input - roots.at(i));
    }
    return func(input) / factor;
}


double find_next_root(double (*func)(double), double estimate, vector<double> roots, bool &all_found){ //takes as arg a function to evaluate roots for
    double x = estimate;
    int iterations = 0;
    double factor = 1;
    while(abs(modified_function(x, func, roots)) > ACCEPTED_ERROR && iterations < ALLOWED_ITERATIONS){ 
        double slope = (modified_function(x + PRECISION_OF_DERIVATIVE, func, roots) - modified_function(x, func, roots)) / PRECISION_OF_DERIVATIVE;
        x = x - (modified_function(x, func, roots) / slope);
        if(DEBUG) cout  << "function returned " << modified_function(x, func, roots) << " on iteration " << iterations << endl; 
        iterations++;
    }
    if((iterations >= ALLOWED_ITERATIONS - 1) || (x > 1024) || (x < -1024) || (x == NAN)){
        all_found = true;
        return DBL_MAX;
    }
    if(DEBUG) cout << "number of iterations: " << iterations << endl;
    return x;
}



void Swap(double *a, double *b) {
    double temp = *a;
    *a = *b;
    *b = temp;
}

void BubbleSort(vector<double> &array) {
    for (int i = 0; i < array.size(); i++)
    {
        for(int j = 0; j < array.size() - 1; j++)
        {   
            if (array[j] > array[j+1])
                Swap(&array[j], &array[j+1]);
        }
    }
}

double get_priority_root(vector<double> roots){
    //return the lowest, positive, real root
    for(int i = 0; i < roots.size(); i++){
        if(roots.at(i)  < 0){
            roots.at(i) = 1024;
        }
    }
    BubbleSort(roots);
    if(roots.at(0) >= 1024){
        return -1;
    }
    return roots.at(0);
}

double deep_impact(double (*func)(double), double estimate){
    vector<double> roots;
    bool all_found = false;

    /*
    for(int i = 0; i < 5; i++){
        roots.push_back(find_next_root(func, estimate, roots, all_found));
        cout << roots.back() << " is a root and all_found is " << all_found << endl;
    }
    */

    while(all_found == false){
        double root = find_next_root(func, estimate, roots, all_found);
        if(all_found == true){
            break;
        } else{
            roots.push_back(root);
        }
    }
    double priority = get_priority_root(roots);
    if(priority == -1){
        perror("No valid trajectory");
        exit(1);
    } else{
        return priority;
    }
    
}


int main(){
    auto start = high_resolution_clock::now();

    double root = deep_impact(&example_func, 1);

    auto stop = high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cout << "runtime in microseconds " << duration.count() << endl;

    cout << "the final root is " << root << endl;
    return 0;
}

/*
cannot self determine the order of the polynomial
    could return an all found if too many iterations pass

ideally needs to find imaginary solutions

needs to know to exit with error if there is no real positive solution




*/