#include <Romi32U4.h>
#include "algorithms.h"
#include <math.h>
// #include <DFW.h>
// using namespace std;

float medianReturn;
int i;
float transit;

void Algorithm::SortAscending(float inputArray[], int n) {
    int i, j, min_idx;

    // One by one move boundary of
    // unsorted subarray
    for (i = 0; i < n - 1; i++) {
 
        // Find the minimum element in
        // unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++) {
            if (inputArray[j] < inputArray[min_idx])
                min_idx = j;
        }
 
        // Swap the found minimum element
        // with the first element
        if (min_idx != i) {
            transit = inputArray[i];
            inputArray[i] = inputArray[min_idx];
            inputArray[min_idx] = transit;
        }
    }
}

float Algorithm::FetchMedian(float inputArray[], int n, bool even) {
    // for some reason, platform.io refuses to allow me to include the 
    // iostream source file for c++, breaking this. Need to talk to a TA.
    if (even == true) {
        medianReturn = inputArray[(n - 2) / 2];
    } else if (even == false) {
        medianReturn = inputArray[(n - 1) / 2];
    }
    return medianReturn;
}