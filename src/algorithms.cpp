#include <Romi32U4.h>
#include "algorithms.h"
// #include <math.h>
// #include <DFW.h>
// using namespace std;

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

void Algorithm::Init(void)
{
    // no initialization required, but good practice
}

// below is Nemitz-provided code
void Algorithm::ComplexTypeSort(int index_a, int index_b)
{
    if(array[index_a] < array[index_b]){
        int temp = array[index_a];
        array[index_a] = array[index_b];
        array[index_b] = temp;
    }
}

int Algorithm::ComplexTypeFilter(int measurement)
{
    array[0] = measurement;
    for(int i = 4; i > 0; i--) array[i] = array[i-1];
    
    ComplexTypeSort(0,1);
    ComplexTypeSort(3,4);
    ComplexTypeSort(0,2);
    ComplexTypeSort(1,2);
    ComplexTypeSort(0,3);
    ComplexTypeSort(2,3);
    ComplexTypeSort(1,4);
    ComplexTypeSort(1,2);

    return array[2];
}