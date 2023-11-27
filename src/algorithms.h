#ifndef ALGORITHMS
#define ALGORITHMS

#include <math.h>
#include <Romi32u4.h>

class Algorithm {  
    private:
        int array[5] = {0};
        float medianReturn;
        int i;
        float transit;
      
    public:
        float FetchMedian(float inputArray[], int n, bool even);
        void SortAscending(float inputArray[], int n);
        void ComplexTypeSort(int, int);
        void Init(void);
        int ComplexTypeFilter(int);
};

#endif