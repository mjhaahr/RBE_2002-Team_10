#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

#include <Romi32U4.h>

class MedianFilter{
    private:
        float dataArray[5] = {0};
        float medianArray[5] = {0};
        int pos = 0; //position of next data point to add (oldest data point)
        
        
    public:
        void Sort(int, int);
        void Init(void);
        float Filter(float);
};

#endif