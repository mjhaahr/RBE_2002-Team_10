#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

#include <Romi32U4.h>

class MedianFilter{
    private:
        int dataArray[5] = {0};
        int medianArray[5] = {0};
        int pos = 0; //position of next data point to add (oldest data point)
        
        
    public:
        void Sort(int, int);
        void Init(void);
        int Filter(int);
};

#endif