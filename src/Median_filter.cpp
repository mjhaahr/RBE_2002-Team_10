#include "Median_filter.h"

void MedianFilter::Init(void)
{
    // no initialization required, but good practice
}

void MedianFilter::Sort(int index_a, int index_b)
{
    if(medianArray[index_a] < medianArray[index_b]){
        int temp = medianArray[index_a];
        medianArray[index_a] = medianArray[index_b];
        medianArray[index_b] = temp;
    }
}

int MedianFilter::Filter(int measurement)
{
    dataArray[pos] = measurement;
    for(int i = 0; i < 5; i++){
    	medianArray[i] = dataArray[i];
    }
    
    Sort(0,1);
    Sort(3,4);
    Sort(0,2);
    Sort(1,2);
    Sort(0,3);
    Sort(2,3);
    Sort(1,4);
    Sort(1,2);
    
    pos++; //increment position to add new data
    pos %= 5; //cap position to within 0-4
    

    return medianArray[2];
}