#ifndef IR_SENSOR
#define IR_SENSOR

#include <Romi32U4.h>

class IRsensor{
    private:
        const int pin_IR = A0;
        const float conversionFactor = 5.0/1024.0;
    public:
        void Init(void);
        float ReadData(void);
        void PrintData(void);
};

#endif