#ifndef DS3231Clock_H
#define DS3231Clock_H
#include <Arduino.h>

class DS3231Clock {

  public:
    void initClock();
    void clk();
    boolean isNight();
    void setTime(int hours);

  private:
    int getIntHours();
};

#endif
