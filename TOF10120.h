#ifndef TOF10120_H
#define TOF10120_H
#include <Arduino.h>

class TOF10120 {

  public:
    void initSensor();
    void clk();
    bool isMovement();

  private:
    void printf_begin();
    int getDistance();
    int ReadDistance();
    void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned int cnt);
};

#endif
