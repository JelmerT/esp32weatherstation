#ifndef WINDSENSOR_H
#define WINDSENSOR_H

#include "Arduino.h"
#include <math.h>

#define D_MARGIN 10

#define D_N  0
#define D_NE 1
#define D_E  2
#define D_SE 3
#define D_S  4
#define D_SW 5
#define D_W  6
#define D_NW 7

class WindSensor {
  private:
    //                  N     NE    E    SE   S    SW    W     NW
    // int winDirVal[8] = {3000, 1690, 200, 570, 975, 2368, 3905, 3518};
    int winDirVal[8] = {3835, 3380, 2925, 2470, 2015, 1560, 1105, 650};
    
    #define windDirAnalogMax 3835
    #define windDirAnalogMin 188

    int windDir = -1;
    int lastDir = -1;
    int windDirDeg = -1;
    float windSpeed = 0;

    float windDirAvg = 0;
    long windDirAvgCnt = 0;
    float windSpeedAvg = 0;
    long windSpeedAvgCnt = 0;
    
    unsigned long lastWindSpeedUpdate = 0;
    unsigned long windSpeedTimeout = 2500;

    int beaufort[12] = {2, 6, 12, 19, 30, 40, 51, 62, 75, 87, 103, 117}; //km/h (http://www.hko.gov.hk/education/beaufort.htm)
    
    int speedPin;
    int dirPin;

    void addWindDirAvg();
    void addWindSpeedAvg();

  public:
    WindSensor(int _speedPin, int _dirPin);
    void initWindSensor();
    void setWindSpeedTimeout(unsigned long nWindSpeedTimeout);
    void updateWindSensor();
    void determineWindDir();
    int getWindDir();
    int getWindDirDeg();
    float getWindDirAvg(bool clearVars = true);
    String getWindDirString();
    void calcWindSpeed();
    float getWindSpeed();
    float getWindSpeedAvg(bool clearVars = true);
    int getBeaufort();
    String getBeaufortDesc();
};

#endif
