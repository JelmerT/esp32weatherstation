#include <Arduino.h>

//#include "WindSensor.h"
//#include "RainSensor.h"

#include <Wire.h>

#include <pms.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_VEML6075.h"
#include "Adafruit_BMP3XX.h"
#include "ens210.h" // ENS210 library
#include <Adafruit_MAX31865.h>
#include <SoftwareSerial.h>
#include <MHZ19.h>


// time library
#include <TimeLib.h>

// Pin Configs
#define APPin 22
#define APLed 19
#define STALed 23
//#define windDirPin 32
//#define windSpeedPin 33
//#define rainPin 27

// Firmware configs
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define SEALEVELPRESSURE_HPA (1013.25)


// Global variables

//ENS210
float ens210_temperature = 0;
int   ens210_humidity = 0;

//VEML6075
float veml6075_UVA = 0;
float veml6075_UVB = 0;
float veml6075_UVI = 0;

//TSL2591
int tsl_lux = 0;
int tsl_ir = 0;
int tsl_full = 0;
int tsl_vis = 0;

//MAX31865
float max31865_temperature = 0;

//BMP388
float bmp_temperature = 0; //*C
float bmp_pressure = 0; //hPa
float bmp_pressureHg = 0; //mmHg
float bmp_altitude = 0;

//PMS5003
float pms5003_pm1;
float pms5003_pm2;
float pms5003_pm10;


//MH-Z19B
float mhz19_co2;
float mhz19_min_co2;
float mhz19_temperature;
float mhz19_accuracy;

//WIND-DIR
//int windDir = 0; //0-7
//int windDirDeg = 0; //degrees
//float windDirAvg = 0;
//String windDirStr = "";

//Wind-SPD
//float windSpeed = 0; //m/s
//int beaufort = 0;
//String beaufortDesc = "";
//float windSpeedAvg = 0;
//bool prevWindPinVal = false;

// Rain
//float rainAmountAvg = 0;
//bool prevRainPinVal = false;


//serial variables
String serialIn;
bool serialRdy = false;

// time interval variables
unsigned long sensorInterval = 5000; // in ms
unsigned long printInterval = 10000; // in ms
unsigned long lastSensorTime = 0;
unsigned long lastPrintTime = 0;

// Classes
//WindSensor ws(windSpeedPin, windDirPin);
//RainSensor rs(rainPin);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_VEML6075 uv = Adafruit_VEML6075();
Adafruit_BMP3XX bmp; // I2C
Adafruit_MAX31865 mx = Adafruit_MAX31865(15, 13, 12, 14); //spi_cs,spi_mosi,spi_miso,spi_clk

SoftwareSerial ss(4,2);
MHZ19 mhz(&ss);
ENS210 ens210;

PMS pms(Serial1);
PMS::DATA data;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displayLuxSensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureLuxSensor(void)
{
  Serial.println(F("Configuring TSL2591"));
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC);
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}


/**************************************************************************/
/*
    Configures VEML6075
*/
/**************************************************************************/
void configureUvSensor(void){
  Serial.println("Configuring VEML6075");
    // Set the integration constant
  uv.setIntegrationTime(VEML6075_100MS);
  // Get the integration constant and print it!
  Serial.print("Integration time set to ");
  switch (uv.getIntegrationTime()) {
    case VEML6075_50MS: Serial.print("50"); break;
    case VEML6075_100MS: Serial.print("100"); break;
    case VEML6075_200MS: Serial.print("200"); break;
    case VEML6075_400MS: Serial.print("400"); break;
    case VEML6075_800MS: Serial.print("800"); break;
  }
  Serial.println("ms");

  // Set the high dynamic mode
  uv.setHighDynamic(true);
  // Get the mode
  if (uv.getHighDynamic()) {
    Serial.println("High dynamic reading mode");
  } else {
    Serial.println("Normal dynamic reading mode");
  }

  // Set the mode
  uv.setForcedMode(false);
  // Get the mode
  if (uv.getForcedMode()) {
    Serial.println("Forced reading mode");
  } else {
    Serial.println("Continuous reading mode");
  }

  // Set the calibration coefficients
  uv.setCoefficients(2.22, 1.33,  // veml6075_UVA_A and veml6075_UVA_B coefficients
                     2.95, 1.74,  // veml6075_UVB_C and veml6075_UVB_D coefficients
                     0.001461, 0.002591); // veml6075_UVA and veml6075_UVB responses
  // Set the calibration coefficients for PTFE window 0.7mm thick & 10mm dia.
  // uv.setCoefficients(2.22, 1.17,  // veml6075_UVA_A and veml6075_UVA_B coefficients
  //                 2.95, 1.58,  // veml6075_UVB_C and veml6075_UVB_D coefficients
  //                 0.007923, 0.008334); // veml6075_UVA and veml6075_UVB responses
}

/**************************************************************************/
/*
    Configures BMP388
*/
/**************************************************************************/
void configurePressureSensor(void){
  Serial.println("Configuring BMP388");
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

/**************************************************************************/
/*
    read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedLuxRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;

  tsl_lux = tsl.calculateLux(full, ir);
  tsl_ir = ir;
  tsl_full = full;
  tsl_vis = full - ir;
}

/**************************************************************************/
/*
    read MAX31865
*/
/**************************************************************************/
void rtdTemperatureRead(void){
  uint16_t rtd = mx.readRTD();

  //Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  max31865_temperature = mx.temperature(RNOMINAL, RREF);

  // Check and print any faults
  uint8_t fault = mx.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    mx.clearFault();
  }
}

/**************************************************************************/
/*
    read wind
*/
/**************************************************************************/
//void readWindSensor() {
  //if (digitalRead(windSpeedPin) && !prevWindPinVal) {
    //ws.calcWindSpeed();
  //}
  //prevWindPinVal = digitalRead(windSpeedPin);

  //ws.updateWindSensor();
  //windSpeed = ws.getWindSpeed();
  //beaufort = ws.getBeaufort();
  //beaufortDesc = ws.getBeaufortDesc();

  //ws.determineWindDir();
  //windDir = ws.getWindDir();
  //windDirDeg = ws.getWindDirDeg();
  //windDirStr = ws.getWindDirString();
//}

/**************************************************************************/
/*
    read rain
*/
/**************************************************************************/
//void readRainSensor() {
  //inverted logic
//  if (!digitalRead(rainPin) && prevRainPinVal) {
 //   Serial.println("Rainbucket tipped");
  //  rs.calcRainAmount();
 // }
 // prevRainPinVal = digitalRead(rainPin);
//}
/**************************************************************************/
/*
    read Hall
*/
/**************************************************************************/

    int hall_sensor_value = hallRead();
/**************************************************************************/
/*
    read PMS5003
*/
/**************************************************************************/
bool updatePmReads()
{
    int reads = 0;
    uint16_t min1 = 0, max1 = 0, sum1 = 0,
             min2 = 0, max2 = 0, sum2 = 0,
             min10 = 0, max10 = 0, sum10 = 0;
    for (int i = 0; i < 5; i++)
    {
        pms.requestRead();
        if (pms.read(data, 10000))
        {
            if (reads == 0)
            {
                min1 = data.PM_AE_UG_1_0;
                max1 = data.PM_AE_UG_1_0;
                sum1 = data.PM_AE_UG_1_0;
                min2 = data.PM_AE_UG_2_5;
                max2 = data.PM_AE_UG_2_5;
                sum2 = data.PM_AE_UG_2_5;
                min10 = data.PM_AE_UG_10_0;
                max10 = data.PM_AE_UG_10_0;
                sum10 = data.PM_AE_UG_10_0;
            }
            else
            {
                if (data.PM_AE_UG_1_0 < min1)
                    min1 = data.PM_AE_UG_1_0;
                if (max1 < data.PM_AE_UG_1_0)
                    max1 = data.PM_AE_UG_1_0;
                sum1 += data.PM_AE_UG_1_0;
                if (data.PM_AE_UG_2_5 < min2)
                    min2 = data.PM_AE_UG_2_5;
                if (max2 < data.PM_AE_UG_2_5)
                    max2 = data.PM_AE_UG_2_5;
                sum2 += data.PM_AE_UG_2_5;
                if (data.PM_AE_UG_10_0 < min10)
                    min10 = data.PM_AE_UG_10_0;
                if (max10 < data.PM_AE_UG_10_0)
                    max10 = data.PM_AE_UG_10_0;
                sum10 += data.PM_AE_UG_10_0;
            }
            reads++;
        }
    }
    if (reads > 2)
    {
        pms5003_pm1 = (float)(sum1 - min1 - max1) / (float)(reads - 2);
        pms5003_pm2 = (float)(sum2 - min2 - max2) / (float)(reads - 2);
        pms5003_pm10 = (float)(sum10 - min10 - max10) / (float)(reads - 2);
    }
    else
    {
        pms5003_pm1 = min1;
        pms5003_pm2 = min2;
        pms5003_pm10 = min10;
    }
    return reads > 0;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

/**************************************************************************/
/*
    Print all currently stored measurements over serial
*/
/**************************************************************************/
void printMeasurements(){
  unsigned long time = millis();
  Serial.printf("#####################################\n");
  Serial.println("Time active: " + String(time) + "ms\n " + String(time/1000/60) + " minutes\n " + String(time/1000/3600) + " Hours");
  Serial.printf("#####################################\n");
  Serial.printf("\n");

  Serial.println("Current time:");
  digitalClockDisplay();
  Serial.printf("\n");

  Serial.printf("Current unix time: %lu\n\n", now());

  Serial.printf("ENS210\n");
  Serial.printf("ENS-T° %.2f °C\n", ens210_temperature);
  Serial.printf("Humidity %i %%RH\n", ens210_humidity);

  Serial.printf("-------------------------------------\n");
  Serial.printf("VEML6075\n");
  Serial.printf("Raw UVA: %.2f\n", veml6075_UVA);
  Serial.printf("Raw UVB: %.2f\n", veml6075_UVB);
  Serial.printf("UV Index: %.2f\n", veml6075_UVI);

  Serial.printf("-------------------------------------\n");
  Serial.printf("TSL2591\n");
  Serial.printf("IR: %i\n", tsl_ir);
  Serial.printf("Full: %i\n", tsl_full);
  Serial.printf("Visible: %i\n", tsl_vis);
  Serial.printf("Lux: %i\n", tsl_lux);

  Serial.printf("-------------------------------------\n");
  Serial.printf("MAX31865\n");
  Serial.printf("PT1000-T°: %.2f\n", max31865_temperature);

  Serial.printf("-------------------------------------\n");
  Serial.printf("BMP388\n");
  Serial.printf("BMP-T°: %.2f °C\n", bmp_temperature);
  Serial.printf("Pressure: %.2f hPa\n", bmp_pressure);
  Serial.printf("Pressure Hg: %.2f mmHg\n", bmp_pressureHg);
  Serial.printf("Approx. Altitude: %.1f m\n", bmp_altitude);

  Serial.printf("-------------------------------------\n");
  Serial.printf("PMS5003\n");
  Serial.printf("PM1.5:  "); Serial.println(pms5003_pm1);
  Serial.printf("PM2.5:  "); Serial.println(pms5003_pm2);
  Serial.printf("PM10:   "); Serial.println(pms5003_pm10);

  Serial.printf("-------------------------------------\n");
  Serial.printf("MH-Z19B\n");
  Serial.printf("CO2: %.0f ppm\n", mhz19_co2);
  //Serial.printf("Min CO2: %f\n", mhz19_min_co2);
  Serial.printf("SensorT°: %.1f\n", mhz19_temperature);
  //Serial.printf("Accuracy: %f\n", mhz19_accuracy);
  Serial.printf("-------------------------------------\n");

//Hall read
  Serial.printf("Hall sensor value = ");
  Serial.println(hall_sensor_value);

  //Serial.printf("-------------------------------------\n");
  //Serial.printf("WIND-DIR \n");
    // Serial.printf("Winddir: %d \n", analogRead(windDirPin));
    // Serial.printf("Wind dir deg: %i\n\r",ws.getWindDirDeg());
    // Serial.printf("Wind dir string: %s\n\r",ws.getWindDirString().c_str());

  //Serial.printf("-------------------------------------\n");
  //Serial.printf("Wind-SPD \n");
  // Serial.println("Wind speed:         " + String(ws.getWindSpeed()) + "m/s, " + String(ws.getWindSpeed() * 3.6) + "km/h");
  // Serial.println("Beaufort:           " + String(ws.getBeaufort()) + " (" + ws.getBeaufortDesc() + ")");
  // Serial.println("Wind speed avg:     " + String(ws.getWindSpeedAvg(false)));
  // Serial.println("Wind direction:     " + ws.getWindDirString() + " (" + String(ws.getWindDir()) + ")");
  // Serial.println("Wind direction avg: " + String(ws.getWindDirAvg(false)));
  // Serial.println("Rain amount:        " + String(rs.getRainAmount(false)) + "mm");

  Serial.printf("##################xx#################\n");
}

/**************************************************************************/
/*
    Blink Led
*/
/**************************************************************************/

void blinkLed(int ledPin) {
digitalWrite(ledPin, HIGH); 
delay(1000); 
digitalWrite(ledPin, LOW); 
//delay(1000); 
}

/**************************************************************************/
/*
    Serial terminal stuff
*/
/**************************************************************************/
void handleSerial() {
  if (serialRdy) {
    if (serialIn.indexOf("p") != -1) {
      printMeasurements();
    }
    else if (serialIn.indexOf("r") != -1) {
      ESP.restart();
    }
    else if (serialIn.indexOf("t") != -1) {
      unsigned long time = millis();
      Serial.println("Time active: " + String(time) + "ms, " + String(time/1000/60) + " minutes");
    }
    else if (serialIn.indexOf("T") != -1) {
      unsigned long pctime;
      const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

      serialIn.remove(0,1); // remove the T
      pctime = serialIn.toInt(); // read the unix time
      if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
        setTime(pctime); // Sync Arduino clock to the time received on the serial port
      }
    }

    serialRdy = false;
    serialIn = "";
  }
}

void checkSerial() {
  while (Serial.available()) {
    char in = Serial.read();
    Serial.print(in);
    serialIn += in;
    if (in == '\n'){
      serialRdy = true;
      handleSerial();
    }
  }
}


/**************************************************************************/
/*
    SETUP function
*/
/**************************************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial1.begin(9600); // PMS sensor

  pinMode(APPin, INPUT_PULLUP);
  pinMode(APLed, OUTPUT);
  pinMode(STALed, OUTPUT);

  digitalWrite(APLed, LOW);
  digitalWrite(STALed, LOW);

  setTime(1357041600); // jan 1 2013

  //ws.initWindSensor();
  //rs.initRainSensor();

  Wire.begin(25, 26, 100000); //sda, scl, freq=100kHz

  ss.begin(9600);

  // Enable ENS210
  if (!ens210.begin()){
    Serial.println("Could not find a valid ENS210 sensor, Bailing out");
    while (1) { delay(100); }
  } else {
    Serial.println("ENS210 started");
  }

    //init LUX sensor
  if (!tsl.begin())
  {
    Serial.println(F("No TSL2591 sensor found ... Bailing out."));
    while (1) { delay(100); }
  } else {
    Serial.println("TSL2591 started");
  }

  if (! uv.begin()) {
    Serial.println("Failed to communicate with VEML6075 sensor ... Bailing out");
    while (1) { delay(100); }
  } else {
    Serial.println("VEML6075 started");
  }

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, Bailing out");
    while (1) { delay(100); }
  } else {
    Serial.println("BMP3 started");
  }

  mx.begin(MAX31865_2WIRE);  // set to 3WIRE or 4WIRE as necessary


  /* Display some basic information on LUX sensor */
  // displayLuxSensorDetails();

  /* Configure all the sensors */
  configureLuxSensor();
  configureUvSensor();
  configurePressureSensor();

  pms.passiveMode();
}

/**************************************************************************/
/*
    LOOP function
*/
/**************************************************************************/
void loop() {
  //serial debugging
  checkSerial();

  // wake up pms
  pms.wakeUp();

  //read sensors every 5 seconds
  if ((lastSensorTime + sensorInterval) < millis()) {

    // read ENS210
    int ens210_t_data, ens210_t_status, ens210_h_data, ens210_h_status;
    ens210.measure(&ens210_t_data, &ens210_t_status, &ens210_h_data, &ens210_h_status );
    ens210_temperature = ens210.toCelsius(ens210_t_data,10)/10.0;
    ens210_humidity = ens210.toPercentageH(ens210_h_data,1);

    // read VEML6075
    veml6075_UVA = uv.readUVA();
    veml6075_UVB = uv.readUVB();
    veml6075_UVI = uv.readUVI();

    // read TSL2591 sensor
    advancedLuxRead();

    // read MAX31865
    rtdTemperatureRead();
    
    //Hall sensor read
    hall_sensor_value = hallRead();

    // read bmp388
    if (! bmp.performReading()) {
      Serial.println("Failed to perform BMP388 reading");
      return;
    } else {
      bmp_temperature = bmp.temperature;
      bmp_pressure = (bmp.pressure / 100.0);
      bmp_pressureHg = (bmp.pressure / 100.0 * 0.75006);
      bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    //read pms5003
    if (!updatePmReads())
      Serial.println("Failed to read PMS5003");

    //read MHZ19
    if (mhz.retrieveData() == MHZ19_RESULT_OK)
    {
      mhz19_co2 = mhz.getCO2();
      mhz19_min_co2 = mhz.getMinCO2();
      mhz19_temperature = mhz.getTemperature();
      mhz19_accuracy = mhz.getAccuracy();
    }
    else
    {
      Serial.print(F("MHZ19 Error"));
    }

    //read wind sensor
    // readWindSensor();

    // read rain sensor
    // readRainSensor();

    lastSensorTime = millis();
  }

  //print readings every 10 seconds
  if ((lastPrintTime + printInterval) < millis()) {
    blinkLed(APLed);
    blinkLed(STALed); 
    printMeasurements();
    lastPrintTime = millis();
  }

}


