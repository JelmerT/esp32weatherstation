#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include "sslCertificate.h"
#include <WebServer.h>
#include <SPIFFS.h>
#include <Preferences.h>
// #include <Adafruit_BME280.h>
#include <Wire.h>
#include <pms.h>
#include <PubSubClient.h>
#include "WindSensor.h"
#include "RainSensor.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_VEML6075.h"
#include "Adafruit_BMP3XX.h"
#include <Adafruit_MAX31865.h>
#include <SoftwareSerial.h>
#include <MHZ19.h>
#include "ens210.h" // ENS210 library

//#define solarRelay 18
//#define measBatt 34

#define APPin 22
#define APLed 19
#define STALed 23
#define windDirPin 32
#define windSpeedPin 33
#define rainPin 27

//#define battLow 11
//#define battFull 13.5
//#define battInterval 2000

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define sensorInterval 5000 //ms = 5 seconds

#define lastConnectedTimeout 600000 //ms = 10 minutes: 10 * 60 * 1000

#define hourMs 3600000 //ms (60 * 60 * 1000 ms)

#define SEALEVELPRESSURE_HPA (1013.25)

#define TOKEN "A1E-i3ileGBt5Wy54q3CivPDQp1wzxZ6OT" // Put your Ubidots' TOKEN
#define MQTT_CLIENT_NAME "ESP32WeatherStation1" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
                                           //it should be a random and unique ascii string and different from all other devices
#define VARIABLE_LABEL "sensor" // Assing the variable label
#define DEVICE_LABEL "esp32" // Assig the device label

//network settings
String APSSID = "WeatherStation";
String ssid;
String pass;

//thingspeak api and fields numbers
//https://thingspeak.com/channels/535447/private_show
bool thingspeakEnabled;
String thingspeakApi;
int tsfWindSpeed = 0;
int tsfWindDir = 0;
int tsfRainAmount = 0;
int tsfTemperature = 0;
int tsfHumidity = 0;
int tsfAirpressure = 0;
int tsfPM1 = 0;
int tsfPM2 = 0;
int tsfPM10 = 0;

//senseBox IDs
bool senseBoxEnabled;
String senseBoxStationId;
String senseBoxTempId;
String senseBoxHumId;
String senseBoxPressId;
String senseBoxWindSId;
String senseBoxWindDId;
String senseBoxRainId;
String senseBoxPM2Id;
String senseBoxPM1Id;
String senseBoxPM10Id;

//ubidots IDs
bool ubidotsEnabled = 0;
char payload[100];
char topic[150];
char mqttBroker[]  = "things.ubidots.com";

unsigned long uploadInterval = hourMs;
bool uploaded = false;


bool prevWindPinVal = false;
bool prevRainPinVal = false;

//current sensor values
int windDir = 0; //0-7
int windDirDeg = 0; //degrees
float windSpeed = 0; //m/s
int beaufort = 0;
String beaufortDesc = "";
float windSpeedAvg = 0;
float windDirAvg = 0;
String windDirStr = "";
float rainAmountAvg = 0;

//bmp sensor
float bmp_temperature = 0; //*C
float bmp_pressure = 0; //hPa
float bmp_altitude = 0;

//backwards compatibility
float pressure = 0;

float temperature = 0;

float humidity = 0; //%

// pm sensor
float PM10 = 0; //particle size: 10 um or less
float PM2 = 0; //particle size: 2.5 um or less
float PM1 = 0; //particle size: 1.5 um or less

float UVA = 0;
float UVB = 0;
float UVI = 0;

int tsl_lux = 0;
int tsl_ir = 0;
int tsl_full = 0;
int tsl_vis = 0;

//float batteryVoltage = 0; //v
//float batteryCharging = false;

//serial variables
String serialIn;
bool serialRdy = false;

unsigned long lastSensorTime = 0;
unsigned long lastUploadTime = 0;
unsigned long lastAPConnection = 0;
//unsigned long lastBattMeasurement = 0;

WindSensor ws(windSpeedPin, windDirPin);
RainSensor rs(rainPin);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_VEML6075 uv = Adafruit_VEML6075();
Adafruit_BMP3XX bmp; // I2C
Adafruit_MAX31865 mx = Adafruit_MAX31865(15, 13, 12, 14); //spi_cs,spi_mosi,spi_miso,spi_clk

SoftwareSerial ss(4,2);
MHZ19 mhz(&ss);
ENS210 ens210;

PMS pms(Serial1);
PMS::DATA data;
struct pms5003
{
    float pm1, pm2, pm10;
};
pms5003 pm;

//webserver, client and secure client pointers
WebServer* server = NULL;
WiFiClient* client = NULL;
WiFiClientSecure* clientS = NULL;

Preferences pref;

WiFiClient ubidots;

PubSubClient mqttclient(ubidots);

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
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
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
  uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                     2.95, 1.74,  // UVB_C and UVB_D coefficients
                     0.001461, 0.002591); // UVA and UVB responses
  // Set the calibration coefficients for PTFE window 0.7mm thick & 10mm dia.
  // uv.setCoefficients(2.22, 1.17,  // UVA_A and UVA_B coefficients
  //                 2.95, 1.58,  // UVB_C and UVB_D coefficients
  //                 0.007923, 0.008334); // UVA and UVB responses                  
}

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
    Show how to read IR and Full Spectrum at once and convert to lux
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

  Serial.print(F("IR: ")); Serial.println(tsl_ir);
  Serial.print(F("Full: ")); Serial.println(tsl_full);
  Serial.print(F("Visible: ")); Serial.println(tsl_vis);
  Serial.print(F("Lux: ")); Serial.println(tsl_lux, 6);
}

void rtdTemperatureRead(void){
  uint16_t rtd = mx.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  temperature = mx.temperature(RNOMINAL, RREF);
  Serial.print("Temperature = "); Serial.println(temperature);

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

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Serial1.begin(9600);
  
  SPIFFS.begin();
  Serial.println("SPIFFS started");
  pinMode(APPin, INPUT_PULLUP);
  pinMode(APLed, OUTPUT);
  pinMode(STALed, OUTPUT);
  //pinMode(solarRelay, OUTPUT);
  //pinMode(measBatt, ANALOG);
  
  digitalWrite(APLed, LOW);
  digitalWrite(STALed, LOW);
  //digitalWrite(solarRelay, LOW);

  ws.initWindSensor();
  rs.initRainSensor();
  
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

  loadNetworkCredentials();
  loadUploadSettings();
  initWiFi();

  if (ubidotsEnabled){
    mqttclient.setServer(mqttBroker, 1883);
    mqttclient.setCallback(callback);
  }

}

void loop() {
  //serial debugging
  checkSerial();
  
  // print analog in windDirPin
  // Serial.printf("Winddir: %d \n", analogRead(windDirPin));

  // wake up pms
  pms.wakeUp();

  //handle WiFi
  if (server != NULL)
    server->handleClient();

  //if the current wifi mode isn't STA and the timout time has passed -> restart
  if ((WiFi.getMode() != WIFI_STA) && ((lastAPConnection + lastConnectedTimeout) < millis())) {
    if (digitalRead(APPin)) {
      Serial.println("Last connection was more than 10 minutes ago. Restart.");
      ESP.restart();
    }
    else {//button still pressed
      lastAPConnection = millis(); //wait some time before checking again
    }
  }

  //reinit if the wifi connection is lost
  if ((WiFi.getMode() == WIFI_STA) && (WiFi.status() == WL_DISCONNECTED)) {
    digitalWrite(STALed, LOW);
    Serial.println("Lost connection. Trying to reconnect.");
    initWiFi();
  }

  //read sensors
  readWindSensor();
  // readRainSensor();

  //read sensors every 5 seconds
  if ((lastSensorTime + sensorInterval) < millis()) {
    Serial.println("--------------------------------------------");

    // read wind direction
    Serial.printf("Wind dir deg: %i\n\r",ws.getWindDirDeg());
    Serial.printf("Wind dir string: %s\n\r",ws.getWindDirString().c_str());

    // read Lux sensor
    advancedLuxRead();

    UVA = uv.readUVA();
    UVB = uv.readUVB();
    UVI = uv.readUVI();

    // read UV sensor
    Serial.print("Raw UVA reading:  "); Serial.println(UVA);
    Serial.print("Raw UVB reading:  "); Serial.println(UVB);
    Serial.print("UV Index reading: "); Serial.println(UVI);

    // read pressure sensor
    if (! bmp.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }

    bmp_temperature = bmp.temperature;
    Serial.print("Temperature = ");
    Serial.print(bmp_temperature);
    Serial.println(" *C");

    bmp_pressure = (bmp.pressure / 100.0);
    pressure = bmp_pressure;
    Serial.print("Pressure = ");
    Serial.print(bmp_pressure);
    Serial.println(" hPa");

    bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.print("Approx. Altitude = ");
    Serial.print(bmp_altitude);
    Serial.println(" m");

    //read pms5003
    if (!updatePmReads())
      Serial.println("Failed to read PMS5003");
    else {
      Serial.println("read PMS5003");
      PM1 = pm.pm1;
      PM2 = pm.pm2;
      PM10 = pm.pm10;
      Serial.print("PM1.5:  "); Serial.println(PM1);
      Serial.print("PM2.5:  "); Serial.println(PM2);
      Serial.print("PM10: "); Serial.println(PM10);
    }

    //read MHZ19
    MHZ19_RESULT response = mhz.retrieveData();
    if (response == MHZ19_RESULT_OK)
    {
      Serial.print(F("CO2: "));
      Serial.println(mhz.getCO2());
      Serial.print(F("Min CO2: "));
      Serial.println(mhz.getMinCO2());
      Serial.print(F("Temperature: "));
      Serial.println(mhz.getTemperature());
      Serial.print(F("Accuracy: "));
      Serial.println(mhz.getAccuracy());
    }
    else
    {
      Serial.print(F("Error, code: "));
      Serial.println(response);
    }

    int t_data, t_status, h_data, h_status;
    ens210.measure(&t_data, &t_status, &h_data, &h_status );

    Serial.print( ens210.toCelsius(t_data,10)/10.0, 1 ); Serial.print(" C, ");
    Serial.print( ens210.toPercentageH(h_data,1)      ); Serial.print(" %RH");
    Serial.println();

    // read
    rtdTemperatureRead();

    Serial.println("--------------------------------------------");

    lastSensorTime = millis();
  }

  //upload data if the uploadperiod has passed and if WiFi is connected
  if (((lastUploadTime + uploadInterval) < millis()) && (WiFi.status() == WL_CONNECTED)) {
    lastUploadTime = millis();
    Serial.println("Upload interval time passed");

    windSpeedAvg = ws.getWindSpeedAvg();
    windDirAvg = ws.getWindDirAvg();
    rainAmountAvg = rs.getRainAmount() * hourMs / uploadInterval;

    if (thingspeakEnabled) {
      if (uploadToThingspeak())
        Serial.println("Uploaded successfully");
      else
        Serial.println("Uploading failed");
    }
    else
      Serial.println("Thingspeak disabled");

    if (senseBoxEnabled) {
      if (uploadToSenseBox())
        Serial.println("Uploaded successfully");
      else
        Serial.println("Uploading failed");
    }
    else
      Serial.println("SenseBox disabled");

    if(ubidotsEnabled) {
      if(uploadToUbidots())
        Serial.println("Uploaded successfully");
      else
        Serial.println("Uploading failed");
    }
    else
      Serial.println("Ubidots disabled");
  }

  // handle battery (resistor divider: vBatt|--[470k]-+-[100k]--|gnd)
  //if ((lastBattMeasurement + battInterval) < millis()) {
   // lastBattMeasurement = millis();
    //float adcVoltage = ((float)analogRead(measBatt)/4096) * 3.3 + 0.15; //0.15 offset from real value
    //batteryVoltage = adcVoltage * 570 / 100 + 0.7; //analog read between 0 and 3.3v * resistor divider + 0.7v diode drop
    // Serial.println("adc voltage: " + String(adcVoltage) + ", batt voltage: " + String(batteryVoltage) + ", currently charging: " + String(batteryCharging ? "yes" : "no"));
    //if (batteryVoltage > battFull)
      //batteryCharging = false;
    //if (batteryVoltage < battLow)
     // batteryCharging = true;

    //digitalWrite(solarRelay, batteryCharging);
  //}
}

//reads the windsensor and stores the values in global variables
void readWindSensor() {
  if (digitalRead(windSpeedPin) && !prevWindPinVal) {
    ws.calcWindSpeed();
  }
  prevWindPinVal = digitalRead(windSpeedPin);

  ws.updateWindSensor();
  windSpeed = ws.getWindSpeed();
  beaufort = ws.getBeaufort();
  beaufortDesc = ws.getBeaufortDesc();

  ws.determineWindDir();
  windDir = ws.getWindDir();
  windDirDeg = ws.getWindDirDeg();
  windDirStr = ws.getWindDirString();
}

void readRainSensor() {
  //inverted logic
  if (!digitalRead(rainPin) && prevRainPinVal) {
    Serial.println("Rainbucket tipped");
    rs.calcRainAmount();
  }
  prevRainPinVal = digitalRead(rainPin);
}

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
        pm.pm1 = (float)(sum1 - min1 - max1) / (float)(reads - 2);
        pm.pm2 = (float)(sum2 - min2 - max2) / (float)(reads - 2);
        pm.pm10 = (float)(sum10 - min10 - max10) / (float)(reads - 2);
    }
    else
    {
        pm.pm1 = min1;
        pm.pm2 = min2;
        pm.pm10 = min10;
    }
    return reads > 0;
}
