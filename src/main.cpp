#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <MS5611.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <LSM6.h>
#include "esp_log.h"



// put function declarations here:


#define SCK           12 // SPI SCK
#define MOSI          11 // SPI MOSI
#define MISO          13 // SPI MISO



#define RADIO_RESET   14 // for safe board 
#define RADIO_CS      10 // for safe board
#define RADIO_INT     9 // for safe board


#define SCL 5 // I2C SCL 
#define SDA 6 // I2C SDA

#define GPS_RESET     4 // for safe board

// needed for RFM95 module
#define LORA_BW       125.0
#define LORA_SF       9
#define LORA_CR       7
#define LORA_SYNCWORD 18
#define LORA_POWER    20
#define LORA_PREAMBLE 8

//MS5611 ONE(0x76);   //  0x76 = CSB to VCC
//MS5611 TWO(0x77);   //  0x77 = CSB to GND


MS5611 barometer(0x76);
LSM6 imu; 
//SFE_UBLOX_GNSS gps; 
//MS5x barometer(&Wire);
RFM95 radio = new Module(RADIO_CS, RADIO_INT, RADIO_RESET, RADIOLIB_NC);

float P0, P;
float sum = 0;
float T_0 = 288.15; //
float L = 0.0065;  // Temperature lapse rate in K/m
float R = 8.31447;  // Universal gas constant in J/(mol K)
float g = 9.80665;  // Gravitational acceleration in m/s²
float M = 0.0289644;

unsigned long startMillis = millis();  // To store the start time
unsigned long currentMillis;  // To store the current time
int minutesPassed = 0;  // Variable to store the number of minutes passed
unsigned long gpsLastUpdate = 0;
const unsigned long gpsInterval = 60000;  // Update GPS every 1 minute

String nmeaData = "";  
static const char* TAG = "MY_TAG";
uint32_t start, stop;
char report[80];


void setup() {
  Serial.begin(115200);
  //pinMode(SCL, INPUT);
  //digitalWrite(SCL, LOW);
  
  Wire.begin(SDA, SCL, 320000);
  delay(1000);
 // set up barometer
  while (!barometer.begin()) {
    Serial.println("barometer error");
  }
  // GPS code to eventually run
  /*
  while (gps.begin() == false) {
    Serial.println("u-blox GNSS not detected at default I2C address. Retrying...");
    delay(1000);
  }
  gps.setI2COutput(COM_TYPE_UBX);
  */
  
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
  /*
  while(1) {
      Serial.println("running");
      delay(1000);
  }
  */
  // pull radio CS high
  
  int status;
  pinMode(RADIO_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);
  // set up SPI for RFM95W
  SPI.setFrequency(5000000);
  SPI.begin(SCK, MISO, MOSI);

  
  status = radio.begin(915.0, LORA_BW, LORA_SF, LORA_CR, LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLE, 0);
  if (status != 0) {
    Serial.printf("Radio error = %d\n", status);
  }
  
  
   while (1) {
    int state = radio.transmit("test message");
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Transmit error = %d\n", state);
    } else {
      Serial.println("sent packet");
    }
    sleep(2);
    }
  
}
  


void loop(){
// put your main code here, to run repeatedly



 delayMicroseconds(500000);
  if (barometer.read() != MS5611_READ_OK) {
    Serial.println("barometer read error");
  }
  float pressure = barometer.getPressure();
  float baro_temp = barometer.getTemperature();

  Serial.printf("pressure = %f, temperature = %f\n", pressure, baro_temp);


  
  delayMicroseconds(500000);
  imu.read();

  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
  imu.a.x, imu.a.y, imu.a.z,
  imu.g.x, imu.g.y, imu.g.z);
  Serial.println(report);

  delay(100);


  /*
  //Read in data for sensor
  
  barometer.read();



  float temperature = barometer.getTemperature(); // Temperature in °C
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Calculate temperature in Kelvin
  float T = temperature + 273.15;

  Serial.println("Move to new height.");

  delay(5000);

  Serial.println("Calculating change in height from original position...");

  sum = 0;  

  for (int i = 0; i < 1000; i++) {
    barometer.read();
    sum += barometer.getPressurePascal();
    delay(10);
  }

  P = sum / 1000.0;

  float deltaH = -1*(T_0 / L) * (pow((P / P0), (R * L) / (g * M)) - 1);


  Serial.println("Change in height: ");
  Serial.println(deltaH * 3.28 );
  Serial.println("feet");

  
  //Read pressure
  Serial.println("Pressure: ");
  Serial.println(barometer.getPressure());
  Serial.println(" mbar");



  Serial.println("Temperature: ");
  Serial.println(barometer.getTemperature());
  Serial.println(" degrees celsius");
  */
  /*
    if (gps.getPVT() == true) {
    int32_t latitude = gps.getLatitude();
    Serial.print("Lat: ");
    Serial.print(latitude);

    int32_t longitude = gps.getLongitude();
    Serial.print(" Long: ");
    Serial.print(longitude);
    Serial.print(" (degrees * 10^-7)");

    int32_t altitude = gps.getAltitudeMSL();
    Serial.print(" Alt: ");
    Serial.print(altitude);
    Serial.print(" (mm)");

    Serial.println();

    Serial.printf("%d satellites in view\n", gps.getSIV());
  } else {
    Serial.printf("no fix, fix type = %d\n", gps.getFixType());
  }
  
  */



}