#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <RadioLib.h>
#include <MS5611.h>
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

MS5611 barometer(0x76);
//MS5x barometer(&Wire);
//Adafruit_LSM6DSO32 LSM6;
//HardwareSerial GPS(2);
//RFM95 radio = new Module(RADIO_CS, RADIO_INT, RADIO_RESET, RADIOLIB_NC);
//TinyGPSPlus gps;

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


void setup() {
  
  //pinMode(SCL, INPUT);
  //digitalWrite(SCL, LOW);
  Serial.begin(115200);
  delay(5000);
  Wire.begin(SDA, SCL, 320000);
 // set up barometer
  if (!barometer.begin()) {
    Serial.println("barometer error");
  }
  /*
   while(1) {
      Serial.println("running");
      delay(1000);
  }
  */
  /*
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, LOW);
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  */
 /*
 Wire.begin(SDA, SCL, 320000);
  while(1) {
      Serial.println("sent running");
      delay(1000);
  }
  */
  // pull radio CS high
  /*
  int status;
  pinMode(RADIO_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);

  
  Serial.begin(115200);

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
  */
  /*
  //esp_log_level_set("*", ESP_LOG_VERBOSE);  // or INFO, WARN, ERROR
  ESP_LOGI(TAG, "Hello from ESP_LOGI");
  ESP_LOGE(TAG, "This is an error!");
  while (!Serial);

  Serial.println("Before Wire.begin");

  Wire.begin(SDA, SCL, 320000);
  //bool result = Wire.begin(SDA, SCL);
  //Serial.printf("Wire.begin() returned: %s\n", result ? "SUCCESS" : "FAILURE");
  delay(100);  // Add a small delay to stabilize I2C

  Serial.println("After Wire.begin");
  Serial.println("I2C Scanner running...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      Serial.println(address, HEX);
    }
  }
  */
  //Serial.println("Hello World, Before wire begin");
  //pinMode(SDA, INPUT_PULLUP);  // SDA
  //pinMode(SCL, INPUT_PULLUP);  // SCL
  /*
  Wire.begin(SDA,SCL); // need to change for the I2C lines for the LSM6
  while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
		Serial.println(F("Error connecting..."));
		delay(500);
	}
	Serial.println(F("Connected to Sensor"));
	delay(5);
  */
  //Serial.println(" After wire begin");

  

  //GPS.begin(9600, SERIAL_8N1, 16, 17); //Set GPS to 9600 baud, 8 bits, no parity, 1 stop but with TX 16 RX 17

  //If no valid sensor found print out
  /*
  if (!ms5611.begin()) {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    while (1);
  }
  //If sensor found print it out
  Serial.println("MS5611 found!");

  //If lsm6 isn't found print error
  if (!LSM6.begin_I2C()) {
    Serial.println("Could not find a valid LSM6 sensor, check wiring!");
    while (1);
  }
  //If lsm6 is found print out validation message
  Serial.println("LSM6 found!");
  */
  /*
  LSM6.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  Serial.println("Acceleration range set to +/- 16 G...");

  LSM6.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.println("Gyro range set to 250 DPS...");

  LSM6.setAccelDataRate(LSM6DS_RATE_416_HZ);
  Serial.println("Data rate for accel set to 416 Hz...");

  LSM6.setGyroDataRate(LSM6DS_RATE_416_HZ);
  Serial.println("Data rate for gyro set to 416 Hz...");


  //Set base pressure to measure change in height 

  
  Serial.println("Move to base height now...");
  delay(3000);

  Serial.println("Hold at this height now for 5 seconds");
  delay(1000);

  for (int i = 0; i < 1000; i++) {
  ms5611.read();
  sum += ms5611.getPressurePascal();
  delay(10);
}
  P0 = sum / 1000.0;



Serial.begin(115200);   //Set baud to 115200, standard for ESP32
*/
}


void loop(){
// put your main code here, to run repeatedly:
 delayMicroseconds(500000);
  if (barometer.read() != MS5611_READ_OK) {
    Serial.println("barometer read error");
  }
  float pressure = barometer.getPressure();
  float baro_temp = barometer.getTemperature();

  Serial.printf("pressure = %f, temperature = %f\n", pressure, baro_temp);

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
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  LSM6.getEvent(&accel, &gyro, &temp);

  if (abs(accel.acceleration.x) > 1.0 || abs(accel.acceleration.y) > 1.0 || abs(accel.acceleration.z) > 1.0) {

  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
  
  }
  */


/*
  currentMillis = millis();
  minutesPassed = (currentMillis - startMillis) / 60000;

while (GPS.available()) {
  char c = GPS.read();
  gps.encode(c);  // Always feed data to TinyGPSPlus
  nmeaData += c;  // Store raw NMEA data
}  // Continuously feed GPS data to TinyGPSPlus


  // Only print updates once per interval
  if (currentMillis - gpsLastUpdate >= gpsInterval) {
    gpsLastUpdate = currentMillis;

    Serial.print(nmeaData);
    nmeaData = "";

    Serial.print("Minutes passed: ");
    Serial.println(minutesPassed);

    int satellites = gps.satellites.value();
    Serial.print("Satellites in view: ");
    Serial.println(satellites);

    if (gps.location.isValid()) {
        Serial.print("Lat: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Lon: ");
        Serial.println(gps.location.lng(), 6);
    } else {
        Serial.println("Waiting for GPS fix...");
    }
  }

  */
  //delay(3000);  
  /*
  //If lsm6 isn't found print error
  if (!LSM6.begin_I2C()) {
    Serial.println("Could not find a valid LSM6 sensor, check wiring!");
    while (1);
  }
  //If lsm6 is found print out validation message
  Serial.println("LSM6 found!");
  */


}