#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>
#include <Adafruit_LSM6DSO32.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// put function declarations here:

MS5611 ms5611;
Adafruit_LSM6DSO32 LSM6;
HardwareSerial GPS(2);
TinyGPSPlus gps;

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

void setup() 
{
  // put your setup code here, to run once:

  Wire.begin(21,22);

  Serial.begin(115200);   //Set baud to 115200, standard for ESP32

  GPS.begin(9600, SERIAL_8N1, 16, 17); //Set GPS to 9600 baud, 8 bits, no parity, 1 stop but with TX 16 RX 17

  //If no valid sensor found print out
  
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

  LSM6.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  Serial.println("Acceleration range set to +/- 16 G...");

  LSM6.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.println("Gyro range set to 250 DPS...");

  LSM6.setAccelDataRate(LSM6DS_RATE_416_HZ);
  Serial.println("Data rate for accel set to 416 Hz...");

  LSM6.setGyroDataRate(LSM6DS_RATE_416_HZ);
  Serial.println("Data rate for gyro set to 416 Hz...");


  //Set base pressure to measure change in height 

  /*
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
  */


  
}

void loop(){
// put your main code here, to run repeatedly:

  //Read in data for sensor
  
  ms5611.read();


/*
  float temperature = ms5611.getTemperature(); // Temperature in °C
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
    ms5611.read();
    sum += ms5611.getPressurePascal();
    delay(10);
  }

  P = sum / 1000.0;

  float deltaH = -1*(T_0 / L) * (pow((P / P0), (R * L) / (g * M)) - 1);


  Serial.println("Change in height: ");
  Serial.println(deltaH * 3.28 );
  Serial.println("feet");
*/

  //Read pressure
  Serial.println("Pressure: ");
  Serial.println(ms5611.getPressure());
  Serial.println(" mbar");



  Serial.println("Temperature: ");
  Serial.println(ms5611.getTemperature());
  Serial.println(" degrees celsius");



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
 delay(3000);

}

// put function definitions here:

