#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>
//#include <Adafruit_LSM6DSO32.h>
#include <HardwareSerial.h>
//#include <TinyGPSPlus.h>

// put function declarations here:

MS5611 ms5611;
//Adafruit_LSM6DSO32 LSM6;
HardwareSerial GPS(2);
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

void setup() 
{
  // put your setup code here, to run once:
  /*
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

Serial.begin(115200);   //Set baud to 115200, standard for ESP32
}


void loop(){
// put your main code here, to run repeatedly:

  //Read in data for sensor
  
  //ms5611.read();


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
  /*
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
  //delay(3000);
  Serial.println("Hello World");

}

// put function definitions here:
/*
#include <Arduino.h>
#include <BMI088.h>
#include <SPI.h>
#include <RadioLib.h>
#include <MS5611.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#define ACCEL_CS      2
#define GYRO_CS       21

#define SCK           12
#define MOSI          11
#define MISO          13

#define GPS_RESET     3

#define POWER_LED     5

#define RADIO_RESET   4
#define RADIO_CS      10
#define RADIO_INT     1

#define BATTERY_SENSE 6
#define CURRENT_SENSE 7

#define SDA           8
#define SCL           9

#define PYRO_1_SENSE  14
#define PYRO_1_GATE   38
#define PYRO_2_SENSE  17
#define PYRO_2_GATE   48
#define PYRO_3_SENSE  18
#define PYRO_3_GATE   47

#define LORA_BW       125.0
#define LORA_SF       9
#define LORA_CR       7
#define LORA_SYNCWORD 18
#define LORA_POWER    20
#define LORA_PREAMBLE 8

Bmi088 bmi(SPI, ACCEL_CS, GYRO_CS);
RFM95 radio = new Module(RADIO_CS, RADIO_INT, RADIO_RESET, RADIOLIB_NC);
MS5611 barometer(0x77);
SFE_UBLOX_GNSS gps;

class VoltageDivPin {
  public:
  int pin;

  VoltageDivPin(int p) : pin(p) {
    pinMode(p, INPUT);
    //analogSetPinAttenuation(p, adc_attenuation_t::ADC_11db);
  }

  float readVoltage() {
    uint16_t v = analogRead(pin);
    uint32_t millivolts = analogReadMilliVolts(pin);
    //Serial.printf("pin = %d, %d mV\n", v, millivolts);
    float voltage = ((float)millivolts) / 1000.0;
    float divisor = 10.0 / (10.0 + 33.0);

    return voltage / divisor;
  }
};

VoltageDivPin battery(6);

void setup() {
  // pull radio CS high
  pinMode(RADIO_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);

  // set up I2C
  Wire.begin(SDA, SCL, 320000);

  // set up barometer
  if (!barometer.begin()) {
    Serial.println("barometer error");
  }

  Serial.begin();

  sleep(10);
  Serial.println("running");

  // set up SPI for BMI088
  SPI.setFrequency(5000000);
  SPI.begin(SCK, MISO, MOSI);

  int status = bmi.begin();
  if (status < 0) {
    Serial.printf("BMI088 error = %d\n", status);
  }

  bmi.setRange(Bmi088::ACCEL_RANGE_3G, Bmi088::GYRO_RANGE_125DPS);

  status = radio.begin(915.0, LORA_BW, LORA_SF, LORA_CR, LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLE, 0);
  if (status != 0) {
    Serial.printf("Radio error = %d\n", status);
  }

  while (gps.begin() == false) {
    Serial.println("u-blox GNSS not detected at default I2C address. Retrying...");
    delay(1000);
  }
  gps.setI2COutput(COM_TYPE_UBX);

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

void loop() {
  bmi.readSensor();

  float x = bmi.getAccelX_mss();
  float y = bmi.getAccelY_mss();
  float z = bmi.getAccelZ_mss();

  float pitch = bmi.getGyroX_rads();
  float roll = bmi.getGyroY_rads();
  float yaw = bmi.getGyroZ_rads();

  float temp = bmi.getTemperature_C();
  //Serial.printf("x = %f, y = %f, z = %f, pitch = %f, roll = %f, yaw = %f, temp = %f\n", x, y, z, pitch, roll, yaw, temp);

  if (barometer.read() != MS5611_READ_OK) {
    Serial.println("barometer read error");
  }
  float pressure = barometer.getPressure();
  float baro_temp = barometer.getTemperature();

  //Serial.printf("pressure = %f, temperature = %f\n", pressure, baro_temp);

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

  Serial.printf("battery voltage = %f\n", battery.readVoltage());

  //sleep(1);
  delayMicroseconds(500000);
}
*/