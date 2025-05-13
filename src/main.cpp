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
#define LORA_BW       62.5
#define LORA_SF       11
#define LORA_CR       7
#define LORA_SYNCWORD 18
#define LORA_POWER    20
#define LORA_PREAMBLE 8

// packet format
struct GPSData{
  int32_t lat, lon, a, p, alt;
  uint32_t time;


};

// Sensor and radio initializations
MS5611 barometer(0x76);
LSM6 imu; 
SFE_UBLOX_GNSS gps; 
RFM95 radio = new Module(RADIO_CS, RADIO_INT, RADIO_RESET, RADIOLIB_NC);


void setup() {
  // allow for monitoring
  Serial.begin(115200);
  
  // set GPS reset high to allow to start calibrating
  pinMode(GPS_RESET, OUTPUT);
  digitalWrite(GPS_RESET, HIGH);

  // Setup I2C
  Wire.begin(SDA, SCL, 50000);
  delay(1000);

 // set up barometer
  while (!barometer.begin()) {
    Serial.println("barometer error");
  }
  barometer.reset(1);
  
  
  // GPS setup
  while (gps.begin() == false) {
    Serial.println("u-blox GNSS not detected at default I2C address. Retrying...");
    delay(1000);
  }
  gps.setI2COutput(COM_TYPE_UBX);
  
  // IMU setup
  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
 
   // pull radio CS high
  int status;
  pinMode(RADIO_CS, OUTPUT);
  digitalWrite(RADIO_CS, HIGH);

  // set up SPI for RFM95W
  SPI.setFrequency(5000000);
  SPI.begin(SCK, MISO, MOSI);

  // Configure radio with specified LoRa values
  status = radio.begin(915.0, LORA_BW, LORA_SF, LORA_CR, LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLE, 0);
  if (status != 0) {
    Serial.printf("Radio error = %d\n", status);
  }
}
  


void loop(){
// put your main code here, to run repeatedly



 delayMicroseconds(500000);

  delay(100);

    // Read pressure from barometer
    if (barometer.read() != MS5611_READ_OK) {
    Serial.println("barometer read error");
    }
    float pressure = barometer.getPressure();
    Serial.printf("pressure = %f ",  pressure);


    // Read data from IMU
    imu.read();

    // read GPS data
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

    Serial.printf("%d satellites in view, fix type = %d\n", gps.getSIV(), gps.getFixType() );
    // configure packet for sending
    GPSData packet;
    packet.lat = gps.getLatitude();
    packet.lon = gps.getLongitude();

    // convert read acceleration value into m/s
    float accel_ms = (float)imu.a.z/((float)32768) * ((float)4) * ((float)9.8); // factor from data sheet
    Serial.print(accel_ms);

    Serial.print(" (m/s)");
    // convert value so it isn't improperly rounded when sent in packet
    packet.a = accel_ms * 1000;
    // grab pressure value to be sent 
    packet.p = barometer.getPressurePascal();
    // use GPS altitude to be eventually be setn
    packet.alt = altitude;
    uint32_t us; // for time if we wanted time to be sent

    // transmit packet with sensor data
    int state =  radio.transmit((uint8_t*)&packet, sizeof(packet),0);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.printf("Transmit error = %d\n", state);
    } else {
      Serial.println("sent packet");
    }    

    // GPS debugging statement
  } else {
    Serial.printf("no fix, fix type = %d\n", gps.getFixType());
  }
  
}