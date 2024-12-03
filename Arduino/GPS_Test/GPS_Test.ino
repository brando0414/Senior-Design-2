#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
//#include "Adafruit_BluefruitLE_UART.h"
//#include <SoftwareSerial.h>

// LilyGO T-SIM7000G Pinout
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define LED_PIN     12

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT  Serial1

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

int start = 0;

//SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
//                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

TinyGsm modem(SerialAT);

void setup(){
  SerialMon.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  SerialMon.println("Place your board outside to catch satelite signal");

  // Set LED OFF
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //Turn on the modem
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);

  delay(1000);
  
  // Set module baud rate and UART pins
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
  }
  
  // Print modem info
  String modemName = modem.getModemName();
  delay(500);
  SerialMon.println("Modem Name: " + modemName);

  String modemInfo = modem.getModemInfo();
  delay(500);
  SerialMon.println("Modem Info: " + modemInfo);
}

void loop(){
  SerialMon.println("Waiting for Start command.");
  while(Serial.available() == 0)
  {
  }
  start = Serial.parseInt();
  while(start == 1)
  {
    SerialMon.println("Start Received");
    // Set SIM7000G GPIO4 HIGH ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
      SerialMon.println(" SGPIO=0,4,1,1 false ");
    }

    modem.enableGPS();
    
    delay(15000);
    float lat      = 0;
    float lon      = 0;
    float speed    = 0;
    float alt     = 0;
    int   vsat     = 0;
    int   usat     = 0;
    float accuracy = 0;
    int   year     = 0;
    int   month    = 0;
    int   day      = 0;
    int   hour     = 0;
    int   min      = 0;
    int   sec      = 0;
    
    while(true) {
      lsm.read();  /* ask it to read in the data */ 

      /* Get a new sensor event */ 
      sensors_event_t a, m, g, temp;

      lsm.getEvent(&a, &m, &g, &temp); 

      Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
      Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
      Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

      Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
      Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
      Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

      Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
      Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
      Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");

      Serial.println();
      delay(200);
      SerialMon.println("Requesting current GPS/GNSS/GLONASS location");
      if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                      &year, &month, &day, &hour, &min, &sec)) {
        SerialMon.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));
        SerialMon.println("Speed: " + String(speed) + "\tAltitude: " + String(alt));
        SerialMon.println("Visible Satellites: " + String(vsat) + "\tUsed Satellites: " + String(usat));
        SerialMon.println("Accuracy: " + String(accuracy));
        SerialMon.println("Year: " + String(year) + "\tMonth: " + String(month) + "\tDay: " + String(day));
        SerialMon.println("Hour: " + String(hour) + "\tMinute: " + String(min) + "\tSecond: " + String(sec));
        //break;
      } 
      else {
        SerialMon.println("Couldn't get GPS/GNSS/GLONASS location, retrying in 5s.");
        delay(5000L);
      }
    }
    SerialMon.println("Retrieving GPS/GNSS/GLONASS location again as a string");
    String gps_raw = modem.getGPSraw();
    SerialMon.println("GPS/GNSS Based Location String: " + gps_raw);
    SerialMon.println("Disabling GPS");
    modem.disableGPS();

    // Set SIM7000G GPIO4 LOW ,turn off GPS power
    // CMD:AT+SGPIO=0,4,1,0
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,0");
    if (modem.waitResponse(10000L) != 1) {
      SerialMon.println(" SGPIO=0,4,1,0 false ");
    }

    delay(200);
    // Do nothing forevermore
    while (true) {
        modem.maintain();
    }
  }
}
