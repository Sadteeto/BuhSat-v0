#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFun_BMI270_Arduino_Library.h"
// #include "DFRobot_QMC5883.h"
#include "DFRobot_BMP3XX.h"
#include "TinyGPS++.h"
#include <HardwareSerial.h>

#define UART1_TX 1
#define UART1_RX 2
#define I2C1_SCL 4
#define I2C1_SDA 5
#define SPI1_MOSI 8
#define SPI1_MISO 9
#define SPI1_CLK 10
#define SPI1_CS 11
#define I2C2_SCL 12
#define I2C2_SDA 13
#define UART2_TX 17
#define UART2_RX 18

#define SerialUSB Serial2

// put function declarations here:
// the pi pico has its pin
// BMI270 sensor INT1 ---> 3
// BMI270 sensor INT2 ---> 4
// BMP390 sensor INT  ---> 2
// I2C SCL pin ---> 21
// I2C SDA pin ---> 20

// BMI270 AND BMP390 AND QMC5883L ON THE SAME I2C BUS
// RFD900_RADIO TX ---> 12
// RFD900_RADIO RX ---> 13

// TwoWire WireI2C(20, 21);
// TwoWire I2C = TwoWire(0); /* create i2c instance */
// UART RFD900_RADIO(12, 13);
TinyGPSPlus gps;
HardwareSerial RFD900_RADIO(1);
HardwareSerial GPSSerial(2);
// DFRobot_QMC5883 compass(&WireI2C, /*I2C addr*/QMC5883_ADDRESS);
DFRobot_BMP388_I2C baro(&Wire, baro.eSDOGND);
BMI270 imu;
volatile float declinatimonAngle = (11.0 + (7.0 / 60.0)) / (180 / PI);
uint8_t bmiAddress = BMI2_I2C_PRIM_ADDR; // 0x68
// uint8_t bmiAddress = BMI2_I2C_SEC_ADDR; // 0x69
uint8_t ledState = 0;
struct
{
  // float magX;
  // float magY;
  // float magZ;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temp;
  float press;
  double alt;
  double lat;
  double lng;
  uint32_t time;
} data_combined;

void setup()
{
  // SETUP i2c and virtual serial port and RFD900_RADIO
  Wire.setPins(I2C1_SDA, I2C1_SCL);
  Wire.begin();
  // WireI2C.setPins(I2C_SDA, I2C_SCL);
  // WireI2C.begin(4,5);
  // I2C.begin(I2C_SDA, I2C_SCL); /* initialise i2c bus with sda and scl, third arg clock freq tbd*/
  SerialUSB.begin(9600);
  RFD900_RADIO.begin(9600, UART2_RX, UART2_TX);
  pinMode(25, OUTPUT);
  GPSSerial.begin(9600, UART1_RX, UART1_TX);

  // SETUP QMC5883L on WireI2C

  // while (!compass.begin())
  // {
  //   SerialUSB.println("Could not find a valid 5883 sensor, check wiring!");
  //   delay(500);
  // }

  // if(compass.isQMC())
  // {
  //   SerialUSB.println("Initialize QMC5883");
  //   compass.setRange(QMC5883_RANGE_2GA);
  //   SerialUSB.print("compass range is:");
  //   SerialUSB.println(compass.getRange());

  //   compass.setMeasurementMode(QMC5883_CONTINOUS);
  //   SerialUSB.print("compass measurement mode is:");
  //   SerialUSB.println(compass.getMeasurementMode());

  //   compass.setDataRate(QMC5883_DATARATE_50HZ);
  //   SerialUSB.print("compass data rate is:");
  //   SerialUSB.println(compass.getDataRate());

  //   compass.setSamples(QMC5883_SAMPLES_8);
  //   SerialUSB.print("compass samples is:");
  //   SerialUSB.println(compass.getSamples());
  // }

  // setup BMI270 on WireI2C
  while (imu.beginI2C(bmiAddress, Wire) != BMI2_OK)
  {
    // Not connected, inform user
    SerialUSB.println("Error: BMI270 not connected, check wiring and I2C address!");

    // Wait a bit to see if connection is established
    delay(1000);
  }

  // setup BMP390 on WireI2C
  int rslt;
  while (ERR_OK != (rslt = baro.begin()))
  {
    if (ERR_DATA_BUS == rslt)
    {
      SerialUSB.println("Data bus error!!!");
    }
    else if (ERR_IC_VERSION == rslt)
    {
      SerialUSB.println("Chip versions do not match!!!");
    }
    delay(3000);
  }
  SerialUSB.println("Begin ok!");

  while (!baro.setSamplingMode(baro.eUltraPrecision))
  {
    SerialUSB.println("Set samping mode fail, retrying....");
    delay(3000);
  }
  /* Get the sampling period of the current measurement mode, unit: us */
  float sampingPeriodus = baro.getSamplingPeriodUS();
  SerialUSB.print("samping period : ");
  SerialUSB.print(sampingPeriodus);
  SerialUSB.println(" us");

  /* Get the sampling frequency of the current measurement mode, unit: Hz */
  float sampingFrequencyHz = 1000000 / sampingPeriodus;
  SerialUSB.print("samping frequency : ");
  SerialUSB.print(sampingFrequencyHz);
  SerialUSB.println(" Hz");

  SerialUSB.println();
}

void loop()
{
  ledState ^= 1;

  /* GPS data */
  while (GPSSerial.available() > 0)
  {
    gps.encode(GPSSerial.read());
  }

  // read MAG data
  // compass.setDeclinationAngle(declinationAngle);
  // sVector_t mag = compass.readRaw();
  // read IMU data
  imu.getSensorData();
  // read BARO data
  float temperature = baro.readTempC();
  float Pressure = baro.readPressPa();
  float altitude = baro.readAltitudeM();

  // flip 25 pin high
  digitalWrite(25, ledState);

  // data_combined.magX = mag.XAxis;
  // data_combined.magY = mag.YAxis;
  // data_combined.magZ = mag.ZAxis;
  data_combined.temp = temperature;
  data_combined.press = Pressure;
  data_combined.alt = altitude;
  data_combined.accX = imu.data.accelY;
  data_combined.accY = -imu.data.accelZ;
  data_combined.accZ = imu.data.accelX;
  data_combined.gyroX = -imu.data.gyroY;
  data_combined.gyroY = imu.data.gyroZ;
  data_combined.gyroZ = -imu.data.gyroX;

  // add gps data
  data_combined.lat = gps.location.lat();
  data_combined.lng = gps.location.lng();
  data_combined.alt = gps.altitude.meters();
  data_combined.time = gps.time.value();

  // // print MAG data
  // SerialUSB.print("MAG X: ");
  // SerialUSB.print(mag.XAxis);
  // SerialUSB.print(" MAG Y: ");
  // SerialUSB.print(mag.YAxis);
  // SerialUSB.print(" MAG Z: ");
  // SerialUSB.print(mag.ZAxis);

  // print BARO data
  SerialUSB.print("temp: ");
  SerialUSB.print(data_combined.temp);
  SerialUSB.print(" pres: ");
  SerialUSB.print(data_combined.press);
  SerialUSB.print(" alt: ");
  SerialUSB.print(data_combined.alt);

  // // print IMU data
  SerialUSB.print(" trans");
  SerialUSB.print(" x: ");
  SerialUSB.print(data_combined.accX, 3);
  SerialUSB.print(" y: ");
  SerialUSB.print(data_combined.accY, 3);
  SerialUSB.print(" z: ");
  SerialUSB.print(data_combined.accZ, 3);
  SerialUSB.print(" rotat");
  SerialUSB.print(" x: ");
  SerialUSB.print(data_combined.gyroX, 3);
  SerialUSB.print(" y: ");
  SerialUSB.print(data_combined.gyroY, 3);
  SerialUSB.print(" z: ");
  SerialUSB.print(data_combined.gyroZ, 3);
  SerialUSB.print("\n");

  RFD900_RADIO.write((uint8_t *)&data_combined, sizeof(data_combined));
}
