/****************************************************************
 * Example2_Advanced.ino
 * ICM 20948 Arduino Library Demo
 * Shows how to use granular configuration of the ICM 20948
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <WiFi.h>
#include <esp_now.h>
#include "USBCDC.h"
#include <RingBuf.h>
#include "att.hpp"

// #define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI     // Your desired SPI port.       Used only when "USE_SPI" is defined
#define SPI_FREQ 5000000 // You can override the default SPI frequency
#define CS_PIN 2         // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

att myAtt;
// OWN 0xE8, 0x3D, 0xC1, 0x82, 0x48, 0x00
uint8_t cannonMAC[] = {0x14, 0xC1, 0x9F, 0x28, 0x8D, 0x88};

#pragma pack(1)
struct attFrame_T
{
  uint8_t head;
  float att[3]; // pitch roll yaw
  float acc[3]; // x y z
  uint8_t crc;
  uint8_t tail;
};
#pragma pack()

struct attFrame_T send;
uint32_t waitInit = 0;

void setup()
{
  // SERIAL_PORT.begin(115200, SERIAL_8N1, 21, 20);
  SERIAL_PORT.begin(460800);
  // Serial1.begin(115200, SERIAL_8N1, 20, 21);

  while (!SERIAL_PORT)
  {
  };

  // WiFi.begin();
  WiFi.mode(WIFI_STA); // ESP-NOW need config WIFI-STA mode
  delay(2000);
  //  ESP-NOW init
  if (esp_now_init() != ESP_OK)
  {
    SERIAL_PORT.println("ESP-NOW init error");
    return;
  }
  //  add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, cannonMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    SERIAL_PORT.println("add peer error");
    return;
  }

  SERIAL_PORT.println(WiFi.macAddress());

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin(3, 2);
  // WIRE_PORT.setClock(400000);
#endif

  bool initialized = false;
  while (!initialized)
  {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm2; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps500; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
  SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
  SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }

  SERIAL_PORT.println();
  SERIAL_PORT.println(F("Configuration complete!"));

}

unsigned long last_update = 0;
int16_t maxX = -32767, maxY = -32767, maxZ = -32767;
int16_t minX = 32767, minY = 32767, minZ = 32767;
float offsetX = 0, offsetY = 0, offsetZ = 0;
// float calibX = 1, calibY = 1, calibZ = 1;

void loop()
{
  float mx, my, mz;
  float roll, pitch, heading;

  if (myICM.dataReady())
  {
    unsigned long now = millis();
    // 10ms cycle
    if (now - last_update >= 10)
    {
      last_update = now;

      ICM_20948_AGMT_t amgt = myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      myICM.readMag(AK09916_REG_ST2);
      // printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt structure

      if (myICM.magX() > maxX)
        maxX = myICM.magX();
      if (myICM.magX() < minX)
        minX = myICM.magX();
      if (myICM.magY() > maxY)
        maxY = myICM.magY();
      if (myICM.magY() < minY)
        minY = myICM.magY();
      if (myICM.magZ() > maxZ)
        maxZ = myICM.magZ();
      if (myICM.magZ() < minZ)
        minZ = myICM.magZ();
      offsetX = (float)(maxX - minX) / 2.0 + minX;
      offsetY = (float)(maxY - minY) / 2.0 + minY;
      offsetZ = (float)(maxZ - minZ) / 2.0 + minZ;

      myAtt.update(DEGREES_TO_RADIANS(myICM.gyrY()), DEGREES_TO_RADIANS(myICM.gyrX()), -DEGREES_TO_RADIANS(myICM.gyrZ()),
                   myICM.accY(), myICM.accX(), -myICM.accZ(),
                   -(myICM.magY() - offsetY) , (myICM.magX() - offsetX)  , -(myICM.magZ() - offsetZ) , false, 0.01);
      // printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units
      // Serial.println(now);

      // 1. get body acc
      pitch = DEGREES_TO_RADIANS(myAtt.Pitch);
      roll = DEGREES_TO_RADIANS(myAtt.Roll);
      heading = 0;

      float ax = myICM.accX();
      float ay = myICM.accY();
      float az = myICM.accZ();

      float gx = sin(pitch) * 9.81;
      float gy = -sin(roll) * cos(pitch) * 9.81;
      float gz = cos(roll) * cos(pitch) * 9.81;
      float ax_no_g = ax - gx;
      float ay_no_g = ay - gy;
      float az_no_g = az - gz;

      float cosR = cos(roll), sinR = sin(roll);
      float cosP = cos(pitch), sinP = sin(pitch);
      float cosY = cos(heading), sinY = sin(heading);

      float ax_enu = cosP * cosY * ax_no_g + (sinR * sinP * cosY - cosR * sinY) * ay_no_g + (cosR * sinP * cosY + sinR * sinY) * az_no_g;
      float ay_enu = cosP * sinY * ax_no_g + (sinR * sinP * sinY + cosR * cosY) * ay_no_g + (cosR * sinP * sinY - sinR * cosY) * az_no_g;
      float az_enu = -sinP * ax_no_g + sinR * cosP * ay_no_g + cosR * cosP * az_no_g;

      SERIAL_PORT.printf("att %f, %f, %f. gyro %f, %f, %f\n", myAtt.Pitch, myAtt.Roll, myAtt.Yaw, myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());

      memset(&send, 0, sizeof(send));
      send.head = 0xA5;
      send.att[0] = myAtt.Pitch; // pitch
      send.att[1] = myAtt.Roll;  // roll
      send.att[2] = myAtt.Yaw;   // yaw
      send.acc[0] = ax_enu;     // body acc x
      send.acc[1] = ay_enu;     // body acc y
      send.acc[2] = az_enu;     // body acc z
      uint8_t check = 0;
      check = crc8((uint8_t *)&send, sizeof(send) - 2);
      send.crc = check;
      send.tail = 0x5A;

      if (waitInit < 100)
      {
        // wait stable
        waitInit++;
      }
      else
      {
        esp_err_t result = esp_now_send(cannonMAC, (uint8_t *)&send, sizeof(send));
        // if (result == ESP_OK)
        // {
        //   SERIAL_PORT.println("esp-now success");
        // }
        // else
        // {
        //   SERIAL_PORT.println("esp-now failed");
        // }
      }
    }
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
  }

  if (readyToPrint())
  {
    // print the heading, pitch and roll
    ICM_20948_AGMT_t amgt = myICM.getAGMT();
    // Serial.print(heading);
    // Serial.print(",");
    // Serial.print(pitch);
    // Serial.print(",");
    // Serial.print(roll);
    // Serial.printf(",%d %d %f %f %f %d\n", maxX, minX, offsetX, offsetY, offsetZ, amgt.acc.axes.z);
    // Serial.println();
    //  SERIAL_PORT.printf("%d %d %d \n", amgt.acc.axes.x, amgt.acc.axes.y, amgt.acc.axes.z);
  }
}

// Decide when to print
bool readyToPrint()
{
  static unsigned long nowMillis;
  static unsigned long thenMillis;
  // If the Processing visualization sketch is sending "s"
  // then send new data each time it wants to redraw
  // while (Serial.available()) {
  //   int val = Serial.read();
  //   if (val == 's') {
  //     thenMillis = millis();
  //     return true;
  //   }
  // }
  // Otherwise, print 8 times per second, for viewing as
  // scrolling numbers in the Arduino Serial Monitor
  nowMillis = millis();
  if (nowMillis - thenMillis > 20)
  {
    thenMillis = nowMillis;
    return true;
  }
  return false;
}

// Below here are some helper functions to print the data nicely!

uint8_t crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31; // 多项式0x31 (x8+x5+x4+1)
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX() - 20, 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY() + 40, 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ() - 70, 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void vofa(float data[6], uint16_t chanal)
{
  SERIAL_PORT.write((char *)data, chanal * 4);
  char tail[4] = {0x00, 0x00, 0x80, 0x7f};
  SERIAL_PORT.write(tail, 4);
}