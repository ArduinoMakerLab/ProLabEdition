#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include "USBHID.h"
#include "USBCDC.h"
#include <RingBuf.h>
#include "motion.hpp"

#define SERIAL_PORT Serial

Servo servoYaw, servoPitch;
const int servoPinYaw = 15;   // yaw servo GPIO15
const int servoPinPitch = 16; // pitch servo GPIO16
const int laserPin = 17;      // laser GPIO17

#define YAW_ORIGIN_DEGREE (90)
#define YAW_REVERSE (0)
#define PITCH_ORIGIN_DEGREE (160)
#define PITCH_REVERSE (1)

#define CONSTRAINT_SERVO_YAW(a) (a < 0 ? 0 : (a > 180 ? 180 : a))
#define CONSTRAINT_SERVO_PITCH(a) (a < 90 ? 90 : (a > 180 ? 180 : a))
int16_t YawSet = YAW_ORIGIN_DEGREE, PitchSet = PITCH_ORIGIN_DEGREE;
int16_t YawSetLast = YAW_ORIGIN_DEGREE, PitchSetLast = PITCH_ORIGIN_DEGREE;
RingBuf<uint8_t, 256> uartBuff;
motion myMotion(0.01);

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

struct attFrame_T imu;
bool waitInit = false;
float refYaw;
bool laserOpen = false;

esp_now_recv_cb_t cb;

void setup()
{
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };
  // Serial1.begin(115200, SERIAL_8N1, 10, 11);
  // Serial1.onReceive(onSerialReceive);

  WiFi.mode(WIFI_STA); // ESP-NOW need config WIFI-STA mode
  //  ESP-NOW init
  delay(2000);
  if (esp_now_init() != ESP_OK)
  {
    SERIAL_PORT.println("ESP-NOW init error");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  SERIAL_PORT.println(WiFi.macAddress());

  servoYaw.attach(servoPinYaw);
  servoPitch.attach(servoPinPitch);

  servoYaw.write(YawSet);
  servoPitch.write(PitchSet);

  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, HIGH);
}

unsigned long last_update;

// const esp_now_recv_info_t *
void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)
{
  // Serial.println(len);
  // Serial.println();
  for (size_t i = 0; i < data_len; i++)
  {
    uartBuff.push(data[i]);
  }
}

void loop()
{
  unsigned long now = millis();

  //About 10ms per frame
  if (uartBuff.size() >= sizeof(attFrame_T))
  {
    uint8_t head, tail;
    uint8_t frame[50];
    struct attFrame_T pack;

    uartBuff.peek(head, 0);
    uartBuff.peek(tail, sizeof(pack) - 1);
    if (head == 0xA5 && tail == 0x5A)
    {
      // SERIAL_PORT.printf("peek :");
      for (uint16_t ii = 0; ii < sizeof(pack); ii++)
      {
        // uartBuff.peek(head, 0);
        // SERIAL_PORT.printf("%x ", head);
        uartBuff.pop(frame[ii]);
      }
      memcpy(&pack, frame, sizeof(pack));
      uint8_t check = 0;
      check = crc8((uint8_t *)&pack, sizeof(pack) - 2);
      if (check = pack.crc)
      {
        memcpy(&imu, &pack, sizeof(attFrame_T));

        if (!waitInit)
        {
          waitInit = true;
          refYaw = imu.att[2];
        }

        float from_angle = imu.att[2], to_angle = refYaw;
        if (from_angle < 0)
          from_angle += 360.0;
        if (to_angle < 0)
          to_angle += 360.0;

        float delta = to_angle - from_angle;
        if (delta > 180)
        {
          delta -= 360;
        }
        else if (delta < -180)
        {
          delta += 360;
        }

        #if(YAW_REVERSE)
        YawSet = YAW_ORIGIN_DEGREE - delta;
        #else
        YawSet = YAW_ORIGIN_DEGREE + delta;
        #endif
        YawSet = CONSTRAINT_SERVO_YAW(YawSet);

        #if(PITCH_REVERSE)
        PitchSet = PITCH_ORIGIN_DEGREE - (imu.att[0]);
        #else
        PitchSet = PITCH_ORIGIN_DEGREE + (imu.att[0]);
        #endif
        PitchSet = CONSTRAINT_SERVO_PITCH(PitchSet);

        if (imu.att[0] < -80)
        {
          YawSet = 0;
          PitchSet = PITCH_ORIGIN_DEGREE;
          servoYaw.write(YawSet);
          servoPitch.write(PitchSet);
          refYaw = imu.att[2];
        }
        else
        {
          //if (abs(YawSet - YawSetLast) > 1)
          {
            servoYaw.write(YawSet);
            YawSetLast = YawSet;
          }
          //if (abs(PitchSet - PitchSetLast) > 1)
          {
            servoPitch.write(PitchSet);
            PitchSetLast = PitchSet;
          }
        }

        SERIAL_PORT.printf("servo set %d, %d, ref %f %f ,att %f \n", YawSet, PitchSet, refYaw, delta, imu.att[2]);

        Motion_E detect = myMotion.update(imu.att, imu.acc);
        switch (detect)
        {
        case MOTION_NONE:
          break;
        case MOTION_FRONT_BACK:
          digitalWrite(laserPin, LOW);
          break;
        case MOTION_LEFT_RIGHT:
          digitalWrite(laserPin, HIGH);
          break;
        default:
          break;
        }

        // SERIAL_PORT.printf("rx 1 pack %f, %f, %f, %f, ctrl: %d %d\n", refYaw, imu.att[0], imu.att[1], imu.att[2], YawSet, PitchSet);
      }
      else
      {
        // SERIAL_PORT.println("frame error");
        uartBuff.pop(head);
      }
    }
    else
    {
      uartBuff.pop(head);
    }
  }
}

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
        crc = (crc << 1) ^ 0x31; // 0x31 (x8+x5+x4+1)
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}