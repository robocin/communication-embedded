#ifndef NRF_CONFIG_H
#define NRF_CONFIG_H

// v2.3

#include <stdint.h>

#define SSL_1_BASE_SEND_CH 112
#define SSL_2_BASE_RECV_CH 114

#define SSL_1_ROBOT_RECV_CH 112
#define SSL_2_ROBOT_SEND_CH 114

#define SSL_ADDR_1 0x752FAD299ALL
#define SSL_ADDR_2 0x6D4ADC444BLL

#define VSS_CHANNEL 108
#define VSS_ADDR_1 0x752FAF0A9ALL
#define VSS_ADDR_2 0x5D4BFBC2BBLL

#define DEEP_CHANNEL 105
#define DEEP_ADDR_1 0x752FAB239ALL
#define DEEP_ADDR_2 0x5D4ADC454BLL

#define ACK_RADIO 0

#define VSS_PAYLOAD_LENGTH 10
#define VSS_CONTROL_LENGTH 4

#define SSL_PAYLOAD_LENGTH 15 
#define SSL_CONTROL_LENGTH 12
#define SSL_TELEMETRY_LENGTH 13

enum MESSAGE_TYPE
{
  msgType_VSS_SPEED = 0,
  msgType_SSL_SPEED,
  msgType_SSL_TELEMTRY
};

typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int64_t rest_a : 64;
  int64_t rest_b : 48;
} __attribute__((packed)) packetTypeGeneric;

typedef union {
  unsigned char encoded[SSL_PAYLOAD_LENGTH];
  packetTypeGeneric decoded;
} packetGeneric;

/**
 * Structure for sending speeds to bi-directional robot,
 * This type sends:
 *  - Message type
 *  - Robot Id
 *  - The left and right motor speeds
 *  - One byte of free for optional flags.
 */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int8_t leftSpeed : 8;
  int8_t rightSpeed : 8;
  uint8_t flags : 8;
} __attribute__((packed)) packetTypeVSS;

typedef union packetVSS {
  unsigned char encoded[VSS_CONTROL_LENGTH];
  packetTypeVSS decoded;
} packetVSS;

/*
  * Structure for sending speeds omni-directional robot,
 * This type sends:
 *  - Message type
 *  - Robot Id
 *  - Vx, Vy and Vw of the robot.
 *  - Type and Strength of Kick.
 *  - Dribbler flag and its speed.
  */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int32_t vx : 20;
  int32_t vy : 20;
  int32_t w : 20;
  uint8_t front : 1;
  uint8_t chip : 1;
  uint8_t charge : 1;
  uint8_t strength : 8;
  uint8_t dribbler : 1;
  uint8_t speed : 8;
  uint8_t command : 8;

} __attribute__((packed)) packetTypeSSL;

typedef union packetSSL {
  unsigned char encoded[SSL_CONTROL_LENGTH];
  packetTypeSSL decoded;
} packetSSL;

typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int16_t m1 : 16;
  int16_t m2 : 16;
  int16_t m3 : 16;
  int16_t m4 : 16;
  int16_t dribbler : 15;
  uint8_t kickLoad : 8;
  bool ball : 1;
  uint8_t battery : 8;

} __attribute__((packed)) packetTypeTelemetrySSL;

typedef union packetSSLTelemetry {
  unsigned char encoded[SSL_TELEMETRY_LENGTH];
  packetTypeTelemetrySSL decoded;
} packetTelemetrySSL;

#endif // NRF_CONFIG_H
