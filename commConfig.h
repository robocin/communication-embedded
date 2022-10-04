#ifndef COMM_CONFIG_H
#define COMM_CONFIG_H

// v3.1

#include <stdint.h>


// NRF Config Defitions
#define SSL_1_BASE_SEND_CH 112
#define SSL_2_BASE_RECV_CH 114

#define SSL_1_ROBOT_RECV_CH 112
#define SSL_2_ROBOT_SEND_CH 114

#define SSL_ADDR_1 0x752FAD299ALL
#define SSL_ADDR_2 0x6D4ADC444BLL

#define VSS_BASE_RECV_CHANNEL 110
#define VSS_BASE_SEND_CHANNEL 108

#define VSS_ROBOT_RECV_CHANNEL 108
#define VSS_ROBOT_SEND_CHANNEL 110

#define VSS_ADDR_1 0x752FAF0A9ALL
#define VSS_ADDR_2 0x5D4BFBC2BBLL

#define ACK_RADIO 0
#define NRF_MAX_PAYLOAD 32

// PAYLOAD DEFINITIONS
#define BST_CONFIG_LENGTH 21

#define VSS_PAYLOAD_LENGTH 7
#define VSS_SPEED_LENGTH VSS_PAYLOAD_LENGTH
#define VSS_TELEMETRY_LENGTH VSS_PAYLOAD_LENGTH

#define SSL_PAYLOAD_LENGTH 20 //15 //
#define SSL_SPEED_LENGTH 20   //12 //
#define POSITION_LENGTH 20    // 9 //
#define TELEMETRY_LENGTH 20   //13 //
#define ODOMETRY_LENGTH 20    //11 //

#pragma pack (push, 1)

enum class msgType
{
  NONE = -1,
  BST_CONFIG,
  VSS_SPEED,
  SSL_SPEED,
  TELEMETRY,
  ODOMETRY,
  POSITION
};

typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int64_t rest_a : 64;
  int64_t rest_b : 64;
  int64_t rest_c : 64;
  int64_t rest_d : 56;
} packetTypeGeneric;

typedef union {
  unsigned char encoded[NRF_MAX_PAYLOAD];
  packetTypeGeneric decoded;
} packetGeneric;

/**
 * Structure to configure the Base Station,
 * This type sends:
 *  - Message type
 *  - Duplex communication?
 *  - Team (follow NetworkType)
 *  - nRF Address 1
 *  - nRF Address 2
 *  - Payload of nRF messages
 *  - Channel of nRF send.
 *  - Channel of nRF recv.
 */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  bool duplex: 1;
  uint8_t team: 4;
  uint64_t addr1: 64;
  uint64_t addr2: 64;
  uint8_t payload: 8;
  uint8_t channel1: 8;
  uint8_t channel2: 8;
  uint8_t free: 3;

} packetTypeBStConfig;

typedef union
{
  unsigned char encoded[BST_CONFIG_LENGTH];
  packetTypeBStConfig decoded;
} packetBStConfig;

/**
 * Structure for sending speeds to bi-directional robot,
 * This type sends:
 *  - Message type
 *  - Robot Id
 *  - The left and right motor speeds
 *  - 
 *  - One byte of free for optional flags.
 */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int32_t m1 : 18;   // (-131.072 <-> 131.071 rad/s or -1.31072 <-> 1.31072 pwm) (clamp(-1.00000, 1.00000) // left motor speed
  int32_t m2 : 18;   // (-132.072 <-> 131.071 rad/s or -1.31072 <-> 1.31072 pwm) (clamp(-1.00000, 1.00000) // right motor speed
  bool isPWM : 1;    // Bit indication for speed type (00 -> rad/s, 01 -> pwm)
  uint16_t free : 11;
  

} packetTypeSpeedVSS;

typedef union packetSpeedVSS {
  unsigned char encoded[VSS_SPEED_LENGTH];
  packetTypeSpeedVSS decoded;
} packetSpeedVSS;

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
  int32_t vw : 20;
  uint8_t front : 1;
  uint8_t chip : 1;
  uint8_t charge : 1;
  uint8_t strength : 8;
  uint8_t dribbler : 1;
  uint8_t speed : 8;
  uint8_t command : 8;
  uint64_t free_1 : 64;

} packetTypeSpeedSSL;

typedef union packetSpeedSSL {
  unsigned char encoded[SSL_SPEED_LENGTH];
  packetTypeSpeedSSL decoded;
} packetSpeedSSL;

/*
  * Structure for sending position to a robot,
 * This type sends:
 *  - Message type
 *  - Robot Id
 *  - x, y and w of the robot.
 *  - Source or Destiny position.
 *  - ....
  */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int16_t x : 16; // -32.767 - 32.767 m
  int16_t y : 16; // -32.767 - 32.767 m
  int16_t w : 16; // 0 - 6.5535 rad
  uint16_t maxSpeed : 13; // 0 - 81.91 m/s
  uint16_t minSpeed : 13; // 0 - 8.191 m/s
  uint8_t positionType : 3;
  // Kick Options
  uint8_t front : 1;
  uint8_t chip : 1;
  uint8_t charge : 1;
  uint8_t strength : 8;
  uint8_t dribbler : 1;
  uint8_t speed : 8;
  uint8_t command : 8;
  uint64_t free_1 : 60;
} packetTypePosition;

typedef union packetPosition
{
  unsigned char encoded[POSITION_LENGTH];
  packetTypePosition decoded;
} packetPosition;

typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int16_t m1 : 16; // -327.67 - 327.67 m/s
  int16_t m2 : 16; // -327.67 - 327.67 m/s
  int16_t m3 : 16; // -327.67 - 327.67 m/s
  int16_t m4 : 16; // -327.67 - 327.67 m/s
  int16_t dribbler : 15;  // -1638.3 - 1638.3 rad/s
  uint8_t kickLoad : 8; // 0 - 2.55
  bool ball : 1;
  uint8_t battery : 8; // 0 - 25.5 V
  uint64_t free_1 : 56;

} packetTypeTelemetry;

typedef union packetTelemetry {
  unsigned char encoded[TELEMETRY_LENGTH];
  packetTypeTelemetry decoded;
} packetTelemetry;

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
  int32_t  m1 : 18;
  int32_t  m2 : 18;  
  uint8_t battery : 8;  // 0 - 12.8 V
  uint8_t free : 4;

} packetTypeVSSTelemetry;

typedef union packetVSSTelemetry {
  unsigned char encoded[VSS_TELEMETRY_LENGTH];
  packetTypeVSSTelemetry decoded;
} packetVSSTelemetry;

/*
  * Structure for send robot basic status and position,
 * This type sends:
 *  - Message type
 *  - Robot Id
 *  - X, Y and W of the robot.
 *  - Dribbler speed and its speed.
 *  - Kick capacitor load.
 *  - Ball on robot?
 *  - Battery load.
  */
typedef struct
{
  uint8_t typeMsg : 4;
  uint8_t id : 4;
  int16_t x : 16; // -32.767 - 32.767 m
  int16_t y : 16; // -32.767 - 32.767 m
  int16_t w : 16; // 0 - 6.5535 rad
  int16_t dribbler : 15; // -1638.3 - 1638.3 rad/s
  uint8_t kickLoad : 8; // 0 - 2.55
  bool ball : 1;
  uint8_t battery : 8; // 0 - 25.5 V
  int16_t m1 : 16; // -327.67 - 327.67 m/s
  int16_t m2 : 16; // -327.67 - 327.67 m/s
  int16_t m3 : 16; // -327.67 - 327.67 m/s
  int16_t m4 : 16; // -327.67 - 327.67 m/s
  uint8_t free_1 : 8;

} packetTypeOdometry;

typedef union packetOdometry
{
  unsigned char encoded[ODOMETRY_LENGTH];
  packetTypeOdometry decoded;
} packetOdometry;

//restoring the standard alignment
#pragma pack(pop)

#endif // COMM_CONFIG_H
