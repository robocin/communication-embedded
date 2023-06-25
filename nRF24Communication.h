#ifndef NRF24_COMMUNICATION_H
#define NRF24_COMMUNICATION_H

#include <mbed.h>
#include <SPI.h>                 // Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01P/nRF24L01P.h> // nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <commConfig.h>
#include <commTypes.h>
#include "utils.h"

class nRF24Communication {
 public:
  // Class constructor
  nRF24Communication(PinName pinMOSI,
                     PinName pinMISO,
                     PinName pinSCK,
                     PinName pinCE,
                     PinName pinCSN,
                     PinName pinVCC,
                     NetworkType network,
                     RadioFunction function);

  // Class destructor
  // ~nRF24Communication();

  int setup(int robotSwitches);
  bool resetRadio();
  bool compareChannel(uint8_t channel);
  int updateRobotId(int robotSwIR2_D);
  int getRobotId();
  void printDetails();
  bool radioStillConfigured();

  bool updatePacket();
  msgType getPacketType();
  void showBitsReceived(int payload, unsigned char* data);
  void enable();
  void disable();

  // VSS Info
  void getDifferentialSpeed(Motors& mSpeed);
  void clearVSSData();

  // SSL Info
  Vector getVectorSpeed();
  bool sendTelemetryPacket(RobotInfo telemetry);
  bool sendOdometryPacket(RobotInfo odometry);
  bool sendVSSTelemetryPacket(VSSRobotInfo telemetryVSS);
  void clearSSLDataSpeed();
  void clearSSLDataPosition();
  void clearSSLDataKick();
  void getKick(KickFlags& isKick);
  void getPosition(RobotPosition& pos);
  refereeCommand getGameState();
  RobotPosition getLastPosition();
  bool robotMoveIsLocked();
  bool robotMoveCriticalTurbo();

  // Aux Info
  float getKP();
  float getKD();
  float getAlpha();

  /******** CREATED PUBLIC RX FOR ETHERNET CONFIG ***********/
  packetGeneric _public_rx;

 private:
  RF24 _radio;
  DigitalOut _vcc;

 private:
  void _network(NetworkType network);
  void _configure();
  void _resetRadio();
  void _receive();
  void _send();

  packetSpeedVSS _mVSS;
  packetSpeedSSL _mSSL;
  packetTelemetry _mTelemetry;
  packetVSSTelemetry _mTelemetryVSS;
  packetOdometry _mOdometry;
  packetPosition _mPostion;
  packetGeneric _rx;

  NetworkConfig _config;
  char _robotId;
  msgType _typeMsg;
  refereeCommand _gameState;
  msgType _lastPacketType;
  Motors _motorSpeed;
  bool _isPWM;
  Vector _v;
  bool _moveIsLocked;
  bool _criticalMoveTurbo;
  KickFlags _kick;
  RobotPosition _pos;
  float _kp, _kd, _ki, _alpha;
};

#endif // NRF_COMM
