#ifndef NRF24_COMMUNICATION_H
#define NRF24_COMMUNICATION_H

#include <mbed.h>
#include <SPI.h>                 // Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01P/nRF24L01P.h> // nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <commConfig.h>
#include <commTypes.h>

class nRF24Communication
{
public:
  // Class constructor
  nRF24Communication(PinName pinMOSI, PinName pinMISO, PinName pinSCK, PinName pinCE, PinName pinCSN, PinName pinVCC, NetworkType network, RadioFunction function);

  // Class destructor
  // ~nRF24Communication();

  int setup(int robotSwitches);
  bool resetRadio();
  bool compareChannel(uint8_t channel);
  int updateRobotId(int robotSwIR2_D);
  int getRobotId();
  void printDetails();

  msgType updatePacket();
  int getTypeOfMessage();
  void showBitsReceived(int payload);
  void enable();
  void disable();

  // VSS Info
  void getDifferentialSpeed(Motors &mSpeed);
  void clearVSSData();

  // SSL Info
  void getVectorSpeed(Vector &mSpeed);
  bool sendTelemetryPacket(RobotInfo telemetry);
  bool sendOdometryPacket(RobotInfo odometry);
  void clearSSLData();
  void getKick(KickFlags &isKick);
  void getPosition(RobotPosition &pos);

  // Aux Info
  float getKP();
  float getKD();
  float getAlpha();

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
  packetOdometry _mOdometry;
  packetPosition _mPostion;
  packetGeneric _rx;

  NetworkConfig _config;
  char _robotId;
  msgType _typeMsg;
  Motors _motorSpeed;
  int _flags;
  Vector _v;
  KickFlags _kick;
  RobotPosition _pos;
  float _kp, _kd, _ki, _alpha;
};

#endif // NRF_COMM
