
#include "nRF24Communication.h"

nRF24Communication::nRF24Communication(PinName pinMOSI, PinName pinMISO, PinName pinSCK, PinName pinCE, PinName pinCSN, PinName pinVCC, NetworkType network, RadioFunction function)
    : _radio(pinMOSI, pinMISO, pinSCK, pinCSN, pinCE), _vcc(pinVCC)
{
  this->_vcc = 1;

  // VSS
  this->_leftMotorSpeed = 0;
  this->_rightMotorSpeed = 0;

  // SSL
  this->_front = false;
  this->_chip = false;
  this->_charge = false;
  this->_dribbler = false;

  this->_config.function = function;
  this->_network(network);
  this->disable();
}

int nRF24Communication::setup(int robotSwitches)
{
  if (this->updateRobotId(robotSwitches) < 0)
  {
    return -1;
  }
  this->enable();
  this->_configure();
  this->disable();
  return 0;
}

bool nRF24Communication::compareChannel(uint8_t channel)
{
  return this->_radio.compareChannel(channel);
}

void nRF24Communication::_network(NetworkType network)
{
  this->_config.pipeNum = 0;
  this->_config.reConfig = false;
  this->_config.ack = ACK_RADIO;

  if (network == NetworkType::ssl)
  {
    this->_config.payload = SSL_PAYLOAD_LENGTH;
    this->_config.receiveChannel = SSL_1_ROBOT_RECV_CH;
    this->_config.sendChannel = SSL_2_ROBOT_SEND_CH;

    this->_config.addr[0] = SSL_ADDR_1;
    this->_config.addr[1] = SSL_ADDR_2;
  }
  else if (network == NetworkType::vss)
  {
    this->_config.payload = VSS_PAYLOAD_LENGTH;
    this->_config.receiveChannel = VSS_CHANNEL;
    this->_config.sendChannel = VSS_CHANNEL; // Future Telemetry

    this->_config.addr[0] = VSS_ADDR_1;
    this->_config.addr[1] = VSS_ADDR_2;
  }
  else if (network == NetworkType::deep)
  {
    this->_config.payload = VSS_PAYLOAD_LENGTH;
    this->_config.receiveChannel = DEEP_CHANNEL;
    this->_config.sendChannel = DEEP_CHANNEL; // Future Telemetry

    this->_config.addr[0] = DEEP_ADDR_1;
    this->_config.addr[1] = DEEP_ADDR_2;
  }
}

void nRF24Communication::_configure()
{
  this->_radio.begin(); // Start the nRF24 module
  this->_radio.setPALevel(RF24_PA_MAX);
  this->_radio.setDataRate(RF24_2MBPS);
  this->_radio.setAutoAck(_config.ack);
  this->_radio.setPayloadSize(_config.payload); // Its possible to set Dynamic PayLoad Size.
  if (_config.function == RadioFunction::receiver)
  {
    // Init communication as receiver
    this->_radio.openWritingPipe(_config.addr[1]);
    this->_radio.openReadingPipe(1, _config.addr[0]);
    this->_radio.setChannel(this->_config.receiveChannel);
    this->_radio.startListening();
  }
  else if (_config.function == RadioFunction::sender)
  {
    this->_radio.openWritingPipe(_config.addr[0]);
    this->_radio.openReadingPipe(1, _config.addr[1]);
    this->_radio.setChannel(this->_config.sendChannel);
    this->_radio.stopListening();
  }
  //this->_radio.printDetails(); // Radio Details
}

void nRF24Communication::_resetRadio()
{
  this->_vcc = 0;
  wait_us(100);
  this->_vcc = 1;
  wait_us(50);
}

void nRF24Communication::_receive()
{
  this->_radio.startListening();
}

void nRF24Communication::_send()
{
  this->_radio.stopListening(); // Stop listening for messages
}

void nRF24Communication::enable()
{
  this->_radio.csn(LOW);
  wait_us(5);
}

void nRF24Communication::disable()
{
  this->_radio.csn(HIGH);
  wait_us(5);
}

int nRF24Communication::updateRobotId(int robotSwID)
{
  if (robotSwID < 0 || robotSwID > 15)
    return -1;
  else
    this->_robotId = robotSwID;
  return this->_robotId;
}

int nRF24Communication::getRobotId()
{
  return this->_robotId;
}

void nRF24Communication::printDetails()
{
  this->_radio.printDetails();
}

int nRF24Communication::getTypeOfMessage()
{ // Returning type of message received
  return this->_rx.decoded.typeMsg;
}

bool nRF24Communication::sendTelemetryPacket(RobotTelemetry telemetry)
{

  this->_mSSLTelemetry.decoded.typeMsg = msgType_SSL_TELEMTRY;
  this->_mSSLTelemetry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mSSLTelemetry.decoded.m1 = static_cast<uint8_t>(telemetry.m.m1 * 100);
  this->_mSSLTelemetry.decoded.m2 = static_cast<uint8_t>(telemetry.m.m2 * 100);
  this->_mSSLTelemetry.decoded.m3 = static_cast<uint8_t>(telemetry.m.m3 * 100);
  this->_mSSLTelemetry.decoded.m4 = static_cast<uint8_t>(telemetry.m.m4 * 100);
  this->_mSSLTelemetry.decoded.dribbler = static_cast<uint8_t>(telemetry.dribbler * 10);
  this->_mSSLTelemetry.decoded.kickLoad = static_cast<uint8_t>(telemetry.kickLoad * 100);
  this->_mSSLTelemetry.decoded.ball = static_cast<bool>(telemetry.ball);
  this->_mSSLTelemetry.decoded.battery = static_cast<uint8_t>(telemetry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mSSLTelemetry.encoded, SSL_TELEMETRY_LENGTH);
  this->disable();
  return answer;
}

bool nRF24Communication::sendOdometryPacket(RobotOdometry odometry)
{

  this->_mSSLOdometry.decoded.typeMsg = msgType_SSL_ODOMETRY;
  this->_mSSLOdometry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mSSLOdometry.decoded.x = static_cast<int16_t>(odometry.v.x * 1000);
  this->_mSSLOdometry.decoded.y = static_cast<int16_t>(odometry.v.y * 1000);
  this->_mSSLOdometry.decoded.w = static_cast<int16_t>(odometry.v.w * 10000);
  this->_mSSLOdometry.decoded.dribbler = static_cast<uint8_t>(odometry.dribbler * 10);
  this->_mSSLOdometry.decoded.kickLoad = static_cast<uint8_t>(odometry.kickLoad * 100);
  this->_mSSLOdometry.decoded.ball = static_cast<bool>(odometry.ball);
  this->_mSSLOdometry.decoded.battery = static_cast<uint8_t>(odometry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mSSLOdometry.encoded, SSL_ODOMETRY_LENGTH);
  this->disable();
  return answer;
}

bool nRF24Communication::updatePacket(bool reconnect)
{
  this->enable();
  if (reconnect)
  {
    if (!this->compareChannel())
    {
      this->_resetRadio();
      this->_configure();
    }
  }
  if (this->_radio.isChipConnected())
  {
    while (this->_radio.available(&(_config.pipeNum)))
    {
      this->_radio.read(&(this->_rx.encoded), SSL_PAYLOAD_LENGTH);
      this->_typeMsg = static_cast<uint8_t>(this->_rx.decoded.typeMsg); // Save the message type

      if (this->_rx.decoded.id == this->_robotId)
      {

        this->clearVSSData();
        this->clearSSLData();

        // According to the type of message I assign certain variables
        if (this->_typeMsg == msgType_VSS_SPEED)
        {
          // VSS
          std::memcpy(this->_mVSS.encoded, this->_rx.encoded, VSS_CONTROL_LENGTH); //require std::, eventual error in copy
          this->_flags = static_cast<uint8_t>(this->_mVSS.decoded.flags);
          this->_leftMotorSpeed = static_cast<int8_t>(this->_mVSS.decoded.leftSpeed);
          this->_rightMotorSpeed = static_cast<int8_t>(this->_mVSS.decoded.rightSpeed);
        }
        else if (this->_typeMsg == msgType_SSL_SPEED)
        {
          // SSL
          std::memcpy(this->_mSSL.encoded, this->_rx.encoded, SSL_CONTROL_LENGTH); //require std::, eventual error in copy
          this->_vx = static_cast<double>((this->_mSSL.decoded.vx) / 10000.0);
          this->_vy = static_cast<double>((this->_mSSL.decoded.vy) / 10000.0);
          this->_w = static_cast<double>((this->_mSSL.decoded.w) / 10000.0);
          this->_front = static_cast<bool>(this->_mSSL.decoded.front);
          this->_chip = static_cast<bool>(this->_mSSL.decoded.chip);
          this->_charge = static_cast<bool>(this->_mSSL.decoded.charge);
          this->_kickStrength = static_cast<float>((this->_mSSL.decoded.strength) / 10.0);
          this->_dribbler = static_cast<bool>(this->_mSSL.decoded.dribbler);
          this->_dribblerSpeed = static_cast<float>((this->_mSSL.decoded.speed) / 10.0);
        }
        else
        {
          break;
        }
        this->disable();
        return true;
      }
      else
      {
        break;
      }
    }
  }
  else
  {
    this->_resetRadio();
    this->_configure();
  }
  this->disable();
  return false;
}

int nRF24Communication::getLeftMotorSpeed()
{
  return this->_leftMotorSpeed;
}

int nRF24Communication::getRightMotorSpeed()
{
  return this->_rightMotorSpeed;
}

void nRF24Communication::clearVSSData()
{
  this->_flags = 0;
  this->_leftMotorSpeed = 0;
  this->_rightMotorSpeed = 0;
}

void nRF24Communication::getRobotVectorSpeed(Vector &mSpeed)
{
  mSpeed.x = this->getVx();
  mSpeed.y = this->getVy();
  mSpeed.w = this->getW();
}

void nRF24Communication::clearSSLData()
{
  this->_vx = 0;
  this->_vy = 0;
  this->_w = 0;
  this->_front = false;
  this->_chip = false;
  this->_charge = false;
  this->_kickStrength = 0;
  this->_dribbler = false;
  this->_dribblerSpeed = 0;
}

double nRF24Communication::getVx()
{
  return this->_vx;
}
double nRF24Communication::getVy()
{
  return this->_vy;
}
double nRF24Communication::getW()
{
  return this->_w;
}

void nRF24Communication::getKick(KickFlags &isKick)
{
  isKick.front = this->getFront();
  isKick.chip = this->getChip();
  isKick.charge = this->getCharge();
  isKick.kickStrength = this->getKickStrength();
  isKick.dribbler = this->getDribbler();
  isKick.dribblerSpeed = this->getDribblerSpeed();
}

bool nRF24Communication::getFront()
{
  return this->_front;
}
bool nRF24Communication::getChip()
{
  return this->_chip;
}
bool nRF24Communication::getCharge()
{
  return this->_charge;
}
float nRF24Communication::getKickStrength()
{
  return this->_kickStrength;
}
bool nRF24Communication::getDribbler()
{
  return this->_dribbler;
}
float nRF24Communication::getDribblerSpeed()
{
  return this->_dribblerSpeed;
}

float nRF24Communication::getKP()
{
  return _kp;
}
float nRF24Communication::getKD()
{
  return _kd;
}
float nRF24Communication::getAlpha()
{
  return _alpha;
}
