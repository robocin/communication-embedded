
#include "nRF24Communication.h"

nRF24Communication::nRF24Communication(PinName pinMOSI, PinName pinMISO, PinName pinSCK, PinName pinCE, PinName pinCSN, PinName pinVCC, NetworkType network, RadioFunction function)
    : _radio(pinMOSI, pinMISO, pinSCK, pinCSN, pinCE), _vcc(pinVCC)
{
  this->_vcc = 1;

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
    this->_config.receiveChannel = VSS_RECV_CHANNEL;
    this->_config.sendChannel = VSS_SEND_CHANNEL; // Future Telemetry

    this->_config.addr[0] = VSS_ADDR_1;
    this->_config.addr[1] = VSS_ADDR_2;
  }
  else if (network == NetworkType::rl)
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
  this->_radio.setPayloadSize(_config.payload);
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
  this->_radio.stopListening();
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

msgType nRF24Communication::updatePacket()
{
  this->enable();

  if (this->_radio.isChipConnected())
  {
    while (this->_radio.available(&(_config.pipeNum)))
    {
      this->_radio.read(&(this->_rx.encoded), _config.payload);
      // Save the message type
      this->_typeMsg = static_cast<msgType>(this->_rx.decoded.typeMsg);

      if (this->_rx.decoded.id == this->_robotId)
      {

        // According to the type of message assign variables
        if (this->_typeMsg == msgType::VSS_SPEED)
        {
          // VSS
          this->clearVSSData();
          std::memcpy(this->_mVSS.encoded, this->_rx.encoded, VSS_SPEED_LENGTH); //require std::, eventual error in copy
          this->_flags = static_cast<uint8_t>(this->_mVSS.decoded.flags);
          this->_motorSpeed.m1 = static_cast<int8_t>(this->_mVSS.decoded.leftSpeed);
          this->_motorSpeed.m2 = static_cast<int8_t>(this->_mVSS.decoded.rightSpeed);
        }
        else if (this->_typeMsg == msgType::SSL_SPEED)
        {
          // SSL
          this->clearSSLData();
          std::memcpy(this->_mSSL.encoded, this->_rx.encoded, SSL_SPEED_LENGTH); //require std::, eventual error in copy
          this->_v.x = static_cast<double>((this->_mSSL.decoded.vx) / 10000.0);
          this->_v.y = static_cast<double>((this->_mSSL.decoded.vy) / 10000.0);
          this->_v.w = static_cast<double>((this->_mSSL.decoded.vw) / 10000.0);
          this->_kick.front = static_cast<bool>(this->_mSSL.decoded.front);
          this->_kick.chip = static_cast<bool>(this->_mSSL.decoded.chip);
          this->_kick.charge = static_cast<bool>(this->_mSSL.decoded.charge);
          this->_kick.kickStrength = static_cast<float>((this->_mSSL.decoded.strength) / 10.0);
          this->_kick.dribbler = static_cast<bool>(this->_mSSL.decoded.dribbler);
          this->_kick.dribblerSpeed = static_cast<float>((this->_mSSL.decoded.speed) / 10.0);
        }
        else if (this->_typeMsg == msgType::POSITION)
        {
          std::memcpy(this->_mPostion.encoded, this->_rx.encoded, POSITION_LENGTH);
          this->_pos.v.x = static_cast<double>((this->_mPostion.decoded.x) / 1000.0);
          this->_pos.v.y = static_cast<double>((this->_mPostion.decoded.y) / 1000.0);
          this->_pos.v.w = static_cast<double>((this->_mPostion.decoded.w) / 10000.0);
          this->_pos.maxSpeed = static_cast<double>((this->_mPostion.decoded.speed) / 100.0);
          this->_pos.type = static_cast<PositionType>((this->_mPostion.decoded.positionType));
        }
        else
        {
          break;
        }
        this->disable();
        return this->_typeMsg;
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
  return msgType::NONE;
}

bool nRF24Communication::sendTelemetryPacket(RobotInfo telemetry)
{

  this->_mTelemetry.decoded.typeMsg = static_cast<uint8_t>(msgType::TELEMETRY);
  this->_mTelemetry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mTelemetry.decoded.m1 = static_cast<uint8_t>(telemetry.m.m1 * 100);
  this->_mTelemetry.decoded.m2 = static_cast<uint8_t>(telemetry.m.m2 * 100);
  this->_mTelemetry.decoded.m3 = static_cast<uint8_t>(telemetry.m.m3 * 100);
  this->_mTelemetry.decoded.m4 = static_cast<uint8_t>(telemetry.m.m4 * 100);
  this->_mTelemetry.decoded.dribbler = static_cast<uint8_t>(telemetry.dribbler * 10);
  this->_mTelemetry.decoded.kickLoad = static_cast<uint8_t>(telemetry.kickLoad * 100);
  this->_mTelemetry.decoded.ball = static_cast<bool>(telemetry.ball);
  this->_mTelemetry.decoded.battery = static_cast<uint8_t>(telemetry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mTelemetry.encoded, TELEMETRY_LENGTH);
  this->disable();
  return answer;
}

bool nRF24Communication::sendVSSTelemetryPacket(VSSRobotInfo telemetry)
{

  this->_mTelemetryVSS.decoded.typeMsg = static_cast<uint8_t>(msgType::TELEMETRY);
  this->_mTelemetryVSS.decoded.id = static_cast<uint8_t>(telemetry.id);
  this->_mTelemetryVSS.decoded.m1 = static_cast<int8_t>(telemetry.m1 * 10);
  this->_mTelemetryVSS.decoded.m2 = static_cast<int8_t>(telemetry.m2 * 10);
  this->_mTelemetryVSS.decoded.battery = static_cast<uint8_t>(telemetry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mTelemetryVSS.encoded, VSS_TELEMETRY_LENGTH);
  this->disable();
  return answer;
}

bool nRF24Communication::sendOdometryPacket(RobotInfo odometry)
{
  this->_mOdometry.decoded.typeMsg = static_cast<uint8_t>(msgType::ODOMETRY);
  this->_mOdometry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mOdometry.decoded.x = static_cast<int16_t>(odometry.v.x * 1000);
  this->_mOdometry.decoded.y = static_cast<int16_t>(odometry.v.y * 1000);
  this->_mOdometry.decoded.w = static_cast<int16_t>(odometry.v.w * 10000);
  this->_mOdometry.decoded.dribbler = static_cast<uint8_t>(odometry.dribbler * 10);
  this->_mOdometry.decoded.kickLoad = static_cast<uint8_t>(odometry.kickLoad * 100);
  this->_mOdometry.decoded.ball = static_cast<bool>(odometry.ball);
  this->_mOdometry.decoded.battery = static_cast<uint8_t>(odometry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mOdometry.encoded, ODOMETRY_LENGTH);
  this->disable();
  return answer;
}

void nRF24Communication::getDifferentialSpeed(Motors &mSpeed)
{
  mSpeed.m1 = this->_motorSpeed.m1;
  mSpeed.m2 = this->_motorSpeed.m2;
}

void nRF24Communication::clearVSSData()
{
  this->_flags = 0;
  this->_motorSpeed.m1 = 0;
  this->_motorSpeed.m2 = 0;
}

void nRF24Communication::getVectorSpeed(Vector &mSpeed)
{
  mSpeed = this->_v;
}
void nRF24Communication::clearSSLData()
{
  this->_v.x = 0;
  this->_v.y = 0;
  this->_v.w = 0;
  this->_kick.front = false;
  this->_kick.chip = false;
  this->_kick.charge = false;
  this->_kick.kickStrength = 0;
  this->_kick.dribbler = false;
  this->_kick.dribblerSpeed = 0;
}

void nRF24Communication::getPosition(RobotPosition &pos)
{
  pos = _pos;
}

void nRF24Communication::getKick(KickFlags &isKick)
{
  isKick.front = _kick.front;
  isKick.chip = _kick.chip;
  isKick.charge = _kick.charge;
  isKick.kickStrength = _kick.kickStrength;
  isKick.dribbler = _kick.dribbler;
  isKick.dribblerSpeed = _kick.dribblerSpeed;
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
