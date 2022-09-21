#include "nRF24Communication.h"
#include "commTypes.h"

nRF24Communication::nRF24Communication(PinName pinMOSI, PinName pinMISO,
                                       PinName pinSCK, PinName pinCE,
                                       PinName pinCSN, PinName pinVCC,
                                       NetworkType network,
                                       RadioFunction function)
    : _radio(pinMOSI, pinMISO, pinSCK, pinCSN, pinCE), _vcc(pinVCC) {
  this->_vcc = 1;

  this->_config.function = function;
  this->_network(network);
  this->disable();
}

int nRF24Communication::setup(int robotSwitches) {
  if (this->updateRobotId(robotSwitches) < 0) {
    return -1;
  }
  this->enable();
  this->_configure();
  this->disable();
  return 0;
}

bool nRF24Communication::resetRadio() {
  this->enable();
  this->_resetRadio();
  wait_us(1000);
  this->_configure();
  bool connected = this->_radio.getPALevel() == RF24_PA_MAX;
  this->disable();
  return connected;
}

bool nRF24Communication::compareChannel(uint8_t channel) {
  return this->_radio.compareChannel(channel);
}

void nRF24Communication::_network(NetworkType network) {
  this->_config.pipeNum = 0;
  this->_config.reConfig = false;
  this->_config.ack = ACK_RADIO;

  if (network == NetworkType::ssl) {
    this->_config.payload = SSL_PAYLOAD_LENGTH;
    this->_config.receiveChannel = SSL_1_ROBOT_RECV_CH;
    this->_config.sendChannel = SSL_2_ROBOT_SEND_CH;

    this->_config.addr[0] = SSL_ADDR_1;
    this->_config.addr[1] = SSL_ADDR_2;
  } else if (network == NetworkType::vss) {
    this->_config.payload = VSS_PAYLOAD_LENGTH;
    this->_config.receiveChannel = VSS_CHANNEL;
    this->_config.sendChannel = VSS_CHANNEL; // Future Telemetry

    this->_config.addr[0] = VSS_ADDR_1;
    this->_config.addr[1] = VSS_ADDR_2;
  } else if (network == NetworkType::rl) {
    this->_config.payload = VSS_PAYLOAD_LENGTH;
    this->_config.receiveChannel = DEEP_CHANNEL;
    this->_config.sendChannel = DEEP_CHANNEL; // Future Telemetry

    this->_config.addr[0] = DEEP_ADDR_1;
    this->_config.addr[1] = DEEP_ADDR_2;
  }
}

void nRF24Communication::_configure() {
  this->_radio.begin(); // Start the nRF24 module
  this->_radio.setPALevel(RF24_PA_MAX);
  this->_radio.setDataRate(RF24_2MBPS);
  this->_radio.setAutoAck(_config.ack);
  this->_radio.setPayloadSize(_config.payload);
  if (_config.function == RadioFunction::receiver) {
    // Init communication as receiver
    this->_radio.openWritingPipe(_config.addr[1]);
    this->_radio.openReadingPipe(1, _config.addr[0]);
    this->_radio.setChannel(this->_config.receiveChannel);
    this->_radio.startListening();
  } else if (_config.function == RadioFunction::sender) {
    this->_radio.openWritingPipe(_config.addr[0]);
    this->_radio.openReadingPipe(1, _config.addr[1]);
    this->_radio.setChannel(this->_config.sendChannel);
    this->_radio.stopListening();
  }
  // this->_radio.printDetails(); // Radio Details
}

void nRF24Communication::_resetRadio() {
  this->_vcc = 0;
  wait_us(1000);
  this->_vcc = 1;
  wait_us(1000);
}

void nRF24Communication::_receive() { this->_radio.startListening(); }

void nRF24Communication::_send() { this->_radio.stopListening(); }

void nRF24Communication::enable() {
  this->_radio.csn(LOW);
  wait_us(5);
}

void nRF24Communication::disable() {
  this->_radio.csn(HIGH);
  wait_us(5);
}

int nRF24Communication::updateRobotId(int robotSwID) {
  if (robotSwID < 0 || robotSwID > 15)
    return -1;
  else
    this->_robotId = robotSwID;
  return this->_robotId;
}

int nRF24Communication::getRobotId() { return this->_robotId; }

void nRF24Communication::printDetails() { this->_radio.printDetails(); }

msgType nRF24Communication::updatePacket() {
  this->enable();
  if (this->_radio.getPALevel() == RF24_PA_MAX) {
    while (this->_radio.available(&(_config.pipeNum))) {
      this->_radio.read(&(this->_rx.encoded), _config.payload);
      // this->showBitsReceived(_config.payload);
      //  Save the message type
      this->_typeMsg = static_cast<msgType>(this->_rx.decoded.typeMsg);

      if (this->_rx.decoded.id == this->_robotId) {
        // According to the type of message assign variables
        if (this->_typeMsg == msgType::VSS_SPEED) {
          // VSS
          this->clearVSSData();
          std::memcpy(
              this->_mVSS.encoded, this->_rx.encoded,
              VSS_SPEED_LENGTH); // require std::, eventual error in copy
          this->_flags = static_cast<uint8_t>(this->_mVSS.decoded.flags);
          this->_motorSpeed.m1 =
              static_cast<int8_t>(this->_mVSS.decoded.leftSpeed);
          this->_motorSpeed.m2 =
              static_cast<int8_t>(this->_mVSS.decoded.rightSpeed);
        } else if (this->_typeMsg == msgType::SSL_SPEED) {
          // SSL
          this->clearSSLData();
          std::memcpy(
              this->_mSSL.encoded, this->_rx.encoded,
              SSL_SPEED_LENGTH); // require std::, eventual error in copy
          this->_gameState = static_cast<Command>(this->_mSSL.decoded.command);
          this->_v.x = static_cast<double>((this->_mSSL.decoded.vx) / 10000.0);
          this->_v.y = static_cast<double>((this->_mSSL.decoded.vy) / 10000.0);
          this->_v.w = static_cast<double>((this->_mSSL.decoded.vw) / 10000.0);
          this->_kick.front = static_cast<bool>(this->_mSSL.decoded.front);
          this->_kick.chip = static_cast<bool>(this->_mSSL.decoded.chip);
          this->_kick.charge = static_cast<bool>(this->_mSSL.decoded.charge);
          this->_kick.kickStrength =
              static_cast<float>((this->_mSSL.decoded.kickStrength) / 10.0);
          this->_kick.dribbler =
              static_cast<bool>(this->_mSSL.decoded.dribbler);
          this->_kick.dribblerSpeed =
              static_cast<float>((this->_mSSL.decoded.dribblerSpeed) / 10.0);
        } else if (this->_typeMsg == msgType::POSITION) {
          this->clearSSLData();
          std::memcpy(this->_mPostion.encoded, this->_rx.encoded,
                      POSITION_LENGTH);
          this->_pos.v.x =
              static_cast<double>((this->_mPostion.decoded.x) / 1000.0);
          this->_pos.v.y =
              static_cast<double>((this->_mPostion.decoded.y) / 1000.0);
          this->_pos.v.w =
              static_cast<double>((this->_mPostion.decoded.w) / 10000.0);
          this->_pos.minSpeed =
              static_cast<double>((this->_mPostion.decoded.minSpeed) / 1000.0);
          this->_pos.maxSpeed =
              static_cast<double>((this->_mPostion.decoded.maxSpeed) / 100.0);
          this->_pos.type =
              static_cast<PositionType>((this->_mPostion.decoded.positionType));
          this->_kick.front = static_cast<bool>(this->_mPostion.decoded.front);
          this->_kick.chip = static_cast<bool>(this->_mPostion.decoded.chip);
          this->_kick.charge =
              static_cast<bool>(this->_mPostion.decoded.charge);
          this->_kick.kickStrength =
              static_cast<float>((this->_mPostion.decoded.kickStrength) / 10.0);
          this->_kick.dribbler =
              static_cast<bool>(this->_mPostion.decoded.dribbler);
          this->_kick.dribblerSpeed =
              static_cast<float>((this->_mPostion.decoded.dribSpeed) / 10.0);
        } else {
          break;
        }
        this->disable();
        return this->_typeMsg;
      } else {
        break;
      }
    }
  } else {
    this->_resetRadio();
    this->_configure();
  }
  this->disable();
  return msgType::NONE;
}

msgType nRF24Communication::updatePacket(protoPositionSSL protomessage) {
  // Return message type
  this->clearSSLData();

  this->_typeMsg = msgType::POSITION;
  this->_pos.v.x = protomessage.x;
  this->_pos.v.y = protomessage.y;
  this->_pos.v.w = protomessage.w;
  this->_pos.resetOdometry = protomessage.resetOdometry;
  this->_pos.maxSpeed = protomessage.max_speed;
  this->_pos.minSpeed = protomessage.min_speed;
  this->_pos.type = static_cast<PositionType>(protomessage.posType);
  this->_kick.front = static_cast<bool>(protomessage.front);
  this->_kick.chip = static_cast<bool>(protomessage.chip);
  this->_kick.charge = static_cast<bool>(protomessage.charge);
  this->_kick.kickStrength = protomessage.kickStrength / 10;
  this->_kick.dribbler = static_cast<bool>(protomessage.dribbler);
  this->_kick.dribblerSpeed = protomessage.dribSpeed / 10;

  return this->_typeMsg;
}

bool nRF24Communication::sendTelemetryPacket(RobotInfo telemetry) {

  this->_mTelemetry.decoded.typeMsg = static_cast<uint8_t>(msgType::TELEMETRY);
  this->_mTelemetry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mTelemetry.decoded.m1 = static_cast<int16_t>(telemetry.m.m1 * 100);
  this->_mTelemetry.decoded.m2 = static_cast<int16_t>(telemetry.m.m2 * 100);
  this->_mTelemetry.decoded.m3 = static_cast<int16_t>(telemetry.m.m3 * 100);
  this->_mTelemetry.decoded.m4 = static_cast<int16_t>(telemetry.m.m4 * 100);
  this->_mTelemetry.decoded.dribbler =
      static_cast<int16_t>(telemetry.dribbler * 10);
  this->_mTelemetry.decoded.kickLoad =
      static_cast<uint8_t>(telemetry.kickLoad * 100);
  this->_mTelemetry.decoded.ball = static_cast<bool>(telemetry.ball);
  this->_mTelemetry.decoded.battery =
      static_cast<uint8_t>(telemetry.battery * 10);
  this->enable();
  bool answer = this->_radio.write(this->_mTelemetry.encoded, TELEMETRY_LENGTH);
  this->disable();
  return answer;
}

bool nRF24Communication::sendOdometryPacket(RobotInfo odometry) {
  this->_mOdometry.decoded.typeMsg = static_cast<uint8_t>(msgType::ODOMETRY);
  this->_mOdometry.decoded.id = static_cast<uint8_t>(this->getRobotId());
  this->_mOdometry.decoded.x = static_cast<int16_t>(odometry.v.x * 1000);
  this->_mOdometry.decoded.y = static_cast<int16_t>(odometry.v.y * 1000);
  this->_mOdometry.decoded.w = static_cast<int16_t>(odometry.v.w * 10000);
  this->_mOdometry.decoded.dribbler =
      static_cast<uint16_t>(odometry.dribbler * 10);
  this->_mOdometry.decoded.kickLoad =
      static_cast<uint8_t>(odometry.kickLoad * 100);
  this->_mOdometry.decoded.ball = static_cast<bool>(odometry.ball);
  this->_mOdometry.decoded.battery =
      static_cast<uint8_t>(odometry.battery * 10);
  this->_mOdometry.decoded.m1 = static_cast<int16_t>(odometry.m.m1 * 100);
  this->_mOdometry.decoded.m2 = static_cast<int16_t>(odometry.m.m2 * 100);
  this->_mOdometry.decoded.m3 = static_cast<int16_t>(odometry.m.m3 * 100);
  this->_mOdometry.decoded.m4 = static_cast<int16_t>(odometry.m.m4 * 100);
  this->_mOdometry.decoded.pcktCount = static_cast<uint8_t>(odometry.count);
  this->enable();
  bool answer = this->_radio.write(this->_mOdometry.encoded, ODOMETRY_LENGTH);
  this->disable();
  return answer;
}

void nRF24Communication::getDifferentialSpeed(Motors &mSpeed) {
  mSpeed.m1 = this->_motorSpeed.m1;
  mSpeed.m2 = this->_motorSpeed.m2;
}

void nRF24Communication::clearVSSData() {
  this->_flags = 0;
  this->_motorSpeed.m1 = 0;
  this->_motorSpeed.m2 = 0;
}

Command nRF24Communication::getGameState() {
  if (this->_pos.type != PositionType::unknown) {
    return Command::forceStart;
  } else {
    return Command::halt;
  }
}

void nRF24Communication::getVectorSpeed(Vector &mSpeed) { mSpeed = this->_v; }
void nRF24Communication::clearSSLData() {
  this->_v.x = 0;
  this->_v.y = 0;
  this->_v.w = 0;
  this->_kick.front = false;
  this->_kick.chip = false;
  this->_kick.charge = false;
  this->_kick.bypassIR = false;
  this->_kick.kickStrength = 0;
  this->_kick.dribbler = false;
  this->_kick.dribblerSpeed = 0;
}

void nRF24Communication::getPosition(RobotPosition &pos) { pos = _pos; }

void nRF24Communication::getKick(KickFlags &isKick) {
  isKick.front = _kick.front;
  isKick.chip = _kick.chip;
  isKick.charge = _kick.charge;
  isKick.kickStrength = _kick.kickStrength;
  isKick.dribbler = _kick.dribbler;
  isKick.bypassIR = (_kick.front | _kick.chip) & _kick.charge;
  isKick.dribbler = _kick.dribbler;
  isKick.dribblerSpeed = _kick.dribblerSpeed;
  // printf("charge %d, chip %d, front %d, force %f, dribbler %d, dribblerSpeed
  // %f \n",isKick.charge,
  // isKick.chip,isKick.front,isKick.kickStrength,isKick.dribbler,isKick.dribblerSpeed
  // );
}

void nRF24Communication::showBitsReceived(int payload) {
  // A sequencia de campos segue o da declaracao da estrutura em commConfig.h,
  // mas os bits estao em little endian
  for (int i = 0; i < payload; i++) {
    int val = this->_rx.encoded[i];
    int mask = 1;
    for (int j = 0; j < 8; j++) {
      if (mask & val) {
        printf("1");
      } else {
        printf("0");
      }
      mask = mask << 1;
    }
    printf("|");
  }
  printf("\n");
}

float nRF24Communication::getKP() { return _kp; }
float nRF24Communication::getKD() { return _kd; }
float nRF24Communication::getAlpha() { return _alpha; }
