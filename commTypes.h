#ifndef COMM_TYPES_H
#define COMM_TYPES_H

/************* AUXILIAR TYPES ************/
#include "commConfig.h"
#include <stdint.h>

enum class RadioFunction { receiver, sender };

/*
 * Network type / catergory configuration
 *  Attencion: Limited in 15
 */
enum class NetworkType { unknown = 0, generic, ssl, vss };

/*
 * Type of the packet position.
 *  Attention: Limited in 8
 */
enum class PositionType { unknown = 0, source, stop, motionControl, rotateControl, rotateInPoint };

enum class refereeCommand {
  halt = 0,
  stop = 4,
  forceStart = 11,
  ballPlacementYellow = 16,
  ballPlacementBlue = 17
};

typedef struct {
  uint8_t payload;
  uint64_t addr[2];
  bool ack;
  uint8_t receiveChannel;
  uint8_t sendChannel;
  uint8_t pipeNum;
  bool reConfig;
  RadioFunction function;
} NetworkConfig;

typedef struct Motors {
  double m1 = 0;
  double m2 = 0;
  double m3 = 0;
  double m4 = 0;

  Motors() {
    m1 = 0;
    m2 = 0;
    m3 = 0;
    m4 = 0;
  }

  inline Motors operator+(Motors a) {
    Motors b;
    b.m1 = m1 + a.m1;
    b.m2 = m2 + a.m2;
    b.m3 = m3 + a.m3;
    b.m4 = m4 + a.m4;
    return b;
  }
} Motors;

typedef struct Vector {
  double x = 0;
  double y = 0;
  double w = 0;

  Vector() {
    x = 0;
    y = 0;
    w = 0;
  }

  Vector(double _x, double _y, double _w) {
    x = _x;
    y = _y;
    w = _w;
  }

  inline Vector operator+(Vector a) {
    Vector b;
    b.x = x + a.x;
    b.y = y + a.y;
    b.w = w + a.w;
    return b;
  }

  inline Vector operator-(Vector a) {
    Vector b;
    b.x = x - a.x;
    b.y = y - a.y;
    b.w = w - a.w;
    return b;
  }

  inline Vector operator*(double a) {
    Vector b;
    b.x = x * a;
    b.y = y * a;
    b.w = w * a;
    return b;
  }
} Vector;

typedef struct KickFlags {
  bool front = false;
  bool chip = false;
  bool charge = false;
  float kickStrength = 0;
  bool ball = false;
  bool dribbler = false;
  bool bypassIR = false;
  float dribblerSpeed = 0;

  KickFlags& operator=(const KickFlags& a) {
    front = a.front;
    chip = a.chip;
    charge = a.charge;
    kickStrength = a.kickStrength;
    ball = a.ball;
    dribbler = a.dribbler;
    bypassIR = a.bypassIR;
    dribblerSpeed = a.dribblerSpeed;
    return *this;
  }
} KickFlags;

typedef struct RobotPosition {
  Vector v;
  bool resetOdometry{};
  PositionType type = PositionType::unknown;
  double maxSpeed{};
  double minSpeed{};
  double rotateKp{};
  bool usingPropSpeed{};
  double minDistanceToPropSpeed{};
  bool rotateInClockWise{};
  double orbitRadius{};
  double approachKp{};

  RobotPosition& operator=(const RobotPosition& a) {
    resetOdometry = a.resetOdometry;
    v = a.v;
    type = a.type;
    maxSpeed = a.maxSpeed;
    minSpeed = a.minSpeed;
    rotateKp = a.rotateKp;
    usingPropSpeed = a.usingPropSpeed;
    minDistanceToPropSpeed = a.minDistanceToPropSpeed;
    rotateInClockWise = a.rotateInClockWise;
    orbitRadius = a.orbitRadius;
    approachKp = a.approachKp;
    return *this;
  }
} RobotPosition;

typedef struct {
  int id = -1;
  msgType type;
  Motors m;
  Motors current;
  Vector v;
  double dribbler = 0;
  double kickLoad = 0;
  bool ball = false;
  double battery = 0;
  uint8_t count = 0;
} RobotInfo;

typedef struct {
  int id = -1;
  msgType type;
  double m1;
  double m2;
  double battery = 0;
} VSSRobotInfo;

#endif // COMM_TYPES_H
