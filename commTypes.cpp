#include "nRF24Communication.h"
#include "commTypes.h"

Pose Pose::operator+(const Pose& a) const {
  return Pose(x + a.x, y + a.y, theta + a.theta);
}

Pose Pose::operator-(const Pose& a) const {
  return Pose(x - a.x, y - a.y, theta - a.theta);
}

Pose Pose::operator*(const double a) const {
  return Pose(a * x, a * y, a * theta);
}

PoseDot Pose::operator/(const Time& t) const {
  return PoseDot(x / t.t, y / t.t, theta / t.t);
}

PoseDot PoseDot::operator+(const PoseDot& a) const {
  return PoseDot(vx + a.vx, vy + a.vy, omega + a.omega);
}

PoseDot PoseDot::operator-(const PoseDot& a) const {
  return PoseDot(vx - a.vx, vy - a.vy, omega - a.omega);
}

PoseDot PoseDot::operator*(const double a) const {
  return PoseDot(vx * a, vy * a, omega * a);
}

Pose PoseDot::operator*(const Time& t) const {
  return Pose(vx * t.t, vy * t.t, omega * t.t);
}