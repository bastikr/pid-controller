#include "pidcontroller/pidcontroller.h"

namespace pidcontroller {

void Plant::step(double dt, double v) {
  x_ += dt*(1.1 + 0.5*v) + sqrt(dt)*gaussian_(gen_);
}

double Sensor::measure(const Plant& plant) {
  return plant.x_;
}

double PIDController::step(double dt, double x_target, double x_control) {
  double e = x_target - x_control;
  E_ += dt*(e0_ + e)/2;
  double de = (e0_ - e)/dt;
  double u = Kp_*e + Ki_*E_ + Kd_*de;
  e0_ = e;
  return u;
}

}

