#include <iostream>
#include <random>
#include <cmath>

class Plant {
  public:
    Plant(double x0, double noise) {
      x_ = x0;
      std::random_device rd;
      gen_ = std::mt19937(rd());
      gaussian_ = std::normal_distribution<>(0, noise);
    }

    void step(double dt, double v);

    double x_;
    std::mt19937 gen_;
    std::normal_distribution<> gaussian_;
};

void Plant::step(double dt, double v) {
  x_ += dt*(1.1 + v) + sqrt(dt)*gaussian_(gen_);
}


class Sensor {
  public:
    double measure(const Plant&);
};

double Sensor::measure(const Plant& plant) {
  return plant.x_;
}


class PIDController {
  public:
    PIDController(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd), e0_(0), E_(0) {}

    double step(double dt, double x_target, double x_control);

    // gain constants
    double Kp_;
    double Ki_;
    double Kd_;

    // previous error
    double e0_;

    // commulated error
    double E_;
};

double PIDController::step(double dt, double x_target, double x_control) {
  double e = x_target - x_control;
  E_ += dt*(e0_ + e)/2;
  double de = (e0_ - e)/dt;
  double u = Kp_*e + Ki_*E_ + Kd_*de;
  e0_ = e;
  return u;
}


int main(int argc, char **argv) {
  PIDController controller(0.6, 0.5, 0.0);
  Plant plant(0, 0.5);
  Sensor sensor;

  const double dt = 0.1;
  const double x_target = 5;
  double F;
  double x_control;

  for (int i=0; i<1000; i++) {
    x_control = sensor.measure(plant);
    F = controller.step(dt, x_target, x_control);
    plant.step(dt, F);
//     std::cout << plant.x_ << std::endl;
    std::cout << "E: " << controller.E_ << "   x: " << plant.x_ << std::endl;
  }

  std::cout << "Hello, world!" << std::endl;
  return 0;
}

