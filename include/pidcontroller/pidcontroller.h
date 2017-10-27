#include <random>
#include <cmath>

namespace pidcontroller {

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

class Sensor {
  public:
    double measure(const Plant&);
};

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

    // cummulated error
    double E_;
};

} // namespace pidcontroller

