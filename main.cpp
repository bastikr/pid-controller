#include "pidcontroller/pidcontroller.h"
#include <iostream>

using namespace pidcontroller;

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

  std::cout << "Hello, world2!" << std::endl;
  return 0;
}

