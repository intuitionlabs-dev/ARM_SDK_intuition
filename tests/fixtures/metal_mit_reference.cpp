// MIT License, Copyright (c) 2025 MakerMods. See LICENSE.MakerMods.
// Extracted encoding arithmetic from metal-python-ros bccf3c8,
// metal_sdk/native/motor_writers/dm_motor_writer.cpp (MitControl).
// Only the executable harness / parameter input is added here.
#include <cstdint>
#include <iomanip>
#include <iostream>

int main() {
  float pmax, vmax, tmax, position, velocity, torque, kp, kd;
  while (std::cin >> pmax >> vmax >> tmax >> position >> velocity >> torque >> kp >> kd) {
    auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
      float span = xmax - xmin;
      float data_norm = (x - xmin) / span;
      uint16_t data_uint = data_norm * ((1 << bits) - 1);
      return data_uint;
    };
    uint16_t pos_tmp = float_to_uint(position, -pmax, pmax, 16);
    uint16_t vel_tmp = float_to_uint(velocity, -vmax, vmax, 12);
    uint16_t kp_tmp = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_tmp = float_to_uint(kd, 0, 5, 12);
    uint16_t tor_tmp = float_to_uint(torque, -tmax, tmax, 12);
    uint8_t data[8];
    data[0] = pos_tmp >> 8;
    data[1] = pos_tmp;
    data[2] = vel_tmp >> 4;
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = kd_tmp >> 4;
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    for (uint8_t byte : data)
      std::cout << std::hex << std::setfill('0') << std::setw(2) << int(byte);
    std::cout << '\n';
  }
}
