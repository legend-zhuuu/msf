#ifndef LQC_GAMEPAD_H_
#define LQC_GAMEPAD_H_

#include <iostream>
#include <map>

#include <libgamepad.hpp>

namespace lqc {

struct Button {
  operator bool() const { return status; }
  bool status{false}, pressed{false};
};

struct GamepadState {
  enum Type { Xbox, PS };
  static constexpr std::size_t nAXIS = 6;
  static constexpr std::size_t nBUTTON = 8;
  using AMap = std::map<int, float *>;
  using BMap = std::map<int, Button *>;

  float las_x{}, las_y{};
  float ras_x{}, ras_y{};
  float lt{-1}, rt{-1};
  Button A, B, X, Y;
  Button LB, RB, LAS, RAS;

  void buildMappings(AMap &amap, BMap &bmap, Type type = Xbox);
};

std::ostream &operator<<(std::ostream &os, const GamepadState &gp);

class Gamepad {
 public:
  using State = GamepadState;
  explicit Gamepad();
  bool connected() const { return connected_; }
  void getState(State &state);

 private:
  void connectHandler(std::shared_ptr<gamepad::device> dev);
  void disconnectHandler(std::shared_ptr<gamepad::device> dev);
  void axisHandler(std::shared_ptr<gamepad::device> dev);
  void buttonHandler(std::shared_ptr<gamepad::device> dev);

  State state_{};
  std::mutex mtx_;
  std::shared_ptr<gamepad::hook> hook_{nullptr};
  std::atomic<bool> connected_{false};
  State::AMap amap_;
  State::BMap bmap_;
};

}  // namespace lqc

#endif  // LQC_GAMEPAD_H_
