#include <lqc/interface/gamepad.h>
#include <lqc/utils/utils.h>

namespace lqc {

void GamepadState::buildMappings(
    GamepadState::AMap &amap,
    GamepadState::BMap &bmap,
    Type type) {
  switch (type) {
    case Xbox: {
      bmap[0] = &A;
      bmap[1] = &B;
      bmap[3] = &X;
      bmap[4] = &Y;
      bmap[6] = &LB;
      bmap[7] = &RB;
      bmap[13] = &LAS;
      bmap[14] = &RAS;
      amap[0] = &las_x;
      amap[1] = &las_y;
      amap[2] = &ras_x;
      amap[3] = &ras_y;
      amap[4] = &rt;
      amap[5] = &lt;
      break;
    }
    case PS: {
      bmap[0] = &A;
      bmap[1] = &B;
      bmap[2] = &X;
      bmap[3] = &Y;
      bmap[4] = &LB;
      bmap[5] = &RB;
      bmap[9] = &LAS;
      bmap[10] = &RAS;
      amap[0] = &las_x;
      amap[1] = &las_y;
      amap[3] = &ras_x;
      amap[4] = &ras_y;
      amap[2] = &lt;
      amap[5] = &rt;
      break;
    }
    default: {
      LQC_ERROR("Unintended condition!")
    }
  }
}

std::ostream &operator<<(std::ostream &os, const GamepadState &gp) {
  return os << "GamepadState("
            << "LAS=(" << gp.las_x << ", " << gp.las_y << "), "
            << "RAS=(" << gp.ras_x << ", " << gp.ras_y << "), "
            << "LT=" << gp.lt << ", RT=" << gp.rt << ", "
            << "A=" << gp.A.pressed << ", B=" << gp.B.pressed << ", "
            << "X=" << gp.X.pressed << ", Y=" << gp.Y.pressed << ", "
            << "LB=" << gp.LB.pressed << ", RB=" << gp.RB.pressed << ", "
            << "LAS=" << gp.LAS.pressed << ", RAS=" << gp.RAS.pressed << ")";
}

Gamepad::Gamepad() {
  hook_ = gamepad::hook::make();
  hook_->set_plug_and_play(true, gamepad::ms(1000));
  hook_->set_sleep_time(gamepad::ms(5));
  std::string gamepad_type;
  if (ev::parse("LQC_GAMEPAD_TYPE", gamepad_type) == ev::Success) {
    if (gamepad_type == "Xbox") {
      state_.buildMappings(amap_, bmap_, GamepadState::Xbox);
    } else if (gamepad_type == "PS") {
      state_.buildMappings(amap_, bmap_, GamepadState::PS);
    } else {
      LQC_ERROR("Invalid gamepad type: " << gamepad_type)
    }
  } else {
    state_.buildMappings(amap_, bmap_);
  }

  hook_->set_connect_event_handler(
      [this](std::shared_ptr<gamepad::device> dev) { connectHandler(dev); });
  hook_->set_axis_event_handler(
      [this](std::shared_ptr<gamepad::device> dev) { axisHandler(dev); });
  hook_->set_button_event_handler(
      [this](std::shared_ptr<gamepad::device> dev) { buttonHandler(dev); });
  hook_->set_disconnect_event_handler(
      [this](std::shared_ptr<gamepad::device> dev) { disconnectHandler(dev); });
  if (!hook_->start()) {
    LQC_ERROR("Fail to connect gamepad.")
  }
}

void Gamepad::getState(State &state) {
  if (!connected_) state = {};
  std::lock_guard<std::mutex> lock(mtx_);
  state = state_;
  for (auto b : bmap_) b.second->status = false;
}

void Gamepad::connectHandler(std::shared_ptr<gamepad::device> dev) {
  connected_ = true;
  log("Gamepad ", dev->get_name(), " connected.");
}

void Gamepad::disconnectHandler(std::shared_ptr<gamepad::device> dev) {
  connected_ = false;
  lg::warn("Gamepad ", dev->get_name(), " disconnected.");
}

void Gamepad::axisHandler(std::shared_ptr<gamepad::device> dev) {
  float val = dev->last_axis_event()->virtual_value;
  uint16_t aid = dev->last_axis_event()->native_id;
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = amap_.find(aid);
  if (it != amap_.end()) {
    *(it->second) = val * 2 - 1;
  }
}

void Gamepad::buttonHandler(std::shared_ptr<gamepad::device> dev) {
  bool val = dev->last_button_event()->virtual_value > 0.99;
  uint16_t bid = dev->last_button_event()->native_id;
  std::lock_guard<std::mutex> lock(mtx_);
  auto it = bmap_.find(bid);
  if (it != bmap_.end()) {
    if (val) {
      it->second->status = true;
      it->second->pressed = true;
    } else {
      it->second->pressed = false;
    }
  }
}

}  // namespace lqc
