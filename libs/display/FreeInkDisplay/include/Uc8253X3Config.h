#pragma once

#include <cstdint>

namespace freeink {

// One UC8253 waveform bank: VCOM + the four transition LUTs.
struct Uc8253LutBank {
  const uint8_t* vcom = nullptr;
  const uint8_t* ww = nullptr;
  const uint8_t* bw = nullptr;
  const uint8_t* wb = nullptr;
  const uint8_t* bb = nullptr;
};

struct Uc8253Lut16Config {
  Uc8253LutBank preBwMid;     // 27-frame differential base bank (CDI 0xA9, 0x07)
  Uc8253LutBank pass1Coarse;  // 7-frame coarse bank (CDI 0x29, 0x07)
  Uc8253LutBank pass2Fine;    // 5-frame fine sub-step bank (CDI 0x29, 0x07)
  uint8_t pllBase = 0x09;     // 80 Hz nominal
  uint8_t pllPass1 = 0x0B;    // 100 Hz nominal
  uint8_t pllPass2 = 0x0F;    // 200 Hz nominal
  uint8_t vdlPass2 = 0x38;    // CMD_POWER_SETTING byte[3] during Pass 2 (-13.6V)
};

struct Uc8253X3Config {
  Uc8253LutBank normal;    // condition-pass / settle (CDI 0xA9)
  Uc8253LutBank half;      // scrub (CDI 0xA9)
  Uc8253LutBank fast;      // turbo differential (CDI 0x29)
  Uc8253LutBank full;      // OEM full / factory (CDI 0x29)
  Uc8253LutBank gc;        // OEM 4-level grayscale nudge (CDI 0x29)
  Uc8253LutBank preBwMid;  // OEM grayscale preconditioning settle (CDI 0xA9)
  uint8_t lutLen = 42;     // bytes per LUT sent to the controller (42)
  const Uc8253LutBank* directGray = nullptr;
  Uc8253Lut16Config lut16 = {};
};

const Uc8253X3Config& uc8253X3DefaultConfig();

}  // namespace freeink
