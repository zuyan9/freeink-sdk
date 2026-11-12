#!/usr/bin/env python3
"""Compile the real UC8253 X3 driver with a recording bus and host Arduino shims."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "src"
with tempfile.TemporaryDirectory(prefix="uc8253_power-test-") as directory:
    root = Path(directory)
    for folder in ("src/driver", "src/bus", "src/lut", "include"):
        (root / folder).mkdir(parents=True, exist_ok=True)
    for name in ("Uc8253X3Driver.cpp", "Uc8253X3Driver.h", "PanelDriver.h"):
        shutil.copy2(SOURCE / "driver" / name, root / "src/driver" / name)
    shutil.copy2(SOURCE / "FreeInkDisplay.cpp", root / "src/FreeInkDisplay.cpp")
    shutil.copy2(SOURCE / "lut/Uc8253X3Luts.h", root / "src/lut/Uc8253X3Luts.h")
    shutil.copy2(SOURCE / "lut/UltraChipDirectGrayLuts.h", root / "src/lut/UltraChipDirectGrayLuts.h")
    for inc_name in ("FreeInkDisplay.h", "GrayscaleCapabilities.h", "Uc8253X3Config.h", "Uc8253X3Lut16.h"):
        shutil.copy2(SOURCE.parent / "include" / inc_name, root / "include" / inc_name)
    (root / "SPI.h").write_text("#pragma once\n")
    (root / "sdkconfig.h").write_text("#pragma once\n")
    (root / "Arduino.h").write_text('''#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#define HIGH 1
#define PROGMEM
#define pgm_read_byte(p) (*reinterpret_cast<const uint8_t*>(p))
#define log_e(...) std::fprintf(stderr, __VA_ARGS__)
inline unsigned long millis() { static unsigned long t; return ++t; }
inline void delay(unsigned long) {}
inline int digitalRead(int) { return 0; }
''')
    (root / "BoardConfig.h").write_text('''#pragma once
#include <cstdint>
namespace BoardConfig {
enum class Board { XteinkX3, XteinkX3Uc8279 };
enum class DisplayController { UC8253, UC8279 };
inline struct {
  uint16_t displayWidth = 792, displayHeight = 528;
  uint32_t displaySpiHz = 0;
  struct { int8_t sclk = 0, mosi = 0, cs = 0, dc = 0, rst = 0, busy = 0, powerEnable = -1; } display;
  Board board = Board::XteinkX3;
  DisplayController displayController = DisplayController::UC8253;
} ACTIVE;
constexpr uint32_t MAX_FRAMEBUFFER_BYTES = (792 / 8) * 528;
inline void selectDevice(Board b) { ACTIVE.board = b; }
}
''')
    (root / "src/bus/EpdBus.h").write_text('''#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>
#include <cstring>
namespace freeink {
enum class BusyPolarity { X3TwoPhase };
struct EpdPins { int8_t sclk = 0, mosi = 0, cs = 0, dc = 0, rst = 0, busy = 0, powerEnable = -1; };
class EpdBus {
  uint8_t command=0;
  bool inPartial=false;
  uint16_t ptlGateStart=0, ptlGateEnd=0, ptlCursorGate=0;
  std::vector<uint8_t> curParam;
public:
  std::vector<uint8_t> oldPlane, newPlane, lastBank, lastWw, lastBb, rawRegisters;
  std::vector<uint8_t> pllWrites;
  std::vector<std::array<uint8_t, 5>> pwrWrites;
  std::vector<std::array<uint8_t, 2>> cdiWrites;
  std::vector<uint8_t> pllAtRefresh;
  std::vector<std::array<uint8_t, 5>> pwrAtRefresh;
  std::vector<uint8_t> pllAtPowerOff;
  std::vector<std::array<uint8_t, 5>> pwrAtPowerOff;
  uint8_t currentPll=0;
  std::array<uint8_t, 5> currentPwr{};
  unsigned powerOns=0, powerOffs=0, refreshes=0, pollingRefreshWaits=0, interruptWaits=0, dataStops=0, partialIns=0, partialOuts=0;
  void commitParam() {
    if (command == 0x30 && curParam.size() == 1) {
      currentPll = curParam[0];
      pllWrites.push_back(currentPll);
    } else if (command == 0x01 && curParam.size() == 5) {
      for (size_t i = 0; i < 5; ++i) currentPwr[i] = curParam[i];
      pwrWrites.push_back(currentPwr);
    } else if (command == 0x50 && curParam.size() == 2) {
      cdiWrites.push_back({curParam[0], curParam[1]});
    } else if (command == 0x90 && curParam.size() == 9) {
      ptlGateStart = static_cast<uint16_t>((curParam[4] << 8) | curParam[5]);
      ptlGateEnd = static_cast<uint16_t>((curParam[6] << 8) | curParam[7]);
      ptlCursorGate = ptlGateStart;
    }
  }
  void cmd(uint8_t c) {
    commitParam();
    curParam.clear();
    command = c;
    if (c == 0x02) {
      ++powerOffs;
      pllAtPowerOff.push_back(currentPll);
      pwrAtPowerOff.push_back(currentPwr);
    }
    if (c == 0x04) ++powerOns;
    if (c == 0x11) ++dataStops;
    if (c == 0x12) {
      ++refreshes;
      pllAtRefresh.push_back(currentPll);
      pwrAtRefresh.push_back(currentPwr);
    }
    if (c == 0x91) { inPartial = true; ++partialIns; }
    if (c == 0x92) { inPartial = false; ++partialOuts; }
    if ((c == 0x10 || c == 0x13) && inPartial) ptlCursorGate = ptlGateStart;
  }
  void cmdData2(uint8_t c, uint8_t a, uint8_t b) { cmd(c); data(a); data(b); commitParam(); curParam.clear(); }
  void data(uint8_t v) { curParam.push_back(v); }
  void data(const uint8_t* p, size_t n) {
    curParam.assign(p, p + n);
    commitParam();
    curParam.clear();
    if(n == 42 || n == 49) {
      if(command == 0x20) rawRegisters.clear();
      rawRegisters.push_back(command);
      if(command == 0x20) lastBank.assign(p,p+n);
      if(command == 0x21) lastWw.assign(p,p+n);
      if(command == 0x24) lastBb.assign(p,p+n);
    }
  }
  void cmdData(uint8_t c, const uint8_t* p, size_t n) { cmd(c); data(p,n); }
  void sendPlaneFlipped(uint8_t c, const uint8_t* p, uint16_t h, uint16_t wb) {
    auto& dst = (c == 0x10 ? oldPlane : newPlane);
    dst.resize(size_t(h) * wb);
    for (uint16_t row = 0; row < h; ++row) {
      const uint16_t gate = static_cast<uint16_t>(h - 1 - row);
      std::memcpy(dst.data() + size_t(gate) * wb, p + size_t(row) * wb, wb);
    }
  }
  void fillPlane(uint8_t c, uint8_t v, uint16_t h, uint16_t wb) {
    (c == 0x10 ? oldPlane : newPlane).assign(size_t(h)*wb,v);
  }
  void reset(int) {}
  void begin(const EpdPins&, uint32_t, BusyPolarity, int8_t = -1, int8_t = -1) {}
  bool isBusy() const { return false; }
  void setBusyWaitHooks(void (*)(), void (*)()) {}
  void setBusyWaitSliceHook(bool (*)(int8_t, uint8_t)) {}
  void waitBusy(const char* tag) { if (!std::strcmp(tag, " X3_DRF")) ++pollingRefreshWaits; }
  void waitRefreshComplete(const char*) { ++interruptWaits; }
  EpdPins pins() const { return {}; }
  struct Transaction {
    EpdBus* owner;
    void writeBytes(const uint8_t* p, size_t n) {
      if (!owner || !owner->inPartial) return;
      auto& dst = (owner->command == 0x10 ? owner->oldPlane : owner->newPlane);
      const size_t off = size_t(owner->ptlCursorGate) * n;
      if (off + n <= dst.size()) std::memcpy(dst.data() + off, p, n);
      ++owner->ptlCursorGate;
    }
    void end() {}
  };
  Transaction beginTxn() { return {this}; }
};
}
''')
    (root / "test.cpp").write_text("""
#include <cassert>
#include <vector>
#include <iostream>
#define private public
#include "FreeInkDisplay.h"
#include "src/driver/Uc8253X3Driver.h"
#undef private
#include "Uc8253X3Lut16.h"
static std::vector<uint8_t> gateFlipped(const std::vector<uint8_t>& logical, size_t h = 528, size_t wb = 99) {
  std::vector<uint8_t> out(h * wb);
  for (size_t r = 0; r < h; ++r) {
    std::memcpy(out.data() + (h - 1 - r) * wb, logical.data() + r * wb, wb);
  }
  return out;
}
int main() {
 freeink::EpdBus bus; freeink::Uc8253X3Driver d;
 assert(d.supportsGray16());
 assert(d.spiHz() == 16000000u);
 const auto& defCfg = freeink::uc8253X3DefaultConfig();
 assert(defCfg.gc.bw[0] == 0x80 && defCfg.gc.bw[1] == 0x05 && defCfg.gc.bw[2] == 0x03 && defCfg.gc.bw[3] == 0x00);
 assert(defCfg.lut16.pllBase == 0x09 && defCfg.lut16.pllPass1 == 0x0B && defCfg.lut16.pllPass2 == 0x0F &&
        defCfg.lut16.vdlPass2 == 0x38);
 const auto caps = d.grayscaleCapabilities();
 assert(caps.supported() && caps.stripUploads && !caps.asyncBase && !caps.stagingWhileBusy);
 assert(d.grayscaleCapabilities(freeink::GrayscaleMode::Direct).base == freeink::GrayscaleBase::Combined);
 std::vector<uint8_t> fb(792/8*528, 0xAA);
 for (size_t r = 0; r < 528; ++r) fb[r * 99] = static_cast<uint8_t>(r & 0xFF);
 const auto fbGates = gateFlipped(fb);
 d.begin(bus);
 // Rollback: both splash and first real screen clean; only the third paint is fast.
 assert(!d.supportsAsyncDisplay());
 assert(!d.displayStart(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false));
 assert(bus.refreshes == 2 && bus.pollingRefreshWaits == 2);
 assert(bus.oldPlane == fbGates && bus.newPlane == fbGates);
 d.displayFinish(bus,fb.data());
 assert(bus.refreshes == 2);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false);
 assert(bus.refreshes == 5 && bus.pollingRefreshWaits == 5);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false);
 assert(bus.refreshes == 6 && bus.pollingRefreshWaits == 6 && bus.interruptWaits == 0);
 assert(bus.powerOns == 1);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Full,false);
 assert(bus.powerOns == 1);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Half,true);
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false);
 assert(bus.powerOns == 2);
 for (auto fallback : {freeink::RefreshMode::Half, freeink::RefreshMode::Fast}) {
   const auto before = bus.refreshes;
   d.beginGrayscale(bus, fb.data(), freeink::GrayscaleMode::Direct, fallback, false);
   d.copyGrayscaleLsb(bus, fb.data());
   d.copyGrayscaleMsb(bus, fb.data());
   assert(bus.refreshes == before);
   d.displayGray(bus, fb.data(), false, nullptr, true);
   assert(bus.refreshes == before + 1);
   const auto vcom = freeink::uc8253X3DefaultConfig().directGray->vcom;
   assert(bus.lastBank == std::vector<uint8_t>(vcom, vcom + 42));
   d.cleanupGrayscaleBuffers(bus, fb.data());
   d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Full,false);
   assert(bus.newPlane == fbGates && bus.oldPlane == fbGates);
 }
 // Rollback: restored B/W RAM permits a differential gray base after an AA pass.
 d.display(bus,fb.data(),nullptr,freeink::RefreshMode::Fast,false);
 d.copyGrayscaleLsb(bus, fb.data());
 d.copyGrayscaleMsb(bus, fb.data());
 d.displayGray(bus, fb.data(), false, nullptr, false);
 d.cleanupGrayscaleBuffers(bus, fb.data());
 const auto before = bus.refreshes;
 d.beginGrayscale(bus, fb.data(), freeink::GrayscaleMode::Absolute, freeink::RefreshMode::Half, false);
 assert(bus.refreshes == before + 1 && bus.interruptWaits == 0);
 d.cleanupGrayscaleBuffers(bus, fb.data());

 // =========================================================================
 // Two-Pass 16-Gray Hardware Sequence Verification
 // =========================================================================
 constexpr std::array<uint8_t, 5> kStdPwr = {0x07, 0x17, 0x3F, 0x3F, 0x17};
 constexpr std::array<uint8_t, 5> kFinePwr = {0x07, 0x17, 0x3F, 0x38, 0x17};
 std::vector<uint8_t> cLsb(99 * 528, 0x11), cMsb(99 * 528, 0x22);
 std::vector<uint8_t> fLsb(99 * 528, 0x33), fMsb(99 * 528, 0x44);
 std::vector<uint8_t> cleanupDtm1(99 * 528, 0xFF);
 for (size_t r = 0; r < 528; ++r) {
   cLsb[r * 99 + 1] = static_cast<uint8_t>(r);
   fMsb[r * 99 + 2] = static_cast<uint8_t>(r ^ 0x55);
   cleanupDtm1[r * 99 + 3] = static_cast<uint8_t>(~(r & 0xFF));  // ink 12..15 + status bar 0 bits
 }
 auto streamStrips = [&](freeink::GrayPlane plane, const std::vector<uint8_t>& planeImg) {
   for (uint16_t y = 0; y < 528; y += 80) {
     const uint16_t rows = (528 - y < 80) ? (528 - y) : 80;
     d.writeGrayscalePlaneStrip(bus, plane, planeImg.data() + size_t(y) * 99, y, rows);
   }
 };

 // Page 1: entering 2-pass 16-gray from clean B/W state (_gray16CleanupActive == false)
 assert(!d.isGray16CleanupActive());
 const unsigned refBeforeP1 = bus.refreshes;
 d.displayGrayscaleBase(bus, fb.data(), freeink::RefreshMode::Fast, false);
 assert(bus.refreshes == refBeforeP1 + 1);
 assert(bus.lastWw[0] == 0x20 && bus.lastBb[0] == 0x10);  // standard OEM preBwMid on first entry

 // Step 2: Pass 1 Coarse (PLL=0x0B, CDI 0x29/0x07, kLut16Pass1Coarse)
 streamStrips(freeink::GrayPlane::Lsb, cLsb);
 streamStrips(freeink::GrayPlane::Msb, cMsb);
 assert(bus.oldPlane == gateFlipped(cLsb) && bus.newPlane == gateFlipped(cMsb));
 d.displayGray16Pass1(bus);
 assert(bus.refreshes == refBeforeP1 + 2);
 assert(bus.pllAtRefresh.back() == freeink::x3_lut16::kPllPass1Coarse);
 assert(bus.pwrAtRefresh.back() == kStdPwr);
 assert((bus.cdiWrites.back() == std::array<uint8_t, 2>{0x29, 0x07}));
 assert(bus.lastWw == std::vector<uint8_t>(freeink::x3_lut16::kLut16Pass1Coarse_ww,
                                           freeink::x3_lut16::kLut16Pass1Coarse_ww + 42));
 // Pass 1 restores base PLL immediately after its refresh:
 assert(bus.currentPll == freeink::x3_lut16::kPllBase);

 // Early-abort of Pass 2 (before LSB strips are written) must leave PLL == 0x09 (kPllBase):
 d.displayGray16Pass2(bus, false);
 assert(bus.refreshes == refBeforeP1 + 2);
 assert(bus.currentPll == freeink::x3_lut16::kPllBase);

 // Step 3: Pass 2 Fine (PLL=0x0F, VDL=0x38, CDI 0x29/0x07, kLut16Pass2Fine -> restore 0x09 + kPwrStandard)
 streamStrips(freeink::GrayPlane::Lsb, fLsb);
 streamStrips(freeink::GrayPlane::Msb, fMsb);
 assert(bus.oldPlane == gateFlipped(fLsb) && bus.newPlane == gateFlipped(fMsb));
 d.displayGray16Pass2(bus, false);
 assert(bus.refreshes == refBeforeP1 + 3);
 assert(bus.pllAtRefresh.back() == freeink::x3_lut16::kPllPass2Fine);
 assert(bus.pwrAtRefresh.back() == kFinePwr);
 assert((bus.cdiWrites.back() == std::array<uint8_t, 2>{0x29, 0x07}));
 assert(bus.lastWw == std::vector<uint8_t>(freeink::x3_lut16::kLut16Pass2Fine_ww,
                                           freeink::x3_lut16::kLut16Pass2Fine_ww + 42));
 // Rails restored immediately after Pass 2 DRF:
 assert(bus.currentPll == freeink::x3_lut16::kPllBase);
 assert(bus.currentPwr == kStdPwr);

 // Step 4: Post-pass DTM1 cleanup (0 refreshes / 0 ms BUSY: DTM2=BW, DTM1=CleanupDTM1)
 const unsigned refBeforeCleanup = bus.refreshes;
 d.beginGray16Cleanup(bus, fb.data());
 streamStrips(freeink::GrayPlane::Lsb, cleanupDtm1);
 d.finishGray16Cleanup(bus);
 assert(bus.refreshes == refBeforeCleanup);  // ZERO BUSY refreshes during cleanup!
 assert(bus.newPlane == fbGates);
 assert(bus.oldPlane == gateFlipped(cleanupDtm1));
 assert(d.isGray16CleanupActive());

 // Page 2: displayGrayscaleBase(Fast) with _gray16CleanupActive == true MUST fire kLut16PreBwMid!
 d.displayGrayscaleBase(bus, fb.data(), freeink::RefreshMode::Fast, false);
 assert(!d.isGray16CleanupActive());
 assert(bus.refreshes == refBeforeCleanup + 1);
 assert(bus.pllAtRefresh.back() == freeink::x3_lut16::kPllBase);
 assert(bus.pwrAtRefresh.back() == kStdPwr);
 assert((bus.cdiWrites.back() == std::array<uint8_t, 2>{0xA9, 0x07}));
 assert(bus.lastWw[0] == 0xAA && bus.lastBb[0] == 0x5F);

 // Re-arm cleanup and verify Reader Popup B/W display(Fast) runs differentially & clears flag:
 streamStrips(freeink::GrayPlane::Lsb, cLsb);
 streamStrips(freeink::GrayPlane::Msb, cMsb);
 d.displayGray16Pass1(bus);
 streamStrips(freeink::GrayPlane::Lsb, fLsb);
 streamStrips(freeink::GrayPlane::Msb, fMsb);
 d.displayGray16Pass2(bus, false);
 d.beginGray16Cleanup(bus, fb.data());
 streamStrips(freeink::GrayPlane::Lsb, cleanupDtm1);
 d.finishGray16Cleanup(bus);
 assert(d.isGray16CleanupActive());

 const unsigned refBeforePopup = bus.refreshes;
 d.display(bus, fb.data(), nullptr, freeink::RefreshMode::Fast, false);
 assert(bus.refreshes == refBeforePopup + 1);  // Single fast diff, NO grayscaleRevert scrub!
 assert(!d.isGray16CleanupActive());

 // Re-arm cleanup and verify HALF_REFRESH + preconditionGrayscale clears flag:
 d.beginGray16Cleanup(bus, fb.data());
 streamStrips(freeink::GrayPlane::Lsb, cleanupDtm1);
 d.finishGray16Cleanup(bus);
 assert(d.isGray16CleanupActive());
 d.display(bus, fb.data(), nullptr, freeink::RefreshMode::Half, false);
 d.preconditionGrayscale(bus, 0, 0, 792, 528);
 assert(!d.isGray16CleanupActive());

 // Verify displayGray16Pass2(turnOff=true) restores PLL + kPwrStandard BEFORE CMD_POWER_OFF
 // and forces CMD_POWER_ON on the next display refresh:
 streamStrips(freeink::GrayPlane::Lsb, cLsb);
 streamStrips(freeink::GrayPlane::Msb, cMsb);
 d.displayGray16Pass1(bus);
 streamStrips(freeink::GrayPlane::Lsb, fLsb);
 streamStrips(freeink::GrayPlane::Msb, fMsb);
 const unsigned pwrOnsBeforeTurnOff = bus.powerOns;
 const unsigned pwrOffsBeforeTurnOff = bus.powerOffs;
 d.displayGray16Pass2(bus, /*turnOff=*/true);
 assert(bus.powerOffs == pwrOffsBeforeTurnOff + 1);
 assert(bus.pllAtPowerOff.back() == freeink::x3_lut16::kPllBase);
 assert(bus.pwrAtPowerOff.back() == kStdPwr);
 assert(bus.currentPll == freeink::x3_lut16::kPllBase);
 assert(bus.currentPwr == kStdPwr);
 d.beginGray16Cleanup(bus, fb.data());
 streamStrips(freeink::GrayPlane::Lsb, cleanupDtm1);
 d.finishGray16Cleanup(bus);
 d.displayGrayscaleBase(bus, fb.data(), freeink::RefreshMode::Fast, false);
 assert(bus.powerOns == pwrOnsBeforeTurnOff + 1);

 // Verify register bitmasks clamp high bits in Uc8253X3Config::lut16:
 freeink::Uc8253X3Config maskedCfg = freeink::uc8253X3DefaultConfig();
 maskedCfg.lut16.pllPass1 = 0xFA;
 maskedCfg.lut16.pllPass2 = 0xFE;
 maskedCfg.lut16.vdlPass2 = 0xF8;
 freeink::Uc8253X3Driver maskedDriver(maskedCfg);
 maskedDriver.begin(bus);
 maskedDriver.skipInitialResync();
 maskedDriver.displayGrayscaleBase(bus, fb.data(), freeink::RefreshMode::Fast, false);
 for (uint16_t y = 0; y < 528; y += 80) {
   const uint16_t rows = (528 - y < 80) ? (528 - y) : 80;
   maskedDriver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Lsb, cLsb.data() + size_t(y) * 99, y, rows);
   maskedDriver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Msb, cMsb.data() + size_t(y) * 99, y, rows);
 }
 maskedDriver.displayGray16Pass1(bus);
 assert(bus.pllAtRefresh.back() == 0x0A);
 for (uint16_t y = 0; y < 528; y += 80) {
   const uint16_t rows = (528 - y < 80) ? (528 - y) : 80;
   maskedDriver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Lsb, fLsb.data() + size_t(y) * 99, y, rows);
   maskedDriver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Msb, fMsb.data() + size_t(y) * 99, y, rows);
 }
 maskedDriver.displayGray16Pass2(bus, false);
 assert(bus.pllAtRefresh.back() == 0x0E);
 assert(bus.pwrAtRefresh.back() == kFinePwr);

 // Verify Uc8253LutBank / Uc8253Lut16Config defaults and null-bank guard:
 const freeink::Uc8253LutBank emptyBank{};
 assert(emptyBank.vcom == nullptr && emptyBank.ww == nullptr && emptyBank.bw == nullptr &&
        emptyBank.wb == nullptr && emptyBank.bb == nullptr);
 const freeink::Uc8253Lut16Config defaultLut16Cfg{};
 assert(defaultLut16Cfg.pllBase == 0x09 && defaultLut16Cfg.pllPass1 == 0x0B &&
        defaultLut16Cfg.pllPass2 == 0x0F && defaultLut16Cfg.vdlPass2 == 0x38);
 assert(freeink::gray16::kMask4BitBw == 0xFFFCu && freeink::gray16::kMaskBwClear == 0xFFFEu);
 freeink::Uc8253X3Config noLut16Cfg = freeink::uc8253X3DefaultConfig();
 noLut16Cfg.lut16 = {};
 freeink::Uc8253X3Driver noLut16Driver(noLut16Cfg);
 assert(!noLut16Driver.supportsGray16());
 noLut16Driver.begin(bus);
 noLut16Driver.skipInitialResync();
 noLut16Driver.displayGrayscaleBase(bus, fb.data(), freeink::RefreshMode::Fast, false);
 noLut16Driver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Lsb, cLsb.data(), 0, 80);
 noLut16Driver.writeGrayscalePlaneStrip(bus, freeink::GrayPlane::Msb, cMsb.data(), 0, 80);
 const unsigned refBeforeNoLut16 = bus.refreshes;
 noLut16Driver.displayGray16Pass1(bus);
 noLut16Driver.displayGray16Pass2(bus, false);
 assert(bus.refreshes == refBeforeNoLut16);
 noLut16Driver.beginGray16Cleanup(bus, fb.data());
 noLut16Driver.finishGray16Cleanup(bus);
 assert(!noLut16Driver.isGray16CleanupActive());

 // Verify powerOffBetweenFrames powers off HV rails cleanly when _isScreenOn is true:
 const unsigned pwrOffBeforeIdle = bus.powerOffs;
 noLut16Driver.powerOffBetweenFrames(bus);
 assert(bus.powerOffs == pwrOffBeforeIdle + 1);
 noLut16Driver.powerOffBetweenFrames(bus);
 assert(bus.powerOffs == pwrOffBeforeIdle + 1);  // idempotent when already off

 // Verify FreeInkDisplay Dark -> Light displayGrayscaleBase runs preconditionGrayscale
 // without prematurely cycling HV rails when turnOffScreen == false:
 freeink::FreeInkDisplay facade(0, 0, 0, 0, 0, 0);
 facade.setDisplayX3();
 facade.begin();
 facade.skipInitialResync();
 std::memcpy(facade.getFrameBuffer(), fb.data(), fb.size());
 facade.displayBuffer(freeink::FreeInkDisplay::FAST_REFRESH, false);

 // Enter Dark Mode: displayGrayscaleBase runs HALF_REFRESH scrub without preconditionGrayscale.
 facade.setInverted(true);
 const unsigned ptlBeforeDark = facade._bus.partialIns;
 facade.displayGrayscaleBase(freeink::FreeInkDisplay::FAST_REFRESH, false);
 assert(facade._bus.partialIns == ptlBeforeDark);

 // Exit Dark Mode (turnOffScreen == false): HALF_REFRESH scrub + preconditionGrayscale, 0 powerOffs!
 facade.setInverted(false);
 const unsigned refBeforeExitDark = facade._bus.refreshes;
 const unsigned pofBeforeExitDark = facade._bus.powerOffs;
 const unsigned ptlBeforeExitDark = facade._bus.partialIns;
 facade.displayGrayscaleBase(freeink::FreeInkDisplay::FAST_REFRESH, /*turnOffScreen=*/false);
 assert(facade._bus.powerOffs == pofBeforeExitDark);
 assert(facade._bus.refreshes == refBeforeExitDark + 2);  // 1 HALF scrub + 1 preconditionGrayscale settle
 assert(facade._bus.partialIns == ptlBeforeExitDark + 1 && facade._bus.partialOuts == ptlBeforeExitDark + 1);
 assert((facade._bus.cdiWrites.back() == std::array<uint8_t, 2>{0xA9, 0x07}));
 assert(facade._bus.lastWw[0] == 0x20 && facade._bus.lastBb[0] == 0x10);

 // Exit Dark Mode with turnOffScreen == true: rails stay ON across HALF + precondition, then power off ONCE:
 facade.setInverted(true);
 facade.displayBuffer(freeink::FreeInkDisplay::FAST_REFRESH, false);
 facade.setInverted(false);
 const unsigned pofBeforeTurnOffExit = facade._bus.powerOffs;
 const unsigned ponBeforeTurnOffExit = facade._bus.powerOns;
 facade.displayGrayscaleBase(freeink::FreeInkDisplay::FAST_REFRESH, /*turnOffScreen=*/true);
 assert(facade._bus.powerOns == ponBeforeTurnOffExit);        // no mid-sequence POF+PON bounce
 assert(facade._bus.powerOffs == pofBeforeTurnOffExit + 1);   // powered off cleanly at end
 facade.releaseBuffers();

 std::cout << "PASS: UC8253 rollback boot cleans, blocking polling, gray baseline, power, Direct grayscale, and two-pass 16-gray\\n";
}
""")
    exe = root / "test_uc8253_power"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra",
                    "-Wno-unused-parameter", "-DARDUINO=1", "-DFREEINK_DRIVER_UC8253_X3=1",
                    "-DEINK_DISPLAY_SINGLE_BUFFER_MODE=1",
                    "-I"+str(root), "-I"+str(root / "include"), str(root / "test.cpp"),
                    str(root / "src/FreeInkDisplay.cpp"),
                    str(root / "src/driver/Uc8253X3Driver.cpp"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
