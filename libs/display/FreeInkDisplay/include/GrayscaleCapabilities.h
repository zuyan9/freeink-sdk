#pragma once

#include <cstdint>

namespace freeink {

// Overlay uses the existing base + LSB/MSB mask API. Absolute requires planes
// containing every pixel, including the B/W background. Direct uses that same
// encoding with one combined activation using the full-quality image waveform.
// Querying selects no mode.
enum class GrayscaleMode : uint8_t { Overlay, Absolute, Direct };
enum class GrayscaleEncoding : uint8_t { Unsupported, OverlayMasks, AbsolutePlanes };
enum class GrayscaleBase : uint8_t { Separate, Combined };

struct GrayscaleCapabilities {
  // Overlay masks: black/white=00 (distinguished by the B/W base), dark=11,
  // light=01, in (LSB, MSB) order. Absolute planes: black=00, dark=10,
  // light=01, white=11. Encoding does not promise independence from prior ink.
  GrayscaleEncoding encoding = GrayscaleEncoding::Unsupported;
  GrayscaleBase base = GrayscaleBase::Separate;
  bool stripUploads = false;
  // An ordinary asynchronous B/W refresh is a valid base for this mode.
  bool asyncBase = false;
  // Plane staging touches no SPI and is safe during a pending B/W waveform.
  bool stagingWhileBusy = false;

  constexpr bool supported() const { return encoding != GrayscaleEncoding::Unsupported; }
};

namespace gray16 {

// Plane bitmasks indexed by 4-bit glyph ink 0..15 (0 = white, 15 = black).
inline constexpr uint16_t kMask4BitBw = 0xFFFCu;
inline constexpr uint16_t kMask4BitGrayLsb = 0x0F80u;
inline constexpr uint16_t kMask4BitGrayMsb = 0x0FFCu;
inline constexpr uint16_t kMaskBwClear = 0xFFFEu;
inline constexpr uint16_t kMaskPass1Dtm1 = 0x1A8Eu;
inline constexpr uint16_t kMaskPass1Dtm2 = 0x1BF0u;
inline constexpr uint16_t kMaskPass2Dtm1 = 0x4CD2u;
inline constexpr uint16_t kMaskPass2Dtm2 = 0x6A64u;
inline constexpr uint16_t kMaskCleanupBlack = 0xF000u;

// Maps 2-bit glyph ink (0..3) onto the corresponding 4-bit ink anchors (0, 5, 10, 15).
constexpr uint16_t mask2BitFrom4Bit(uint16_t mask4) {
  return static_cast<uint16_t>((((mask4 >> 0) & 1u) << 0) | (((mask4 >> 5) & 1u) << 1) | (((mask4 >> 10) & 1u) << 2) |
                               (((mask4 >> 15) & 1u) << 3));
}

// Maps a 2-bit image value (0 = black, 1 = dark gray, 2 = light gray, 3 = white) to 4-bit ink (15, 10, 5, 0).
constexpr uint8_t image2BitToInk4(uint8_t val) { return static_cast<uint8_t>((3u - (val & 3u)) * 5u); }

constexpr bool maskHasImage2Bit(uint16_t mask4, uint8_t val) {
  return (mask4 & static_cast<uint16_t>(1u << image2BitToInk4(val))) != 0;
}

// Maps 5-bit plane tuple (bw, coarseLsb, coarseMsb, fineLsb, fineMsb) to 4-bit gray (0 = black .. 15 = white).
inline constexpr uint8_t kDecodeGray16[32] = {
    0,  2,  5,  1,  7,  10, 11, 9,  12, 13, 14, 13, 3,  6,  8,  4,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
};

}  // namespace gray16

namespace x3_lut16 {
using gray16::image2BitToInk4;
using gray16::kDecodeGray16;
using gray16::kMask4BitBw;
using gray16::kMask4BitGrayLsb;
using gray16::kMask4BitGrayMsb;
using gray16::kMaskBwClear;
using gray16::kMaskCleanupBlack;
using gray16::kMaskPass1Dtm1;
using gray16::kMaskPass1Dtm2;
using gray16::kMaskPass2Dtm1;
using gray16::kMaskPass2Dtm2;
using gray16::mask2BitFrom4Bit;
using gray16::maskHasImage2Bit;
}  // namespace x3_lut16

}  // namespace freeink
