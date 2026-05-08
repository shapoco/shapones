#include <shapones/shapones.hpp>
#include <xiamocon.hpp>

using namespace xmc;
using namespace xmc::input;

static constexpr xmc::PixelFormat DISPLAY_FORMAT = xmc::PixelFormat::RGB444;
xmc::Sprite frameBuffer =
    xmc::createSprite444(xmc::display::WIDTH, xmc::display::HEIGHT);
xmc::FpsKeeper fpsKeeper(60);

// NES color table
const uint16_t COLOR_TABLE_0[] = {
    0x5055, 0x7002, 0x9001, 0x8030, 0x6040, 0x3060, 0x0050, 0x0041,
    0x0023, 0x0004, 0x0004, 0x0004, 0x4003, 0x0000, 0x0000, 0x0000,
    0x9099, 0xC005, 0xF033, 0xE062, 0xB081, 0x60A1, 0x2092, 0x0074,
    0x0056, 0x0027, 0x0008, 0x2007, 0x7006, 0x0000, 0x0000, 0x0000,
    0xF0FF, 0xF05A, 0xF078, 0xF0B6, 0xF0E5, 0xB0F5, 0x60F7, 0x20D8,
    0x00AB, 0x007C, 0x205D, 0x703D, 0xD03B, 0x4044, 0x0000, 0x0000,
    0xF0FF, 0xF0AD, 0xF0CC, 0xF0DB, 0xF0FB, 0xD0FB, 0xB0FB, 0x90EC,
    0x70DD, 0x70BE, 0x90AE, 0xB09E, 0xE0AD, 0xA0AA, 0x0000, 0x0000,
};
const uint16_t COLOR_TABLE_1[] = {
    0x5505, 0x2700, 0x1900, 0x0803, 0x0604, 0x0306, 0x0005, 0x1004,
    0x3002, 0x4000, 0x4000, 0x4000, 0x3400, 0x0000, 0x0000, 0x0000,
    0x9909, 0x5C00, 0x3F03, 0x2E06, 0x1B08, 0x160A, 0x2209, 0x4007,
    0x6005, 0x7002, 0x8000, 0x7200, 0x6700, 0x0000, 0x0000, 0x0000,
    0xFF0F, 0xAF05, 0x8F07, 0x6F0B, 0x5F0E, 0x5B0F, 0x760F, 0x820D,
    0xB00A, 0xC007, 0xD205, 0xD703, 0xBD03, 0x4404, 0x0000, 0x0000,
    0xFF0F, 0xDF0A, 0xCF0C, 0xBF0D, 0xBF0F, 0xBD0F, 0xBB0F, 0xC90E,
    0xD70D, 0xE70B, 0xE90A, 0xEB09, 0xDE0A, 0xAA0A, 0x0000, 0x0000,
};

const xmc::input::Button KEY_ASSIGN[8] = {
    xmc::input::Button::B,    xmc::input::Button::Y,
    xmc::input::Button::X,    xmc::input::Button::A,
    xmc::input::Button::UP,   xmc::input::Button::DOWN,
    xmc::input::Button::LEFT, xmc::input::Button::RIGHT,
};

uint8_t ppuLineBuff[shapones::SCREEN_WIDTH];
bool displayTransferRequested = false;
bool displayTransferInProgress = false;

constexpr uint32_t AUDIO_LATENCY_SAMPLES = 512;
uint8_t audioBuff[AUDIO_LATENCY_SAMPLES];
int32_t audioDcOffset = 0;

bool core1Loop();
void convertColor(int y);
void audioCallback(void *buffer, uint32_t numSamples, void *context);

AppConfig xmcAppGetConfig(void) {
  auto cfg = getDefaultAppConfig();
  cfg.displayPixelFormat = DISPLAY_FORMAT;
  cfg.speakerSampleFormat = xmc::audio::SampleFormat::LINEAR_PCM_S16_MONO;
  cfg.speakerLatencySamples = AUDIO_LATENCY_SAMPLES;
  cfg.speakerEnabled = true;
  return cfg;
}

void xmcAppSetup(void) {
  auto cfg = shapones::get_default_config();
  cfg.apu_sampling_rate = xmc::speaker::getStreamFormat().rateHz;
  shapones::init(cfg);
  shapones::menu::show();

  xmc::audio::SourcePort audioPort;
  audioPort.callback = audioCallback;
  audioPort.context = nullptr;
  xmc::speaker::setSourcePort(&audioPort);

  startCore1(core1Loop);
  xmc::speaker::setMuted(false);
}

void xmcAppLoop(void) {
  if (xmc::input::wasPressed(xmc::input::Button::FUNC)) {
    if (shapones::menu::is_shown()) {
      shapones::menu::hide();
    } else {
      shapones::menu::show();
    }
  }

  shapones::input::status_t inputStatus;
  inputStatus.raw = 0;
  for (int i = 0; i < 8; i++) {
    if (xmc::input::isPressed(KEY_ASSIGN[i])) {
      inputStatus.raw |= (1 << i);
    }
  }
  shapones::input::set_status(0, inputStatus);

  for (int i = 0; i < 10; i++) {
    shapones::cpu::service();
  }

  if (displayTransferRequested) {
    displayTransferRequested = false;
    if (displayTransferInProgress) {
      while (xmc::spi::dmaIsBusy()) {
        shapones::cpu::service();
      }
      xmc::completeTransferToDisplay();
      displayTransferInProgress = false;
    }
    xmc::startTransferToDisplay(frameBuffer, 0, 0);
    displayTransferInProgress = true;
  }
}

bool core1Loop() {
  bool frameSkip = fpsKeeper.isFrameSkipping();
  shapones::ppu::status_t ppuStatus;
  shapones::ppu::service(ppuLineBuff, frameSkip, &ppuStatus);
  if (ppuStatus.is_end_of_visible_line() && !frameSkip) {
    convertColor(ppuStatus.focus_y);
  }
  if (ppuStatus.is_end_of_visible_area()) {
    if (!frameSkip) {
      displayTransferRequested = true;
    }
    fpsKeeper.waitVsync();
  }
  return true;
}

void convertColor(int y) {
  uint32_t *rdPtr =
      (uint32_t *)(ppuLineBuff + (shapones::SCREEN_WIDTH - display::WIDTH) / 2);
  uint32_t *wrPtr = (uint32_t *)frameBuffer->linePtr(y);
  for (int i = 0; i < display::WIDTH / 8; i++) {
    uint32_t in0 = *(rdPtr++);
    uint_fast16_t pix0 = COLOR_TABLE_0[(in0 >> 0) & 0xFF];
    uint_fast16_t pix1 = COLOR_TABLE_1[(in0 >> 8) & 0xFF];
    uint_fast16_t pix2 = COLOR_TABLE_0[(in0 >> 16) & 0xFF];
    uint_fast16_t pix3 = COLOR_TABLE_1[(in0 >> 24) & 0xFF];
    uint32_t in1 = *(rdPtr++);
    uint_fast16_t pix4 = COLOR_TABLE_0[(in1 >> 0) & 0xFF];
    uint_fast16_t pix5 = COLOR_TABLE_1[(in1 >> 8) & 0xFF];
    uint_fast16_t pix6 = COLOR_TABLE_0[(in1 >> 16) & 0xFF];
    uint_fast16_t pix7 = COLOR_TABLE_1[(in1 >> 24) & 0xFF];
    uint32_t out0 = 0;
    out0 |= pix0;
    out0 |= pix1 << 8;
    out0 |= pix2 << 24;
    *(wrPtr++) = out0;
    uint32_t out1 = 0;
    out1 |= pix2 >> 8;
    out1 |= pix3;
    out1 |= pix4 << 16;
    out1 |= pix5 << 24;
    *(wrPtr++) = out1;
    uint32_t out2 = 0;
    out2 |= pix5 >> 8;
    out2 |= pix6 << 8;
    out2 |= pix7 << 16;
    *(wrPtr++) = out2;
  }
}

void audioCallback(void *buffer, uint32_t numSamples, void *context) {
  shapones::apu::service(audioBuff, numSamples);

  // DC offset removal
  int16_t *outBuff = (int16_t *)buffer;
  for (uint32_t i = 0; i < numSamples; i++) {
    int32_t sample = audioBuff[i] * 256;
    audioDcOffset = (audioDcOffset * 255 + sample) / 256;
    sample = XMC_CLIP(-32768, 32767, sample - audioDcOffset);
    outBuff[i] = (int16_t)sample;
  }
}