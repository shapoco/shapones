#pragma GCC optimize("O2")

#include <M5Unified.h>
#include <WiFi.h>
#include <AsyncUDP.h>

enum class state_t {
  DATA,
  AFTER_FF,
  START_OF_RUN,
};

static AsyncUDP udp;
static const uint16_t udp_port = 12345;
static const char *ssid = "ShapoNES_AP";
static const char *password = "hogepiyo";
static const IPAddress remote_ip(192, 168, 1, 100);

static constexpr uint32_t RECV_BUFF_SIZE = 32 * 1024;
static uint8_t recv_buff[RECV_BUFF_SIZE];
static volatile uint32_t recv_wr_ptr = 0;
static volatile uint32_t recv_rd_ptr = 0;

static constexpr int SCREEN_WIDTH = 256;
static constexpr int SCREEN_HEIGHT = 240;
static int focus_y = 0;
static int focus_x = 0;
uint8_t last_byte = 0;
uint8_t last_byte2 = 0;
state_t state;

static constexpr int BUFF_W = SCREEN_WIDTH / 2;
static constexpr int BUFF_H = SCREEN_HEIGHT / 2;
static uint16_t frame_buff[BUFF_W * BUFF_H];

// clang-format off
static const uint16_t COLOR_TABLE[] = {
  0xae73, 0xd120, 0x1500, 0x1340, 0x0e88, 0x02a8, 0x00a0, 0x4078, 0x6041, 0x2002, 0x8002, 0xe201, 0xeb19, 0x0000, 0x0000, 0x0000,
  0xf7bd, 0x9d03, 0xdd21, 0x1e80, 0x17b8, 0x0be0, 0x40d9, 0x61ca, 0x808b, 0xa004, 0x4005, 0x8704, 0x1104, 0x0000, 0x0000, 0x0000,
  0xffff, 0xff3d, 0x9f5b, 0x5fa4, 0xdff3, 0xb6fb, 0xacfb, 0xc7fc, 0xe7f5, 0x8286, 0xe94e, 0xd35f, 0x5b07, 0xae73, 0x0000, 0x0000,
  0xffff, 0x3faf, 0xbfc6, 0x5fd6, 0x3ffe, 0x3bfe, 0xf6fd, 0xd5fe, 0x34ff, 0xf4e7, 0x97af, 0xf9b7, 0xfe9f, 0xf7bd, 0x0000, 0x0000,
};
// clang-format on

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

#if defined(ARDUINO_M5STACK_STICKS3)
  // to supress power noise
  M5.Power.setExtOutput(false);
#endif

  Serial.printf("ESP-IDF Version: %s\n", esp_get_idf_version());

  Serial.printf("Connecting...\n");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.printf("Connected.\n");

  udp.listen(udp_port);
  udp.onPacket(onPacket);
}

void loop() {
  uint16_t *line = &frame_buff[(focus_y / 2) * BUFF_W];

  int last_focus_y = focus_y;

  uint32_t wp = recv_wr_ptr;
  uint32_t rp = recv_rd_ptr;

  bool vsync = false;
  for (int i = 0; i < 100; i++) {
    uint32_t usage = (wp - rp) & (RECV_BUFF_SIZE - 1);
    if (usage < 2) break;

    uint8_t b0 = recv_buff[rp];
    uint8_t b0h;
    rp = (rp + 1) & (RECV_BUFF_SIZE - 1);

    if (b0 == 0xFF) {
      uint8_t b1 = recv_buff[rp];
      rp = (rp + 1) & (RECV_BUFF_SIZE - 1);

      focus_y = b1 % SCREEN_HEIGHT;
      focus_x = 0;

      vsync = (focus_y < last_focus_y);
      line = &frame_buff[(focus_y / 2) * BUFF_W];
    } else {
      int n = ((b0 & 0xC0) >> 6) + 1;
      if (n >= 4) {
        uint8_t b1 = recv_buff[rp];
        rp = (rp + 1) & (RECV_BUFF_SIZE - 1);
        n = b1 + 1;
      }
      for (int j = 0; j < n; j++) {
        line[focus_x / 2] = COLOR_TABLE[b0 & 0x3F];
        focus_x = (focus_x + 1) % SCREEN_WIDTH;
      }
    }
  }

  recv_rd_ptr = rp;
  last_focus_y = focus_y;

  if (vsync) {
    int dx = (M5.Display.width() - BUFF_W) / 2;
    int dy = (M5.Display.height() - BUFF_H) / 2;
    M5.Display.endWrite();
    M5.Display.startWrite();
    M5.Display.pushImageDMA(dx, dy, BUFF_W, BUFF_H, frame_buff);
  }
}

void onPacket(AsyncUDPPacket packet) {
  if (packet.localPort() != udp_port) return;
  const uint8_t *data = packet.data();
  int len = packet.length();
  uint32_t wp = recv_wr_ptr;
  uint32_t rp = recv_rd_ptr;
  uint32_t usage = (wp - rp) & (RECV_BUFF_SIZE - 1);
  uint32_t space = RECV_BUFF_SIZE - usage - 1;
  if (len > space) return;

  uint32_t n = RECV_BUFF_SIZE - wp;
  if (len > n) {
    memcpy(recv_buff + wp, data, n);
    len -= n;
    data += n;
    wp = 0;
  }
  if (len > 0) {
    memcpy(recv_buff + wp, data, len);
    wp += len;
  }

  recv_wr_ptr = wp;
}
