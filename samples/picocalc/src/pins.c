/* PicoCalc pins, under the names the XIAO port's shared code uses.
   The SD card has SPI0 to itself here (the LCD is on SPI1, see lcd.cpp). */
#include "shapones/xiao/pins.h"

const int XIAO_I2C_SDA_PIN = 6;      /* keyboard (STM32) */
const int XIAO_I2C_SCL_PIN = 7;
const int XIAO_SPI_MOSI_PIN = 19;    /* SD card, SPI0 */
const int XIAO_SPI_MISO_PIN = 16;
const int XIAO_SPI_SCK_PIN = 18;
const int XIAO_TFCARD_CS_PIN = 17;
const int XIAO_DISPLAY_CS_PIN = 13;  /* unused by the shared code */
const int XIAO_DISPLAY_DC_PIN = 14;
const int XIAO_AUDIO_OUT_PIN = 26;
const int XIAO_POWER_BUTTON_PIN = -1;
const int XIAO_GPIO_PIN = -1;
