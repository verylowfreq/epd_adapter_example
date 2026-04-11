// Required: GxEPD2 by Jean-Marc Zingg

#include <GxEPD2_BW.h>

#include <GxEPD2_3C.h>
#include <GxEPD2_4C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#include <Adafruit_TinyUSB.h>

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW

#define EPD_MODULE_420

// Select one module with a build flag, for example:
//   -DEPD_MODULE_154
//   -DEPD_MODULE_213
//   -DEPD_MODULE_420
#if !defined(EPD_MODULE_154) && !defined(EPD_MODULE_213) && !defined(EPD_MODULE_420)
#define EPD_MODULE_213
#endif

#if (defined(EPD_MODULE_154) ? 1 : 0) + (defined(EPD_MODULE_213) ? 1 : 0) + (defined(EPD_MODULE_420) ? 1 : 0) != 1
#error "Define exactly one of EPD_MODULE_154, EPD_MODULE_213, or EPD_MODULE_420."
#endif

#ifndef PIN_EPD_CS
#define PIN_EPD_CS SS
#endif

#ifndef PIN_EPD_DC
#define PIN_EPD_DC PA1
#endif

#ifndef PIN_EPD_RES
#define PIN_EPD_RES PA2
#endif

#ifndef PIN_EPD_BUSY
#define PIN_EPD_BUSY PA3
#endif

#ifdef EPD_MODULE_154
#define GxEPD2_DRIVER_CLASS GxEPD2_154_D67
constexpr uint8_t DISPLAY_ROTATION = 0;
constexpr const char* PANEL_NAME = "1.54in 200x200";
constexpr uint16_t PREFERRED_BAND_COUNT = 1;
#elif defined(EPD_MODULE_213)
#define GxEPD2_DRIVER_CLASS GxEPD2_213_BN
constexpr uint8_t DISPLAY_ROTATION = 3;
constexpr const char* PANEL_NAME = "2.13in 122x250";
constexpr uint16_t PREFERRED_BAND_COUNT = 1;
#else
#define GxEPD2_DRIVER_CLASS GxEPD2_420_GDEY042T81
constexpr uint8_t DISPLAY_ROTATION = 0;
constexpr const char* PANEL_NAME = "4.2in 400x300";
constexpr uint16_t PREFERRED_BAND_COUNT = 4;
#endif

constexpr size_t DISPLAY_BUFFER_BUDGET = 8ul * 1024ul;
constexpr uint16_t PANEL_NATIVE_W = GxEPD2_DRIVER_CLASS::WIDTH;
constexpr uint16_t PANEL_NATIVE_H = GxEPD2_DRIVER_CLASS::HEIGHT;
constexpr bool ROTATION_SWAPS_AXES = (DISPLAY_ROTATION & 1u) != 0;
constexpr uint16_t IMAGE_W = ROTATION_SWAPS_AXES ? PANEL_NATIVE_H : PANEL_NATIVE_W;
constexpr uint16_t IMAGE_H = ROTATION_SWAPS_AXES ? PANEL_NATIVE_W : PANEL_NATIVE_H;
constexpr size_t IMAGE_ROW_BYTES = (IMAGE_W + 7u) / 8u;
constexpr size_t MAX_IMAGE_DRAW_HEIGHT = (IMAGE_ROW_BYTES == 0u) ? 1u : (DISPLAY_BUFFER_BUDGET / IMAGE_ROW_BYTES);
constexpr size_t PREFERRED_IMAGE_DRAW_HEIGHT = (IMAGE_H + PREFERRED_BAND_COUNT - 1u) / PREFERRED_BAND_COUNT;
constexpr size_t IMAGE_DRAW_HEIGHT =
  (PREFERRED_IMAGE_DRAW_HEIGHT <= MAX_IMAGE_DRAW_HEIGHT) ? PREFERRED_IMAGE_DRAW_HEIGHT : MAX_IMAGE_DRAW_HEIGHT;
constexpr size_t CLAMPED_IMAGE_DRAW_HEIGHT =
  (IMAGE_DRAW_HEIGHT == 0u) ? 1u : ((IMAGE_DRAW_HEIGHT > IMAGE_H) ? IMAGE_H : IMAGE_DRAW_HEIGHT);
constexpr size_t IMAGEBUF_SIZE = IMAGE_ROW_BYTES * CLAMPED_IMAGE_DRAW_HEIGHT;
static_assert(IMAGEBUF_SIZE <= DISPLAY_BUFFER_BUDGET, "Image buffer must fit within 10 KB.");

// Keep GxEPD2 page buffer small; streamed bitmap data uses imagebuf below.
#define MAX_DISPLAY_BUFFER_SIZE 1024ul
#define MAX_HEIGHT(EPD) ((EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / ((EPD::WIDTH + 7) / 8)) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / ((EPD::WIDTH + 7) / 8)))

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(
  GxEPD2_DRIVER_CLASS(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RES, PIN_EPD_BUSY));

Adafruit_USBD_WebUSB usb_web;

constexpr int PIN_LED = PA8;
const char HelloWorld[] = "Hello World!";

uint8_t imagebuf[IMAGEBUF_SIZE];
size_t cursor = 0;
uint16_t draw_cursor_y = 0;

void led_task();
void helloWorld();
void process_stream_input_binary(Stream& stream);

uint16_t image_width() {
  return IMAGE_W;
}

uint16_t image_height() {
  return IMAGE_H;
}

void setup() {
  Serial.begin(115200);
  TinyUSBDevice.setProductDescriptor("EPD adapter");

  pinMode(PIN_LED, OUTPUT);

  display.init(0, true, 2, false);
  display.setRotation(DISPLAY_ROTATION);
  display.setFullWindow();

  usb_web.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

void loop() {
  led_task();
  process_stream_input_binary(usb_web);
}

void led_task() {
  static unsigned long timer = 0;
  static bool led_on = false;
  const unsigned long now = millis();
  unsigned int duration_on;
  unsigned int duration_off;
  if (usb_web.connected()) {
    duration_on = 100;
    duration_off = 900;
  } else {
    duration_on = 500;
    duration_off = 500;
  }
  if ((led_on && (now - timer >= duration_on)) ||
      (!led_on && (now - timer >= duration_off))) {
    led_on = !led_on;
    digitalWrite(PIN_LED, led_on ? HIGH : LOW);
    timer = now;
  }
}

void helloWorld() {
  display.setRotation(DISPLAY_ROTATION);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(HelloWorld);
  } while (display.nextPage());
}

void process_stream_input_binary(Stream& stream) {
  if (stream.available() < 1) {
    return;
  }
  int cmd = stream.read();

  if (cmd == 0x80) {
    draw_cursor_y = 0;
    display.setFullWindow();
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
    } while (display.nextPage());
    stream.write(0x80);
    stream.flush();

  } else if (cmd == 0x81) {
    cursor = 0;
    memset(imagebuf, 0x00, sizeof(imagebuf));
    stream.write(0x81);
    stream.flush();

  } else if (cmd == 0x82) {
    uint16_t len = 0;
    while (stream.available() < 2) {
      yield();
    }
    len = (uint16_t)stream.read() + ((uint16_t)stream.read() << 8);
    if (cursor + len > sizeof(imagebuf)) {
      while (len > 0) {
        while (stream.available() < 1) {
          yield();
        }
        stream.read();
        len -= 1;
      }
      stream.write(0x82);
      stream.flush();
      return;
    }
    while (len > 0) {
      while (stream.available() < 1) {
        yield();
      }
      imagebuf[cursor] = (uint8_t)stream.read();
      cursor += 1;
      len -= 1;
      yield();
    }
    stream.write(0x82);
    stream.flush();

  } else if (cmd == 0x83) {
    const uint16_t draw_h = min<uint16_t>(CLAMPED_IMAGE_DRAW_HEIGHT, image_height() - draw_cursor_y);
    if (draw_h == 0) {
      stream.write(0x83);
      stream.flush();
      return;
    }
    display.firstPage();
    display.setPartialWindow(0, draw_cursor_y, image_width(), draw_h);
    do {
      display.drawBitmap(0, draw_cursor_y, imagebuf, image_width(), draw_h, GxEPD_BLACK);
    } while (display.nextPage());
    draw_cursor_y += draw_h;
    stream.write(0x83);
    stream.flush();

  } else if (cmd == 0x84) {
    const uint16_t w = image_width();
    const uint16_t h = image_height();
    uint8_t resp[] = {
      0x84,
      7,
      (uint8_t)(w & 0xff), (uint8_t)(w >> 8),
      (uint8_t)(h & 0xff), (uint8_t)(h >> 8),
      1,
      (uint8_t)(CLAMPED_IMAGE_DRAW_HEIGHT & 0xff), (uint8_t)(CLAMPED_IMAGE_DRAW_HEIGHT >> 8),
    };
    stream.write(resp, sizeof(resp));
    stream.flush();
  }
}
