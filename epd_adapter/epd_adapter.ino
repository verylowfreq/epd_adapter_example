//Required: GxEPD2 by Jean-Mirc Zingg

#include <GxEPD2_BW.h>

#include <GxEPD2_3C.h>
#include <GxEPD2_4C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#include <Adafruit_TinyUSB.h>

#define GxEPD2_DISPLAY_CLASS GxEPD2_BW

#define GxEPD2_DRIVER_CLASS GxEPD2_213_Z98c // GDEY0213Z98 122x250, SSD1680, (FPC-A002 20.04.08)

#define MAX_DISPLAY_BUFFER_SIZE 1024ul
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))

#define PIN_EPD_CS SS
#define PIN_EPD_DC PA1
#define PIN_EPD_RES PA2
#define PIN_EPD_BUSY PA3

GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(
  GxEPD2_DRIVER_CLASS(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RES, PIN_EPD_BUSY));
  // GxEPD2_DRIVER_CLASS(/*CS=*/ D10, /*DC=*/ D8, /*RST=*/ D9, /*BUSY=*/ D7));


Adafruit_USBD_WebUSB usb_web;


constexpr int PIN_LED = PA8;


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(PIN_LED, OUTPUT);

  display.init(0, true, 2, false); // USE THIS for Waveshare boards with "clever" reset circuit, 2ms reset pulse
  display.setRotation(3);
  display.setFullWindow();

  // helloWorld();

  usb_web.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // display.hibernate();
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


const char HelloWorld[] = "Hello World!";

void helloWorld()
{
  display.setRotation(3);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(HelloWorld);
  }
  while (display.nextPage());
}


constexpr uint16_t IMG_W = 250;
constexpr uint16_t IMG_H = 122;
uint8_t imagebuf[((IMG_W + 7) / 8) * IMG_H];
size_t cursor = 0;

void loop() {
  led_task();

  process_stream_input_binary(usb_web);
  process_serial_input();
}


void process_stream_input_binary(Stream& stream) {
  if (stream.available() < 1) { return; }
  int cmd = stream.read();

  if (cmd == 0x80) {
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
    while (stream.available() < 2) { yield(); }
    len = (uint16_t)stream.read() + ((uint16_t)stream.read() << 8);
    while (len > 0) {
      while (stream.available() < 1) { yield(); }
      uint8_t b = stream.read();
      imagebuf[cursor] = b;
      cursor += 1;
      len -= 1;
      yield();
    }
    stream.write(0x82);
    stream.flush();

  } else if (cmd == 0x83) {
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
      display.drawBitmap(0, 0, imagebuf, IMG_W, IMG_H, GxEPD_BLACK);
    } while (display.nextPage());
    stream.write(0x83);
    stream.flush();

  } else if (cmd == 0x84) {
    uint8_t resp[16] = {
      0x84,  // Command byte echo
      14,    // Data length
      (IMG_W & 0xff), (IMG_W >> 8),   // Width as u16le
      (IMG_H & 0xff), (IMG_H >> 8),   // Height as u16le
      1                               // Bit per pixel
    };
    stream.write(resp, sizeof(resp));
    stream.flush();
  }
}

void process_serial_input() {
  if (Serial.available() > 0) {
    int b = Serial.peek();

    if (b >= 0x80) {
      process_stream_input_binary(Serial);
      return;
    }


    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.equals("clear")) {
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
      } while (display.nextPage());
      cursor = 0;
      Serial.printf("Cleared.\r\n");

    } else if (input.equals("test")) {
      Serial.printf("Draw hello world...\r\n");
      helloWorld();
      Serial.printf("Completed.\r\n");

    } else if (input.startsWith("put ")) {
      size_t hexlen = input.length() - 4;
      if (hexlen % 2 != 0) {
        Serial.printf("ERR odd hex length\r\n");
        return;
      }

      size_t len = hexlen / 2;
      if (cursor + len > sizeof(imagebuf)) {
        Serial.printf("ERR overflow %u + %u > %u\r\n", (unsigned)cursor, (unsigned)len, (unsigned)sizeof(imagebuf));
        return;
      }

      const uint8_t* hex = (const uint8_t*)&input.c_str()[4];
      load_hex(hex, len, &imagebuf[cursor]);
      cursor += len;
      Serial.printf("OK %u\r\n", (unsigned)cursor);

    } else if (input.equals("show")) {
      Serial.printf("Show...\r\n");
      display.firstPage();
      int i = 0;
      do {
        display.fillScreen(GxEPD_WHITE);
        display.drawBitmap(0, 0, imagebuf, IMG_W, IMG_H, GxEPD_BLACK);
        Serial.printf("Page %d\r\n", i);
        i += 1;
      } while (display.nextPage());
      Serial.printf("Completed.\r\n");
    }
  }
}

uint8_t hex2byte(const uint8_t* hex) {
  uint8_t ret = 0;
  char ch = hex[0];
  for (int i = 0; i < 2; i++) {
    if ('0' <= ch && ch <= '9') {
      ret += ch - '0';
    } else if ('a' <= ch && ch <= 'f') {
      ret += ch - 'a' + 10;
    } else if ('A' <= ch && ch <= 'F') {
      ret += ch - 'A' + 10;
    }
    if (i == 0) {
      ret = ret << 4;
      ch = hex[1];
    }
  }
  return ret;
}

void dump_hex(const uint8_t* data, size_t nbytes) {
  Serial.printf("--------\r\n");
  for (int i = 0; i < nbytes; i++) {
    if (i != 0 && i % 32 == 0) {
      Serial.printf("\r\n");
    }
    Serial.printf("%02x ", data[i]);
  }
  Serial.printf("\r\n--------\r\n");
}

void load_hex(const uint8_t* hextext, size_t nbytes, uint8_t* out) {
  while (nbytes > 0) {
    *out = hex2byte(hextext);
    hextext += 2;
    out += 1;
    nbytes -= 1;
  }
}
