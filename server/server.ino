
extern "C" {
#include "osapi.h"
#include "ets_sys.h"
#include "user_interface.h"
}

#include <lwip/udp.h>

#include <ESP8266WiFi.h>

#include <NeoPixelBrightnessBus.h>

const char* ssid     = "Noisebridge Legacy 2.4 gHz";

#define BRIGHTNESS          96
#define NETWORK_TIMEOUT     10

const int NUM_LEDS_PER_STRIP[8] = {277, 277, 24, 39, 14, 9, 105, 65};

const uint16_t TOTAL_LEN = 810;
const uint8_t NUM_STRIPS = 8;
const int WIDTH = TOTAL_LEN;
const int HEIGHT = 1;

udp_pcb *_pcb;

bool unhandled = 0;

void recv(void *arg,
          udp_pcb *upcb, pbuf *p,
          ip_addr_t *addr, u16_t port);

#define IPADDR_ANY          ((u32_t)0x00000000UL)

static const int kBufferSize = 5000;
char packetBuffer[kBufferSize];
int packetSize = 0;

NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strips [] = {
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[0], 0),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[1], 1),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[2], 2),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[3], 3),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[4], 4),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[5], 5),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[6], 6),
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod>(NUM_LEDS_PER_STRIP[7], 7),
};

void set_led(const uint32_t addr, RgbColor col) {
  uint8_t strip_ix = 0;
  uint16_t strip_offset = 0;
  uint16_t current_count = 0;

  for (uint8_t i = 0; i < NUM_STRIPS; ++i) {
    if (current_count + NUM_LEDS_PER_STRIP[i] < addr) {
      current_count += NUM_LEDS_PER_STRIP[i];
    } else {
      strip_ix = i;
      strip_offset = NUM_LEDS_PER_STRIP[i] - (addr - current_count);
    }
  }

  strips[strip_ix].SetPixelColor(strip_offset, col);
}

void show_all() {
  for (uint8_t i = 0; i < NUM_STRIPS; ++i) {
    strips[i].Show();
  }
}

int last = 0;
int fps = 30;
int time_between_frames = 1000 / fps;

int last_packet = 0;
int max_incoming_fps = fps;
int min_packet_interval = 1000 / max_incoming_fps;

uint8_t gHue = 0; // rotating "base color" used by many of the patterns

struct ImageMetaInfo {
  int width;
  int height;
  int range;      // Range of gray-levels. We only handle 255 correctly(=1byte)

  // FlaschenTaschen extensions
  int offset_x;   // display image at this x-offset
  int offset_y;   // .. y-offset
  int layer;      // stacked layer
};

#define TIMECTL_MAXTICKS  4294967295L
#define TIMECTL_INIT      0

int IsTime(uint32_t *timeMark, uint32_t timeInterval) {
  uint32_t timeCurrent;
  uint32_t timeElapsed;
  int result = false;

  timeCurrent = millis();
  if (timeCurrent < *timeMark) { //Rollover detected
    timeElapsed = (TIMECTL_MAXTICKS - *timeMark) + timeCurrent; //elapsed=all the ticks to overflow + all the ticks since overflow
  }
  else {
    timeElapsed = timeCurrent - *timeMark;
  }

  if (timeElapsed >= timeInterval) {
    *timeMark = timeCurrent;
    result = true;
  }
  return (result);
}

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  for (uint8_t i = 0; i < NUM_STRIPS; ++i) {
    strips[i].SetBrightness(BRIGHTNESS);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  //   WiFi.begin(ssid, password);
}

void finish_wifi_connection() {
  _pcb = udp_new();
  udp_recv(_pcb, &recv, 0);
  ip_addr_t addr;
  addr.addr = IPADDR_ANY;
  int port = 1337;
  udp_bind(_pcb, &addr, port);
}

void loop() {
  static uint32_t lastHueUpdate = millis();
  static bool gotWifi = false;
  static uint32_t lastConnectionCheck = 0;

  if (!gotWifi) {
    if (IsTime(&lastConnectionCheck, 500)) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
      } else {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());

        Serial.println("listening to udp on port 1337");
        gotWifi = true;
      }
    }
  }

  if (unhandled) {

    int start = millis();
    ImageMetaInfo img_info = {0};
    img_info.width = WIDTH;
    img_info.height = HEIGHT;

    const char *pixel_pos = ReadImageData(packetBuffer, packetSize,
                                          &img_info);

    for (int x = 0; x < img_info.width; ++x) {
      const byte red = *pixel_pos++;
      const byte green  = *pixel_pos++;
      const byte blue = *pixel_pos++;

      set_led(x + img_info.offset_x, RgbColor(red, green, blue));
    }
    // If this runs too frequently, the network traffic can back up and overwhelm the network stack, which
    // doesn't seem to gracefully discard packets.
    if (millis() - last > time_between_frames) {
      show_all();
    }
    last = millis();
    unhandled = 0;

  } else if (millis() - last_packet > (1000 * NETWORK_TIMEOUT) ) {
    if (millis() - last < time_between_frames) {
      return;
    }
    int start = millis();
    rainbowWithGlitter();
    show_all();
  }

  if (IsTime(&lastHueUpdate, 20)) {
    gHue++;
  }
}

void recv(void *arg,
          udp_pcb *upcb, pbuf *p,
          ip_addr_t *addr, u16_t port) {
  packetSize = p->tot_len;
  pbuf_copy_partial(p, packetBuffer, kBufferSize, 0);
  pbuf_free(p);
  unhandled = 1;
  last_packet = millis();
}

static const char *skipWhitespace(const char *buffer, const char *end) {
  for (;;) {
    while (buffer < end && isspace(*buffer))
      ++buffer;
    if (buffer >= end)
      return NULL;
    if (*buffer == '#') {
      while (buffer < end && *buffer != '\n') // read to end of line.
        ++buffer;
      continue;  // Back to whitespace eating.
    }
    return buffer;
  }
}

// Read next number. Start reading at *start; modifies the *start pointer
// to point to the character just after the decimal number or NULL if reading
// was not successful.
static int readNextNumber(const char **start, const char *end) {
  const char *start_number = skipWhitespace(*start, end);
  if (start_number == NULL) {
    *start = NULL;
    return 0;
  }
  char *end_number = NULL;
  int result = strtol(start_number, &end_number, 10);
  if (end_number == start_number) {
    *start = NULL;
    return 0;
  }
  *start = end_number;
  return result;
}

const char *ReadImageData(const char *in_buffer, size_t buf_len,
                          struct ImageMetaInfo *info) {
  if (in_buffer[0] != 'P' || in_buffer[1] != '6' ||
      (!isspace(in_buffer[2]) && in_buffer[2] != '#')) {
    return in_buffer;  // raw image. No P6 magic header.
  }
  const char *const end = in_buffer + buf_len;
  const char *parse_buffer = in_buffer + 2;
  const int width = readNextNumber(&parse_buffer, end);
  if (parse_buffer == NULL) return in_buffer;
  const int height = readNextNumber(&parse_buffer, end);
  if (parse_buffer == NULL) return in_buffer;
  const int range = readNextNumber(&parse_buffer, end);
  if (parse_buffer == NULL) return in_buffer;
  if (!isspace(*parse_buffer++)) return in_buffer;   // last char before data
  // Now make sure that the rest of the buffer still makes sense
  const size_t expected_image_data = width * height * 3;
  const size_t actual_data = end - parse_buffer;
  if (actual_data < expected_image_data)
    return in_buffer;   // Uh, not enough data.
  if (actual_data > expected_image_data) {
    // Our extension: at the end of the binary data, we provide an optional
    // offset. We can't store it in the header, as it is fixed in number
    // of fields. But nobody cares what is at the end of the buffer.
    const char *offset_data = parse_buffer + expected_image_data;
    info->offset_x = readNextNumber(&offset_data, end);
    if (offset_data != NULL) {
      info->offset_y = readNextNumber(&offset_data, end);
    }
    if (offset_data != NULL) {
      info->layer = readNextNumber(&offset_data, end);
    }
  }
  info->width = width;
  info->height = height;
  info->range = range;
  return parse_buffer;
}

void fill_rainbow(
  uint8_t initialhue,
  uint8_t deltahue )
{
  uint8_t hue = initialhue;
  uint8_t val = 255;
  uint8_t sat = 240;
  for ( int i = 0; i < TOTAL_LEN; i++) {
    set_led(i, HsbColor(hue, sat, val));
    hue += deltahue;
  }
}

void rainbow()
{
  fill_rainbow(gHue, 7);
}

void rainbowWithGlitter()
{
  //Serial.println("rainbowWithGlitter()");
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( uint8_t chanceOfGlitter)
{
  if ( random(0, 255) < chanceOfGlitter) {
    set_led(random(0, TOTAL_LEN), RgbColor(255));
  }
}
