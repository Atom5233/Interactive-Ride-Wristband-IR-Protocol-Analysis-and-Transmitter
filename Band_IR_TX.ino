/*
 * band_ir_tx.ino
 *
 * IR code transmitter for the ESP32-C3 1.28" round LCD board
 * (Waveshare ESP32-C3-LCD-1.28 pinout, GC9A01 240x240 + optional CST816S touch)
 *
 * CONTROLS
 *   tap the top pill       : 38 kHz carrier on / off
 *   tap left / right arrow : previous / next code
 *   swipe left / right     : same thing
 *   double tap the middle  : start / stop transmitting
 *   swipe up / down        : trim the unit period +/- 1%
 *   press and hold ~1.5 s  : sleep. The backlight dips once armed, so
 *                            release then. Tap anywhere to wake.
 *
 *   Every function is reachable by touch alone, because the BOOT button is
 *   inside the case once assembled. The button bindings below are a bench
 *   convenience, not a requirement.
 *
 *   BOOT button, by how long it is held before release:
 *     tap (< 0.6 s)      : next code
 *     hold 0.6 - 1.6 s   : start / stop transmitting
 *     hold > 1.6 s       : sleep. Press again to wake.
 *
 *   Sleep is light sleep, not a true power cut: GPIO9 is not an RTC pin on
 *   the C3, so it cannot wake the chip from deep sleep. Light sleep keeps
 *   the pin live and costs a few hundred microamps.
 *
 *   With TOUCH_WAKE set, a tap on the screen also wakes it. TP_INT is GPIO0,
 *   which is RTC-capable, so it works as a wake source.
 *
 * Gestures are decoded in software from raw touch coordinates rather than
 * read from the CST816S gesture register, because gesture firmware varies
 * between batches of these panels.
 *
 * FRAME FORMAT
 *   Preamble : ON 3u, OFF 3u, ON 2u          (8 units, same on every code)
 *   Payload  : 10 cells x 2u                 (20 units)
 *                cell 0 -> OFF, OFF
 *                cell 1 -> OFF, ON
 *   Total    : 28 units, sent back to back with no inter-frame gap
 *
 * The base code does not sit on the cell grid, so everything is stored as a raw
 * unit schedule rather than as a bitmask.
 *
 * LIBRARIES
 *   "GFX Library for Arduino" by moononournation  (Library Manager: Arduino_GFX)
 *   Wire (bundled)
 *
 * BOARD
 *   ESP32C3 Dev Module. USB CDC On Boot: Enabled. Core 3.x required.
 *   Sunton ESP32-2424S012. The side switch is not a GPIO: it gates the
 *   battery rail at the charger's KEY pin, so it cannot be read or used to
 *   wake anything. USB power ignores it.
 *
 * RECOVERY WITH THE CASE CLOSED
 *   Slide the side switch off, then plug in USB. The board boots fresh on
 *   USB power and esptool can reset it into download mode over native USB,
 *   no BOOT button needed. FLASH_GRACE_MS keeps that window open.
 *
 *   GPIO21 (TX on the 4-pin connector) is unused and free for an external
 *   button if you want one outside the case: set SLEEP_BTN to 21.
 *
 * IR OUTPUT WIRING (GPIO20 = U0RXD, low-side N-channel MOSFET)
 *   GPIO20 --[100R]--> gate,  10k gate-to-source pulldown (do not omit)
 *   source -> GND
 *   drain  -> IR LED cathode, IR LED anode --[10R]--> 3V3
 *   10uF ceramic + 100nF from 3V3 to GND, physically at the LED, to keep
 *   the 38 kHz switching loop short. Not across the LED, which would fight
 *   the carrier. Do not omit the series resistor: without it the only limit
 *   is the MOSFET on-resistance and the LED will pull well over an amp.
 *   Non-inverting: gate high = LED on, same polarity the code assumes.
 *   Use a genuine logic-level part (AO3400, SI2302, IRLML2502).
 */

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

// ---------------------------------------------------------------- pins

#define LCD_SCK   6
#define LCD_MOSI  7
#define LCD_CS   10
#define LCD_DC    2
// The GC9A01 on this board has no reset line of its own; it relies on its
// power-on reset. GPIO1 is the CST816 touch reset, which is separate.
#define LCD_RST   GFX_NOT_DEFINED
#define LCD_BL    3

#define TP_SDA    4
#define TP_SCL    5
#define TP_INT    0
#define TP_RST    1
#define CST816S_ADDR 0x15

#define BOOT_BTN  9      // fallback for the no-touch variant
#define IR_PIN   20      // U0RXD, driving a low-side MOSFET

// Side button. Run pin_finder.ino to identify it, then set these two.
// Leave SLEEP_BTN at -1 and sleep stays on a long BOOT hold instead.
#define SLEEP_BTN        -1
#define SLEEP_ACTIVE_LOW true

// Wake on touch as well as on the button. TP_INT is GPIO0, one of the C3's
// RTC pins, so it is a valid wake source. Costs standby current: see notes.
#define TOUCH_WAKE       true

// During sleep, hand the CST816S to its own low-power mode. It saves roughly
// a milliamp, but whether it still asserts INT in that mode varies between
// batches of these panels, and if yours does not, touch will never wake the
// board. Left off so wake is guaranteed. Turn on only after testing.
#define TOUCH_SLEEP_LOWPOWER false

// Display orientation only: 0, 1 (90 CW), 2 (180), 3 (270).
// Back to 1: this is what looked right on the hardware. Changing this does
// NOT change the touch mapping below, which is set separately.
#define LCD_ROTATION 1

// Touch orientation, set independently of the display. These panels are not
// always bonded the same way round as the screen, so deriving one from the
// other does not work. Fix by symptom, changing one flag at a time:
//
//   taps land on the opposite side, left <-> right   -> flip TOUCH_FLIP_X
//   taps land on the opposite side, top <-> bottom   -> flip TOUCH_FLIP_Y
//   moving a finger sideways moves the point up and
//   down instead, or vice versa                      -> flip TOUCH_SWAP_XY
//
// Set TOUCH_DEBUG to 1 and a dot is drawn wherever the code thinks you
// touched, with raw and mapped coordinates on serial. Fastest way to get
// this right is to enable it, tap the four edges, and read the numbers.
#define TOUCH_SWAP_XY true
#define TOUCH_FLIP_X  true
#define TOUCH_FLIP_Y  true
#define TOUCH_DEBUG   0

// If left/right or up/down come out backwards, flip these.
#define INVERT_SWIPE_X  false
#define INVERT_SWIPE_Y  false

// ---------------------------------------------------------------- gestures

#define TOP_ZONE         58    // tap above this    -> toggle carrier
#define EDGE_L           72    // tap left of this  -> previous code
#define EDGE_R          168    // tap right of this -> next code
#define SWIPE_MIN_PX     40
#define SWIPE_MAX_MS    600
#define TAP_MAX_PX       25
#define DTAP_WINDOW_MS  400
#define SLEEPPRESS_MS  1500    // hold does one thing only: sleep
#define POLL_INTERVAL_MS 12

// BOOT button hold thresholds, acted on at release so a hold can escalate
#define BTN_TX_MS       600
#define BTN_PWR_MS     1600

// ---------------------------------------------------------------- power

#define CPU_MHZ         80   // 80 keeps APB at 80 MHz, so LEDC is unaffected
#define BL_PWM_HZ    20000
#define BL_ACTIVE      180   // of 255
#define BL_DIM          20
#define DIM_AFTER_MS 15000UL
#define OFF_AFTER_MS 45000UL
#define SLEEP_AFTER_MS 300000UL   // auto sleep when idle and not transmitting

// Sleep is refused for this long after every boot. With the case closed the
// BOOT button is unreachable, so USB CDC is the only way back in, and light
// sleep tears USB down. This guarantees a window where the board is always
// enumerable and flashable no matter what the rest of the firmware does.
#define FLASH_GRACE_MS 15000UL

enum { SCR_ACTIVE, SCR_DIM, SCR_OFF };
uint8_t  screenState = SCR_ACTIVE;
uint32_t lastActivityMs = 0;
volatile bool pendWake = false;

// ---------------------------------------------------------------- carrier

#define CARRIER_HZ    38000
#define LEDC_BITS     8
#define DUTY_CARRIER 128    // 50% of 255. Receivers decode happily at 50%,
                            // and it is half again the optical power of the
                            // 33% we started with, for free.
#define DUTY_DC      255    // solid on, carrier disabled

// CONFIRMED against the receiver: 38 kHz modulated works. This stays true.
bool carrierOn = true;

// ---------------------------------------------------------------- codes

enum Emblem { EM_TRIANGLE, EM_HEX, EM_STAR, EM_HOOD,
              EM_INSECT, EM_ARACHNID, EM_CLAW, EM_RING, EM_PROBE };

struct Code {
  const char *name;
  const char *item;
  uint16_t    cells;        // bit 9 = cell 1 ... bit 0 = cell 10, display only
  uint32_t    unitUs;       // measured unit period for this code
  uint8_t     sched[24];    // unit counts, alternating ON/OFF, starting ON
  uint8_t     schedLen;
  bool        offGrid;
  bool        guess;        // extrapolated, never captured from hardware
  uint8_t     emblem;
  uint16_t    tint;         // RGB565
};

// every schedule sums to 28 units
static const Code CODES[] = {
  { "Code 1",      "Field A = 1",      0b0010000000, 31750,
    {3,3,2, 5,1,14}, 6, false, false, EM_TRIANGLE, 0xFB40 },

  { "Code 2",      "Field A = 2",      0b0110000000, 31700,
    {3,3,2, 3,1,1,1,14}, 8, false, false, EM_HEX, 0x861F },

  { "Code 3",      "Field A = 3",      0b1110000000, 32700,
    {3,3,2, 1,1,1,1,1,1,14}, 10, false, false, EM_STAR, 0x07EF },

  { "Code 4",      "Field B = 1",      0b0000010000, 32200,
    {3,3,2, 11,1,8}, 6, false, false, EM_HOOD, 0xFC7F },

  { "Code 5",      "A = 1 + B = 2",    0b0010110000, 32200,
    {3,3,2, 5,1,3,1,1,1,8}, 10, false, false, EM_INSECT, 0xFD20 },

  { "Code 6",      "Field C = 1",      0b0000000010, 32200,
    {3,3,2, 17,1,2}, 6, false, false, EM_ARACHNID, 0xF800 },

  { "Base",        "No attachment",    0b0000110000, 32750,
    {3,3,2, 8,1,1,1,9}, 8, true, false, EM_RING, 0x9CF3 },
};
static const int NUM_CODES = sizeof(CODES) / sizeof(CODES[0]);

// --------------------------------------------------------------- probes
//
// Cell positions no captured code uses. Read as three fields of
// three cells each, every field filling leftward from its anchor:
//     A = 3,2,1     B = 6,5,4     C = 9,8,7
// Cell 10 is deliberately absent from this list. Lighting it would put an
// ON in the final unit, which runs straight into the next frame's 3-unit
// ON preamble and reads as a 4-unit one. It is very likely unusable by
// construction, which is a decent explanation for why nothing uses it.
//
// Schedules are built at runtime from the mask, so adding a line here is
// all it takes to test another guess.

// Set to 1 to expose the untested probe codes at the end of the list.
#define ENABLE_PROBES 1

struct Probe { const char *name; uint16_t cells; };

static const Probe PROBES[] = {
  // Ordered by what to try next. Cells 8+9 was tested against the receiver
  // and did nothing, ruling out field C = 2 under the leftward-fill reading, so
  // these relax that assumption one step at a time.
  { "C=3",     0b0000001110 },   // cells 7,8,9 - next value up in field C
  { "cell 8",  0b0000000100 },   // maybe cells stand alone rather than as fields
  { "cell 7",  0b0000001000 },
  { "C fwd",   0b0000000011 },   // cells 9,10 - field C filling rightward
  { "B=3",     0b0001110000 },   // cells 4,5,6
  { "cell 4",  0b0001000000 },
  { "A1 C1",   0b0010000010 },   // combinations, the same shape as code 5
  { "B1 C1",   0b0000010010 },
  { "A1 B1",   0b0010010000 },
  { "cell 10", 0b0000000001 },
};
static const int NUM_PROBES = sizeof(PROBES) / sizeof(PROBES[0]);
static const int TOTAL_ENTRIES = NUM_CODES + (ENABLE_PROBES ? NUM_PROBES : 0);

// Expand a 10-cell mask into a unit schedule. Preamble, then each cell as
// OFF+OFF or OFF+ON, with consecutive OFF units merged into single runs.
uint8_t buildSched(uint16_t cells, uint8_t *out) {
  uint8_t n = 0, off = 0;
  out[n++] = 3; out[n++] = 3; out[n++] = 2;
  for (int i = 0; i < 10; i++) {
    if (cells & (1 << (9 - i))) { out[n++] = off + 1; out[n++] = 1; off = 0; }
    else off += 2;
  }
  if (off) out[n++] = off;
  return n;
}

static Code probeBuf;

const Code &entry(int i) {
  if (i < 0 || i >= TOTAL_ENTRIES) return CODES[0];
  if (i < NUM_CODES) return CODES[i];
  const Probe &pr = PROBES[i - NUM_CODES];
  probeBuf.name     = pr.name;
  probeBuf.item     = "untested probe";
  probeBuf.cells    = pr.cells;
  probeBuf.unitUs   = 32200;
  probeBuf.schedLen = buildSched(pr.cells, probeBuf.sched);
  probeBuf.offGrid  = false;
  probeBuf.guess    = true;
  probeBuf.emblem   = EM_PROBE;
  probeBuf.tint     = 0x07FF;
  return probeBuf;
}

int  sel = 0;
bool txEnabled = false;
int  trimPct = 0;

// ---------------------------------------------------------------- display

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, LCD_RST, LCD_ROTATION, true /* IPS */);

#define COL_BG    0x0000
#define COL_TEXT  0xFFFF
#define COL_DIM   0x8410
#define COL_FAINT 0x4208
#define COL_ON    0xF81F
#define COL_OFF   0x2124
#define COL_GOOD  0x07E0
#define COL_WARN  0xFD20

bool needsRedraw = true;

// ---------------------------------------------------------------- input

volatile bool pendNext = false, pendPrev = false;
volatile bool pendCarrier = false, pendTx = false;
volatile bool pendTrimUp = false, pendTrimDown = false;
volatile bool abortFrame = false;
volatile bool pendPower = false;

bool touchPresent = false;

bool     tDown = false;
uint8_t  tMiss = 0;         // consecutive empty reads before calling it a lift
uint16_t tX0 = 0, tY0 = 0, tX = 0, tY = 0;
uint32_t tT0 = 0;
uint8_t  tLongArmed = 0;    // 0 none, 1 carrier toggle, 2 sleep
uint32_t lastTapMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastActionMs = 0;

bool     btnDown = false;
uint32_t btnT0 = 0;
uint8_t  btnArmed = 0;      // 0 = tap, 1 = tx toggle, 2 = sleep

bool     sideDown = false;
uint32_t sideT0 = 0;

static inline bool sidePressed() {
#if SLEEP_BTN >= 0
  return digitalRead(SLEEP_BTN) == (SLEEP_ACTIVE_LOW ? LOW : HIGH);
#else
  return false;
#endif
}

// -------------------------------------------------------------------------

void carrierWrite(bool on) {
  if (!on) {
    ledcWrite(IR_PIN, 0);
  } else {
    ledcWrite(IR_PIN, carrierOn ? DUTY_CARRIER : DUTY_DC);
  }
}

void requestTxToggle() {
  pendTx = true;
  if (txEnabled) abortFrame = true;   // stop mid-frame rather than waiting ~900 ms
}

// The CST816S always reports raw panel coordinates regardless of what the
// display driver is doing, so rotate them into screen space by hand.
static void remapTouch(uint16_t rx, uint16_t ry, uint16_t *x, uint16_t *y) {
  uint16_t a = rx, b = ry;
  if (TOUCH_SWAP_XY) { uint16_t t = a; a = b; b = t; }
  if (TOUCH_FLIP_X)  a = (a > 239) ? 0 : 239 - a;
  if (TOUCH_FLIP_Y)  b = (b > 239) ? 0 : 239 - b;
  *x = a; *y = b;
}

bool readTouch(uint16_t *x, uint16_t *y) {
  Wire.beginTransmission(CST816S_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)CST816S_ADDR, (uint8_t)5) != 5) return false;

  uint8_t fingers = Wire.read();
  uint8_t xh = Wire.read(), xl = Wire.read();
  uint8_t yh = Wire.read(), yl = Wire.read();
  if (fingers == 0) return false;

  uint16_t rx = ((xh & 0x0F) << 8) | xl;
  uint16_t ry = ((yh & 0x0F) << 8) | yl;
  remapTouch(rx, ry, x, y);
#if TOUCH_DEBUG
  Serial.printf("raw %3u,%3u  ->  screen %3u,%3u\n", rx, ry, *x, *y);
#endif
  return true;
}

void onSwipe(int dx, int dy) {
  if (INVERT_SWIPE_X) dx = -dx;
  if (INVERT_SWIPE_Y) dy = -dy;

  if (abs(dx) > abs(dy)) {
    if (dx > 0) pendNext = true;
    else        pendPrev = true;
  } else {
    if (dy > 0) pendTrimDown = true;
    else        pendTrimUp = true;
  }
}

// Edge taps change code immediately. Only taps in the middle band of the
// screen count toward the double tap, so one can't trigger the other.
void onTap(uint16_t x, uint16_t y) {
  uint32_t now = millis();

  if (y < TOP_ZONE)    { pendCarrier = true; lastTapMs = 0; return; }
  if (x < EDGE_L)      { pendPrev = true; lastTapMs = 0; return; }
  if (x > EDGE_R)      { pendNext = true; lastTapMs = 0; return; }

  if (now - lastTapMs < DTAP_WINDOW_MS) {
    requestTxToggle();
    lastTapMs = 0;
  } else {
    lastTapMs = now;
  }
}

// Safe to call at any point, including mid-frame: it only sets flags, and the
// carrier is generated in hardware so an I2C read cannot disturb pulse shape.
void pollInput() {
  uint32_t now = millis();
  if (now - lastPollMs < POLL_INTERVAL_MS) return;
  lastPollMs = now;

  if (touchPresent) {
    uint16_t x, y;
    bool contact = readTouch(&x, &y);

    if (contact) { lastActivityMs = now; tMiss = 0; }

#if TOUCH_DEBUG
    // Mark where the code believes the finger is. If the dot does not follow
    // your finger, the three flags above are wrong.
    if (contact && !txEnabled && screenState == SCR_ACTIVE)
      gfx->fillCircle(x, y, 4, COL_ON);
#endif

    if (contact && !tDown) {
      tDown = true; tX0 = x; tY0 = y; tT0 = now; tLongArmed = 0;
      tX = x; tY = y;
    } else if (contact && tDown) {
      tX = x; tY = y;
      int dx = (int)x - (int)tX0, dy = (int)y - (int)tY0;
      bool still = (abs(dx) < TAP_MAX_PX && abs(dy) < TAP_MAX_PX);
      // Holding does exactly one thing now: sleep. Nothing fires until the
      // finger lifts, and the backlight dips once it is armed.
      if (still && screenState == SCR_ACTIVE && !tLongArmed
          && now - tT0 > SLEEPPRESS_MS) {
        tLongArmed = 1;
        ledcWrite(LCD_BL, BL_DIM);
      }
    } else if (!contact && tDown && ++tMiss >= 2) {
      // The CST816S drops the occasional report mid-touch. Requiring two
      // empty reads stops one dropped frame being read as a finger lift,
      // which used to split a single tap into two partial gestures.
      tMiss = 0;
      tDown = false;
      uint32_t dt = now - tT0;
      int dx = (int)tX - (int)tX0, dy = (int)tY - (int)tY0;

      if (tLongArmed) {
        pendPower = true;
        abortFrame = true;
        lastActionMs = now;
      } else if (now - lastActionMs > 120) {
        if (screenState != SCR_ACTIVE) {
          // First touch after the screen dimmed only wakes it, so you never
          // change code by accident while reaching for a dark display.
          pendWake = true;
          lastActionMs = now;
        } else if (dt < SWIPE_MAX_MS && (abs(dx) > SWIPE_MIN_PX || abs(dy) > SWIPE_MIN_PX)) {
          onSwipe(dx, dy);
          lastActionMs = now;
          lastTapMs = 0;
        } else {
          // Anything that is not a swipe counts as a tap. Previously a touch
          // that drifted between TAP_MAX_PX and SWIPE_MIN_PX matched neither
          // test and was silently dropped, which is why the pill sometimes
          // ignored you.
          onTap(tX0, tY0);
          lastActionMs = now;
        }
      }
    }
  }

  // Side button: a single press sleeps, debounced at 40 ms.
  if (sidePressed() && !sideDown) {
    sideDown = true; sideT0 = now; lastActivityMs = now;
  } else if (!sidePressed() && sideDown) {
    sideDown = false;
    if (now - sideT0 > 40) { pendPower = true; abortFrame = true; }
  }

  // Held duration selects the action; it fires on release, so continuing to
  // hold escalates tap -> transmit -> sleep rather than committing early.
  bool pressed = (digitalRead(BOOT_BTN) == LOW);
  if (pressed && !btnDown) {
    btnDown = true; btnT0 = now; btnArmed = 0; lastActivityMs = now;
  } else if (pressed && btnDown) {
    uint32_t held = now - btnT0;
    if (btnArmed < 1 && held > BTN_TX_MS)  btnArmed = 1;
    if (btnArmed < 2 && held > BTN_PWR_MS) btnArmed = 2;
  } else if (!pressed && btnDown) {
    btnDown = false;
    if      (btnArmed == 2) { pendPower = true; abortFrame = true; }
    else if (screenState != SCR_ACTIVE) pendWake = true;
    else if (btnArmed == 1) requestTxToggle();
    else                    pendNext = true;
  }
}

void applyPending() {
  if (pendNext)     { sel = (sel + 1) % TOTAL_ENTRIES;             pendNext = false;     needsRedraw = true; }
  if (pendPrev)     { sel = (sel + TOTAL_ENTRIES - 1) % TOTAL_ENTRIES; pendPrev = false;     needsRedraw = true; }
  if (pendCarrier)  { carrierOn = !carrierOn;                  pendCarrier = false;  needsRedraw = true; }
  if (pendTx)       { txEnabled = !txEnabled;                  pendTx = false;       needsRedraw = true; }
  if (pendTrimUp)   { if (trimPct <  6) trimPct++;             pendTrimUp = false;   needsRedraw = true; }
  if (pendTrimDown) { if (trimPct > -6) trimPct--;             pendTrimDown = false; needsRedraw = true; }
  abortFrame = false;
}

// Deadline-based wait so rounding never accumulates across a frame.
void waitUntil(uint32_t deadline) {
  while ((int32_t)(micros() - deadline) < 0) {
    if (abortFrame) return;
    if ((int32_t)(deadline - micros()) > 5000) pollInput();
  }
}

void transmitFrame(const Code &c) {
  uint32_t unitUs = (uint32_t)((float)c.unitUs * (1.0f + trimPct / 100.0f));
  uint32_t t = micros();
  bool level = true;

  for (uint8_t i = 0; i < c.schedLen; i++) {
    carrierWrite(level);
    t += (uint32_t)c.sched[i] * unitUs;
    waitUntil(t);
    if (abortFrame) break;
    level = !level;
  }
  carrierWrite(false);
}

// Backlight dimming and blanking. Called only between frames, never inside
// transmitFrame, because displayOn/Off writes over SPI and would cost
// milliseconds in the middle of a 32 ms pulse.
void updateScreenPower() {
  uint8_t want;
  uint32_t idle = millis() - lastActivityMs;

  if (pendWake) {
    pendWake = false;
    lastActivityMs = millis();
    want = SCR_ACTIVE;
  } else if (idle > OFF_AFTER_MS) {
    want = SCR_OFF;
  } else if (idle > DIM_AFTER_MS) {
    want = SCR_DIM;
  } else {
    want = SCR_ACTIVE;
  }

  if (want == screenState) return;

  if (want == SCR_OFF) {
    ledcWrite(LCD_BL, 0);
    gfx->displayOff();
  } else {
    if (screenState == SCR_OFF) {
      gfx->displayOn();
      needsRedraw = true;
    }
    ledcWrite(LCD_BL, want == SCR_ACTIVE ? BL_ACTIVE : BL_DIM);
  }
  screenState = want;
}

// Soft power down. GPIO9 is not an RTC pin on the ESP32-C3, so deep sleep
// cannot wake from it; light sleep can, and keeps RAM so we resume in place.
void powerDown() {
  txEnabled = false;
  carrierWrite(false);

  gfx->fillScreen(COL_BG);
  gfx->displayOff();              // GC9A01 sleep-in
  ledcWrite(LCD_BL, 0);
  Serial.println("sleeping");
  Serial.flush();

#if SLEEP_BTN >= 0
  const gpio_num_t wakePin = (gpio_num_t)SLEEP_BTN;
  const gpio_int_type_t wakeLvl = SLEEP_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL
                                                   : GPIO_INTR_HIGH_LEVEL;
#else
  const gpio_num_t wakePin = (gpio_num_t)BOOT_BTN;
  const gpio_int_type_t wakeLvl = GPIO_INTR_LOW_LEVEL;
#endif

  // Do not sleep while the button is still down, or the level that wakes us
  // is already present and we return immediately.
  while (digitalRead(wakePin) == (wakeLvl == GPIO_INTR_LOW_LEVEL ? LOW : HIGH))
    delay(10);
  delay(80);

  gpio_wakeup_enable(wakePin, wakeLvl);

  bool touchWake = false;
#if TOUCH_WAKE
  if (touchPresent) {
#if TOUCH_SLEEP_LOWPOWER
    Wire.beginTransmission(CST816S_ADDR);
    Wire.write(0xFE); Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
#endif
    // Never sleep while INT is already asserted or we wake instantly.
    uint32_t guard = millis();
    while (digitalRead(TP_INT) == LOW && millis() - guard < 1000) delay(10);
    gpio_wakeup_enable((gpio_num_t)TP_INT, GPIO_INTR_LOW_LEVEL);
    touchWake = true;
  }
#endif

  esp_sleep_enable_gpio_wakeup();
  esp_light_sleep_start();
  // ---- execution resumes here on the next press or touch ----
  gpio_wakeup_disable(wakePin);

  if (touchWake) {
    gpio_wakeup_disable((gpio_num_t)TP_INT);
#if TOUCH_SLEEP_LOWPOWER
    // Back to continuous reporting so gestures work normally again.
    Wire.beginTransmission(CST816S_ADDR);
    Wire.write(0xFE); Wire.write(0x01);
    Wire.endTransmission();
#endif
  }

  ledcWrite(LCD_BL, BL_ACTIVE);
  gfx->displayOn();
  screenState = SCR_ACTIVE;
  lastActivityMs = millis();
  Serial.println("awake");

  while (digitalRead(wakePin) == (wakeLvl == GPIO_INTR_LOW_LEVEL ? LOW : HIGH))
    delay(10);

  // Wait for the finger that woke us to lift, so the wake touch is not also
  // read as a tap once the main loop resumes.
  if (touchWake) {
    uint16_t dx, dy;
    uint32_t guard = millis();
    while (readTouch(&dx, &dy) && millis() - guard < 3000) delay(10);
  }
  delay(120);
  btnDown = false; btnArmed = 0;
  sideDown = false; tDown = false;
  lastTapMs = 0; lastActionMs = millis();
  pendNext = pendPrev = pendTx = pendCarrier = false;
  pendTrimUp = pendTrimDown = false;
  needsRedraw = true;
}

// -------------------------------------------------------------------- art

// Unit vectors for 8 directions, x100 so radii scale with integer maths
static const int8_t U_X[8] = { 100,  71,   0, -71, -100, -71,    0,  71 };
static const int8_t U_Y[8] = {   0,  71, 100,  71,    0, -71, -100, -71 };

static inline int px(int cx, int i, int r) { return cx + U_X[i] * r / 100; }
static inline int py(int cy, int i, int r) { return cy + U_Y[i] * r / 100; }

// Highlight colour, derived from the code's own tint rather than a fixed one.
// A single shared accent fought every tint it was laid over; moving each
// channel 60% toward full keeps the highlight in the same family as the mark.
static uint16_t lighten(uint16_t c) {
  uint16_t r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
  r += (31 - r) * 3 / 5;
  g += (63 - g) * 3 / 5;
  b += (31 - b) * 3 / 5;
  return (r << 11) | (g << 5) | b;
}

// Original geometric marks, not reproductions of any licensed logo.
void drawEmblem(uint8_t em, int cx, int cy, uint16_t c) {
  uint16_t a = lighten(c);

  // Shared outer frame, so every mark sits in the same ring.
  gfx->drawCircle(cx, cy, 26, c);
  gfx->drawCircle(cx, cy, 25, c);

  switch (em) {

    case EM_TRIANGLE:                       // segmented housing, triangle core
    case EM_HEX:                            // same housing, hexagonal core
      for (int i = 0; i < 8; i++)
        gfx->drawLine(px(cx, i, 24), py(cy, i, 24),
                      px(cx, i, 19), py(cy, i, 19), c);
      gfx->drawCircle(cx, cy, 18, c);
      if (em == EM_TRIANGLE) {
        gfx->fillTriangle(cx, cy - 12, cx - 11, cy + 7, cx + 11, cy + 7, c);
        gfx->fillCircle(cx, cy, 5, COL_BG);
        gfx->fillCircle(cx, cy, 3, a);
      } else {
        gfx->fillTriangle(cx, cy - 13, cx - 11, cy - 4, cx + 11, cy - 4, c);
        gfx->fillRect(cx - 11, cy - 4, 23, 9, c);
        gfx->fillTriangle(cx, cy + 13, cx - 11, cy + 5, cx + 11, cy + 5, c);
        gfx->fillCircle(cx, cy, 6, COL_BG);
        gfx->drawCircle(cx, cy, 6, a);
        gfx->fillCircle(cx, cy, 2, a);
      }
      break;

    case EM_STAR:                           // rings, ticks, interlocked triangles
      for (int i = 0; i < 8; i++)
        gfx->drawLine(px(cx, i, 24), py(cy, i, 24),
                      px(cx, i, 20), py(cy, i, 20), c);
      gfx->drawCircle(cx, cy, 19, c);
      gfx->drawTriangle(cx, cy - 16, cx - 14, cy + 8, cx + 14, cy + 8, c);
      gfx->drawTriangle(cx, cy + 16, cx - 14, cy - 8, cx + 14, cy - 8, c);
      gfx->fillCircle(cx, cy, 3, a);
      break;

    case EM_HOOD:                           // hood, angular lenses
      gfx->fillCircle(cx, cy - 5, 23, c);
      gfx->fillTriangle(cx - 23, cy - 5, cx + 23, cy - 5, cx, cy + 25, c);
      gfx->fillTriangle(cx - 19, cy - 12, cx - 3, cy - 8, cx - 14, cy + 3, COL_BG);
      gfx->fillTriangle(cx + 19, cy - 12, cx + 3, cy - 8, cx + 14, cy + 3, COL_BG);
      gfx->fillCircle(cx - 12, cy - 8, 2, a);
      gfx->fillCircle(cx + 12, cy - 8, 2, a);
      break;

    case EM_INSECT:                         // head, thorax, gaster, six legs
      gfx->fillCircle(cx, cy - 17, 5, c);
      gfx->drawLine(cx - 4, cy - 21, cx - 12, cy - 27, c);
      gfx->drawLine(cx - 12, cy - 27, cx - 17, cy - 25, c);
      gfx->drawLine(cx + 4, cy - 21, cx + 12, cy - 27, c);
      gfx->drawLine(cx + 12, cy - 27, cx + 17, cy - 25, c);
      gfx->fillEllipse(cx, cy - 5, 5, 8, c);
      gfx->fillEllipse(cx, cy + 14, 8, 11, c);
      for (int i = 0; i < 3; i++) {
        int y = cy - 11 + i * 7;
        gfx->drawLine(cx - 4, y, cx - 14, y - 3, c);
        gfx->drawLine(cx - 14, y - 3, cx - 21, y + 6, c);
        gfx->drawLine(cx + 4, y, cx + 14, y - 3, c);
        gfx->drawLine(cx + 14, y - 3, cx + 21, y + 6, c);
      }
      gfx->fillCircle(cx - 2, cy - 18, 1, a);
      gfx->fillCircle(cx + 2, cy - 18, 1, a);
      break;

    case EM_ARACHNID:                       // arachnid, eight jointed legs
      gfx->fillEllipse(cx, cy - 8, 6, 7, c);
      gfx->fillEllipse(cx, cy + 7, 8, 12, c);
      for (int i = 0; i < 4; i++) {
        int y = cy - 12 + i * 5;
        int reach = 15 + (i < 2 ? i * 3 : (3 - i) * 3 + 6);
        gfx->drawLine(cx - 5, y, cx - reach, y - 7, c);
        gfx->drawLine(cx - reach, y - 7, cx - reach - 4, y + 6, c);
        gfx->drawLine(cx + 5, y, cx + reach, y - 7, c);
        gfx->drawLine(cx + reach, y - 7, cx + reach + 4, y + 6, c);
      }
      gfx->fillCircle(cx - 3, cy - 9, 2, a);
      gfx->fillCircle(cx + 3, cy - 9, 2, a);
      break;

    case EM_CLAW:                           // four raking slashes
      for (int i = 0; i < 4; i++) {
        int x = cx - 20 + i * 9;
        int outer = (i == 0 || i == 3) ? 14 : 20;
        for (int t = 0; t < 2; t++) {
          gfx->drawLine(x + t, cy - outer, x + 8 + t, cy - 2, c);
          gfx->drawLine(x + 8 + t, cy - 2, x + 14 + t, cy + outer, c);
        }
      }
      break;

    case EM_PROBE:                          // unknown, ticked ring and a query
      for (int i = 0; i < 8; i++)
        gfx->drawLine(px(cx, i, 24), py(cy, i, 24),
                      px(cx, i, 19), py(cy, i, 19), c);
      gfx->setTextSize(3);
      gfx->setTextColor(a);
      gfx->setCursor(cx - 8, cy - 12);
      gfx->print("?");
      break;

    case EM_RING:                           // plain band, no attachment
    default:
      gfx->drawCircle(cx, cy, 14, c);
      gfx->drawCircle(cx, cy, 13, c);
      for (int i = 0; i < 8; i += 2)
        gfx->fillCircle(px(cx, i, 19), py(cy, i, 19), 3, a);
      break;
  }
}

// -------------------------------------------------------------------------

static void centerText(const char *t, int y, uint8_t size, uint16_t col) {
  gfx->setTextSize(size);
  gfx->setTextColor(col);
  gfx->setCursor((240 - (int)strlen(t) * 6 * size) / 2, y);
  gfx->print(t);
}

void drawScreen() {
  const Code &c = entry(sel);
  gfx->fillScreen(COL_BG);

  // Bezel doubles as the transmit indicator: green while sending, so state
  // is readable at a glance without finding any text.
  uint16_t ring = txEnabled ? COL_GOOD : COL_FAINT;
  gfx->drawCircle(120, 120, 118, ring);
  gfx->drawCircle(120, 120, 117, ring);

  // Carrier pill. Filled means modulated, outlined means raw DC.
  const char *cl = carrierOn ? "38 kHz" : "DC ONLY";
  int cw = (int)strlen(cl) * 6;
  if (carrierOn) {
    gfx->fillRoundRect(120 - cw / 2 - 11, 18, cw + 22, 22, 11, COL_GOOD);
    gfx->setTextColor(COL_BG);
  } else {
    gfx->drawRoundRect(120 - cw / 2 - 11, 18, cw + 22, 22, 11, COL_WARN);
    gfx->setTextColor(COL_WARN);
  }
  gfx->setTextSize(1);
  gfx->setCursor(120 - cw / 2, 25);
  gfx->print(cl);

  char meta[32];
  snprintf(meta, sizeof(meta), "%d / %d", sel + 1, TOTAL_ENTRIES);
  if (trimPct != 0) {
    char t[12];
    snprintf(t, sizeof(t), "   %+d%%", trimPct);
    strncat(meta, t, sizeof(meta) - strlen(meta) - 1);
  }
  if (c.guess) strncat(meta, "   ?", sizeof(meta) - strlen(meta) - 1);
  centerText(meta, 50, 1, c.guess ? COL_WARN : COL_FAINT);

  drawEmblem(c.emblem, 120, 92, c.tint);

  gfx->fillTriangle(14, 92, 26, 84, 26, 100, COL_FAINT);
  gfx->fillTriangle(226, 92, 214, 84, 214, 100, COL_FAINT);

  centerText(c.name, 126, 2, c.guess ? COL_WARN : COL_TEXT);
  centerText(c.item, 148, 1, COL_DIM);

  // Ten payload cells as dots, lit ones in the code's own colour.
  if (c.offGrid) {
    centerText("non-standard frame", 163, 1, COL_OFF);
  } else {
    for (int i = 0; i < 10; i++) {
      int x = 66 + i * 12;
      if (c.cells & (1 << (9 - i))) gfx->fillCircle(x, 166, 4, c.tint);
      else                          gfx->drawCircle(x, 166, 3, COL_OFF);
    }
  }

  if (txEnabled) {
    gfx->fillRoundRect(70, 178, 100, 26, 13, COL_GOOD);
    gfx->setTextColor(COL_BG);
  } else {
    gfx->drawRoundRect(70, 178, 100, 26, 13, COL_DIM);
    gfx->setTextColor(COL_DIM);
  }
  gfx->setTextSize(2);
  gfx->setCursor(78, 184);
  gfx->print(txEnabled ? "SENDING" : "STOPPED");

  centerText(txEnabled ? "double-tap to stop" : "double-tap to start",
             212, 1, COL_FAINT);

  Serial.printf("[%d/%d] %-13s %s  unit=%lu us  carrier=%s  tx=%s\n",
                sel + 1, TOTAL_ENTRIES, c.name, c.item,
                (unsigned long)((float)c.unitUs * (1.0f + trimPct / 100.0f)),
                carrierOn ? "38k" : "off",
                txEnabled ? "on" : "off");
}

void setup() {
  // Drop the core clock before anything else is configured. 80 MHz leaves the
  // APB clock at 80 MHz, so LEDC and SPI divisors are unchanged; going lower
  // would drag APB down with it and shift the 38 kHz carrier.
  setCpuFrequencyMhz(CPU_MHZ);

  Serial.begin(115200);
  delay(300);

  pinMode(BOOT_BTN, INPUT_PULLUP);
#if SLEEP_BTN >= 0
  pinMode(SLEEP_BTN, SLEEP_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
#endif

  ledcAttach(IR_PIN, CARRIER_HZ, LEDC_BITS);
  ledcWrite(IR_PIN, 0);

  ledcAttach(LCD_BL, BL_PWM_HZ, LEDC_BITS);
  ledcWrite(LCD_BL, BL_ACTIVE);
  gfx->begin();
  gfx->fillScreen(COL_BG);

  // Reset the touch controller explicitly rather than relying on the display
  // driver happening to toggle the same pin.
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);
  delay(10);
  digitalWrite(TP_RST, HIGH);
  delay(60);

  pinMode(TP_INT, INPUT_PULLUP);
  Wire.begin(TP_SDA, TP_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(CST816S_ADDR);
  touchPresent = (Wire.endTransmission() == 0);

  if (touchPresent) {
    // Disable auto-sleep, or the panel stops reporting after a few seconds
    // of no contact and swallows the first tap waking it up.
    Wire.beginTransmission(CST816S_ADDR);
    Wire.write(0xFE); Wire.write(0x01);
    Wire.endTransmission();
    Serial.println("CST816S found");
  } else {
    Serial.println("No touch, using BOOT button");
  }

  lastActivityMs = millis();
  drawScreen();
  needsRedraw = false;
}

void loop() {
  if (txEnabled) {
    transmitFrame(entry(sel));
  } else {
    // Yield rather than spin. delay() hands the core to the FreeRTOS idle
    // task, which parks it in WAITI instead of burning cycles in a loop.
    pollInput();
    delay(8);
  }

  updateScreenPower();

  if (!txEnabled && millis() > FLASH_GRACE_MS
      && millis() - lastActivityMs > SLEEP_AFTER_MS) {
    Serial.println("idle timeout");
    pendPower = true;
  }

  if (pendPower) {
    pendPower = false;
    abortFrame = false;
    if (millis() < FLASH_GRACE_MS) {
      Serial.println("sleep refused: still inside flash window");
      ledcWrite(LCD_BL, BL_ACTIVE);   // undo the armed dip
      lastActivityMs = millis();
      needsRedraw = true;
    } else {
      powerDown();
    }
  }

  applyPending();
  if (needsRedraw) {
    drawScreen();
    needsRedraw = false;
  }
}
