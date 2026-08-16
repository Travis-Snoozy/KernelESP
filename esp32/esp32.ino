/*
   KernelESP v1.0
   A Linux-like interactive shell for the ESP32
  Tested on ESP32 - WROOM
 */

#include <Arduino.h>
#include <Console.h>
#include "argtable3/argtable3.h"
#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <driver/dac.h>
#include <driver/touch_pad.h>
#include <string.h>
#include <stdlib.h>

#include <list>

// Boards
#ifdef ARDUINO_ESP32S3_DEV
#define KESP_BOARD_SET
// Avoid the pins used for PSRAM/Flash (26-32, 33-38 octal), UART0 (15-16, 43-44),
// and USB (19-20), which we might be using.
// TODO: be smarter about including/excluding those pins based on further config defines
#define KESP_GPIO_OPI_MASK (BIT33 | BIT34 | BIT35 | BIT36 | BIT37)
#define KESP_GPIO_VALID_GPIO_MASK (SOC_GPIO_VALID_GPIO_MASK & ~(0ULL | BIT15 | BIT16 | BIT19 | BIT20 | BIT26 | BIT27 | BIT28 | BIT29 | BIT30 | BIT31 | BIT32 | KESP_GPIO_OPI_MASK | BIT43 | BIT44))
const int boardAdcPins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17};
const int boardTouchPins[] = {T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14};
const int boardDiscoPins[] = {2, 4, 5, 12, 13, 14, 17, 18, 21};
const int boardDacPins[] = {};
#endif

// Fallback (original "ESP32 WROOM" values)
#ifndef KESP_BOARD_SET
#define KESP_GPIO_VALID_GPIO_MASK (SOC_GPIO_VALID_GPIO_MASK & ~(0ULL | BIT0 | BIT1 | BIT3 | BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT20 | BIT37 | BIT38))
const int boardAdcPins[] = {36, 39, 34, 35, 32, 33, 25, 26, 27, 14};
const int boardTouchPins[] = {4, 0, 2, 15, 13, 12, 14, 27, 33, 32};
const int boardDiscoPins[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23};
const int boardDacPins[] = {DAC1, DAC2};
#endif

#define BOARD_ADC_COUNT (sizeof(boardAdcPins)/sizeof(int))
#define BOARD_TOUCH_COUNT (sizeof(boardTouchPins)/sizeof(int))
#define BOARD_DISCO_COUNT (sizeof(boardDiscoPins)/sizeof(int))
#define BOARD_DAC_COUNT (sizeof(boardDacPins)/sizeof(int))

#define BOARD_ADC_END (boardAdcPins+BOARD_ADC_COUNT)
#define BOARD_TOUCH_END (boardTouchPins+BOARD_TOUCH_COUNT)
#define BOARD_DISCO_END (boardDiscoPins+BOARD_DISCO_COUNT)
#define BOARD_DAC_END (boardDacPins+BOARD_DAC_COUNT)

// Take a default GPIO mask if the board didn't define one.
#ifndef KESP_GPIO_VALID_GPIO_MASK
#define KESP_GPIO_VALID_GPIO_MASK SOC_GPIO_VALID_GPIO_MASK
#endif

// Determine if a pin should be avoided/excluded for this board.
#define GPIO_AVOID(p) (p >= SOC_GPIO_PIN_COUNT || p < 0 || ((KESP_GPIO_VALID_GPIO_MASK & (1ULL << p)) == 0))

// Limits
#define CMD_LEN       256
#define MAX_ARGS      16
#define ARG_LEN       64
#define PATH_LEN      64
#define DMESG_LINES   12
#define DMESG_LEN     80
#define HOSTNAME      "kernelesp"

// ANSI helpers
#define RESET   "\033[0m"
#define BOLD    "\033[1m"
#define DIM     "\033[2m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define GRAY    "\033[90m"
#define BGREEN  "\033[92m"
#define BYELLOW "\033[93m"
#define BCYAN   "\033[96m"

// Pin management
typedef uint8_t pinCap_t;
#define PC_DIN    (1 << 0)
#define PC_DOUT   (1 << 1)
#define PC_LEDPWM (1 << 2)
#define PC_TOUCH  (1 << 3)
#define PC_ANALOG (1 << 4)
#define PC_DAC    (1 << 5)

// Global State
char  currentPath[PATH_LEN] = "/";
unsigned long bootTime;
pinCap_t  pinCapabilities[SOC_GPIO_PIN_COUNT];
pinCap_t  pinAssignment[SOC_GPIO_PIN_COUNT];

// Kernel log
struct DmesgEntry { unsigned long ts; char msg[DMESG_LEN]; };
DmesgEntry dmesgBuf[DMESG_LINES];
uint8_t dmesgHead = 0;
uint8_t dmesgCount = 0;

// WiFi state
bool   wifiConnected  = false;
bool   apActive       = false;
String staSSID        = "";
String apSSID         = HOSTNAME "_ap";
String apPASS         = "kernelesp";

// Web server (optional)
WebServer* httpServer = nullptr;
bool httpRunning = false;

// Kernel Log
void klog(const char* msg) {
  uint8_t idx = dmesgHead % DMESG_LINES;
  dmesgBuf[idx].ts = (millis() - bootTime) / 1000;
  strncpy(dmesgBuf[idx].msg, msg, DMESG_LEN - 1);
  dmesgBuf[idx].msg[DMESG_LEN - 1] = '\0';
  dmesgHead++;
  if (dmesgCount < DMESG_LINES) dmesgCount++;
}

// Utilities
int safeAtoi(const char* s) {
  if (!s || !*s) return 0;
  return (int)strtol(s, nullptr, 0); // handles 0x hex too
}

float safeAtof(const char* s) {
  if (!s || !*s) return 0.0f;
  return strtof(s, nullptr);
}

void strlowerBuf(char* s) {
  for (; *s; s++) if (*s >= 'A' && *s <= 'Z') *s += 32;
}

// Trim leading whitespace in-place, returns pointer into s
char* ltrim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  return s;
}

// Console class command adapter/convenience wrapper
template <typename T> class Command {
  private:
    // Wrap the core implementation of a command in boiler plate that
    // does the common parsing and error handling in a re-entrant safe
    // way, so that the implementation itself only has to focus on the
    // details of the command itself.
    static int execute(int argc, char** argv, void(*setArgs)(T&), int(*implementation)(int argc, char** argv, T& args)) {
      T args;
      int retval = 0;
      setArgs(args);
      if (arg_parse(argc, argv, (void**)&args) != 0) {
        arg_print_errors(stderr, args.end, argv[0]);
        retval = -1;
      } else {
        retval = implementation(argc, argv, args);
      }

      arg_freetable((void**)&args, sizeof(T)/sizeof(void*));
      return retval;
    }
  public:
    // Wrap Console.addCmd so that aliases can be registered without
    // room for copy/paste or incomplete maintenance errors.
    static void addCmd(std::list<const char*> names, const char* description, T &help) {
      T::setArgs(help);
      for (const char* name : names) {
        Console.addCmd(name, description, (void*)&help, Command<T>::wrapper);
      }
    }

    // Adapt the internal execute command, to the external Console callback API.
    static int wrapper(int argc, char** argv) {
      return Command<T>::execute(argc, argv, T::setArgs, T::implementation);
    }
};

// Basic command implementation template
/*
struct cmdNoop_args {
  // Add all command arguments here (in implicit order)
  arg_end_t *end;
  static void setArgs(struct cmdNoop_args &args) {
    // Set all arg_xxx_t* that were defined
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdNoop_args &args) {
    // Real command implementation goes here.
    return 0;
  }
};
static struct cmdNoop_args cmdNoopHelp;
*/


// Pin Resolution
void initPins() {
  for (int i = 0; i < SOC_GPIO_PIN_COUNT; i++) {
    pinAssignment[i] = 0;
    pinCapabilities[i] = 0;
    // Avoid pins get no assignment, and no capabilities.
    if (GPIO_AVOID(i)) { continue; }

    pinAssignment[i] = PC_DIN;   // All pins start as digital inputs.
    pinCapabilities[i] = PC_DIN; // All pins are valid digital inputs.
    // Mark pins capable of digital output.
    if (GPIO_IS_VALID_OUTPUT_GPIO(i)) {
      // All pins capable of digital output are also capable of LED PWM
      pinCapabilities[i] |= PC_DOUT;
      pinCapabilities[i] |= PC_LEDPWM;
    }
  }

  for (int i = 0; i < BOARD_ADC_COUNT; i++) {
      // Only add PC_ANALOG to un-avoided pins
      pinCapabilities[i] |= pinCapabilities[i] ? PC_ANALOG : 0;
  }

  for (int i = 0; i < BOARD_TOUCH_COUNT; i++) {
      // Only add PC_TOUCH to un-avoided pins
      pinCapabilities[i] |= pinCapabilities[i] ? PC_TOUCH : 0;
  }

  for (int i = 0; i < BOARD_DAC_COUNT; i++) {
    // Only add PC_DAC to un-avoided pins
    pinCapabilities[i] |= pinCapabilities[i] ? PC_DAC : 0;
  }
}

// Handle converting names to pin numbers (An, Tn, DACn), and checking
// if a given pin has any of the requested capabilities (output, ADC, 
// PWM, touch)
int resolvePin(const char* name, uint32_t capability_mask = 0xFFFFFFFF) {
  if (!name) return -1;
  int ch = -2;
  // No prefix -> plain pin number
  if (name[0] >= '0' && name[0] <= '9') { ch = safeAtoi(name); }
  // A prefix -> translate to a pin number (analog input)
  else if ((name[0] == 'A' || name[0] == 'a') && name[1]) {
    ch = safeAtoi(name + 1);
    ch = (ch < BOARD_ADC_COUNT) ? boardAdcPins[ch] : -1;
  }
  // T prefix -> translate to a pin number (touch input)
  else if ((name[0] == 'T' || name[0] == 't') && name[1]) {
    ch = safeAtoi(name + 1);
    ch = (ch < BOARD_TOUCH_COUNT) ? boardTouchPins[ch] : -1;
  }
  else if (
               (name [0] == 'D' || name[0] == 'd') &&
    name[1] && (name[1] == 'A' || name[1] == 'a') &&
    name[2] && (name[2] == 'C' || name[2] == 'c') &&
    name[3]) {
      ch = safeAtoi(name + 3);
      ch = (ch < BOARD_DAC_COUNT) ? boardDacPins[ch] : -1;
  }

  // Bail out if we know we have an invalid pin
  if (ch == -1) { return -1; }


  // We completed a translation; mask out any forbidden pins.
  if (ch >= 0) {
    return ((pinAssignment[ch] & capability_mask) == 0) ? -1 : ch;
  }

  // We did not complete a translation, keep parsing...
  // Note this is NOT typed or managed the way other outputs are!
#ifdef LED_BUILTIN
  if (strcasecmp(name, "LED") == 0) return LED_BUILTIN;   // Built-in LED most boards
#endif
  return -1;
}

// SPIFFS Filesystem Helpers
String buildPath(const char* name) {
  if (name[0] == '/') return String(name);
  String p = String(currentPath);
  if (!p.endsWith("/")) p += "/";
  p += name;
  return p;
}

void ensureDir(const String& dirPath) {
  // SPIFFS has no real directories — we track them via sentinel files
  String marker = dirPath;
  if (!marker.endsWith("/")) marker += "/";
  marker += ".dir";
  if (!SPIFFS.exists(marker)) {
    File f = SPIFFS.open(marker, FILE_WRITE);
    if (f) f.close();
  }
}

bool isDirectory(const String& path) {
  String marker = path;
  if (!marker.endsWith("/")) marker += "/";
  marker += ".dir";
  return SPIFFS.exists(marker);
}

void initFilesystem() {
  if (!SPIFFS.begin(true)) {
    Serial.println(RED "SPIFFS mount failed — formatting..." RESET);
    SPIFFS.format();
    SPIFFS.begin(true);
  }
  // Create default directories
  const char* defaults[] = {"/home", "/tmp", "/etc", "/dev"};
  for (auto d : defaults) ensureDir(String(d));
  klog("SPIFFS mounted OK");
}

// ASCII Logo
void showLogo() {
  Serial.print(F("\033[2J\033[H")); // clear screen, home
  Serial.println(F(CYAN
    "  ██╗  ██╗███████╗██████╗ ███╗   ██╗███████╗██╗     ███████╗███████╗██████╗ "  RESET));
  Serial.println(F(CYAN
    "  ██║ ██╔╝██╔════╝██╔══██╗████╗  ██║██╔════╝██║     ██╔════╝██╔════╝██╔══██╗" RESET));
  Serial.println(F(BCYAN
    "  █████╔╝ █████╗  ██████╔╝██╔██╗ ██║█████╗  ██║     █████╗  ███████╗██████╔╝" RESET));
  Serial.println(F(CYAN
    "  ██╔═██╗ ██╔══╝  ██╔══██╗██║╚██╗██║██╔══╝  ██║     ██╔══╝  ╚════██║██╔═══╝ " RESET));
  Serial.println(F(CYAN
    "  ██║  ██╗███████╗██║  ██║██║ ╚████║███████╗███████╗███████╗███████║██║      " RESET));
  Serial.println(F(GRAY
    "  ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚══════╝╚══════╝╚══════╝╚══════╝╚═╝  v1.0" RESET));
  Serial.println();

  // System info bar
  uint32_t heap = ESP.getFreeHeap();
  uint32_t flash = ESP.getFlashChipSize() / 1024;
  Serial.printf(GRAY "  Board: " WHITE ARDUINO_BOARD" @ %dMHz  " GRAY
                "RAM: " WHITE "%u KB free  " GRAY
                "Flash: " WHITE "%u KB" RESET "\n",
                ESP.getCpuFreqMHz(), heap / 1024, flash);

  // WiFi status
  if (wifiConnected) {
    Serial.printf(GRAY "  WiFi: " GREEN "Connected  " GRAY "IP: " WHITE "%s" RESET "\n",
                  WiFi.localIP().toString().c_str());
  } else if (apActive) {
    Serial.printf(GRAY "  WiFi: " YELLOW "AP Mode  " GRAY "IP: " WHITE "%s" RESET "\n",
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println(GRAY "  WiFi: " RED "Offline" RESET);
  }

  Serial.println(GRAY "  ─────────────────────────────────────────────────────────────────────────" RESET);
  Serial.println(GRAY "  Type " YELLOW "'help'" GRAY " for commands.  " YELLOW "'wifi help'" GRAY " for network commands." RESET "\n");
}

// Hardware Commands
struct cmdPinMode_args {
  arg_str_t *pin;
  arg_str_t *mode;
  arg_end_t *end;
  static void setArgs(struct cmdPinMode_args &args) {
    args.pin = arg_str1(NULL, NULL, "<pin>", "The pin to set the mode on");
    args.mode = arg_str1(NULL, NULL, "<input|output|pullup|pulldown|analog|touch|ledpwm|dac>", "The mode to assign to the pin");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdPinMode_args &args) {
    int pin = resolvePin(args.pin->sval[0]);
    if (pin < 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
    String m = String(args.mode->sval[0]); m.toLowerCase();
    if (m == "output" || m == "out") {
      if ((pinCapabilities[pin] & PC_DOUT) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_DOUT;
      pinMode(pin, OUTPUT);
      printf("GPIO%d → OUTPUT\n", pin);
    }
    else if (m == "input") {
      if ((pinCapabilities[pin] & PC_DIN) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_DIN;
      pinMode(pin, INPUT);
      printf("GPIO%d → INPUT\n", pin);
    }
    else if (m == "pullup"   || m == "input_pullup") {
      if ((pinCapabilities[pin] & PC_DIN) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_DIN;
      pinMode(pin, INPUT_PULLUP);
      printf("GPIO%d → INPUT_PULLUP\n", pin);
    }
    else if (m == "pulldown" || m == "input_pulldown") {
      if ((pinCapabilities[pin] & PC_DIN) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_DIN;
      pinMode(pin, INPUT_PULLDOWN);
      printf("GPIO%d → INPUT_PULLDOWN\n", pin);
    }
    else if (m == "analog") {
      if ((pinCapabilities[pin] & PC_ANALOG) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_ANALOG;
      analogRead(pin);
      printf("GPIO%d → ANALOG INPUT\n", pin);
    }
    else if (m == "touch") {
      if ((pinCapabilities[pin] & PC_TOUCH) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_TOUCH;
      touchRead(pin);
      printf("GPIO%d → TOUCH INPUT\n", pin);
    }
    else if (m == "ledpwm") {
      if ((pinCapabilities[pin] & PC_LEDPWM) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_LEDPWM;
      pinMode(pin, OUTPUT);
      printf("GPIO%d → LED PWM OUTPUT\n", pin);
    }
    else if (m == "dac") {
      if ((pinCapabilities[pin] & PC_DAC) == 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      pinAssignment[pin] = PC_DAC;
      pinMode(pin, OUTPUT);
      printf("GPIO%d → ANALOG OUTPUT\n", pin);
    }
    else { printf(RED "Unknown mode" RESET"\n"); return -1; }
    char buf[48]; snprintf(buf, sizeof(buf), "pinMode GPIO%d %s", pin, argv[2]); klog(buf);
    return 0;
  }
};
static struct cmdPinMode_args cmdPinModeHelp;
void cmdPinMode(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: pinmode <pin> <input|output|pullup|pulldown|analog|touch|ledpwm|dac>")); return; }

}

struct cmdDigitalWrite_args {
  arg_str_t *pin;
  arg_str_t *value;
  arg_end_t *end;
  static void setArgs(struct cmdDigitalWrite_args &args) {
    args.pin = arg_str1(NULL, NULL, "<pin>", "Pin to write to");
    args.value = arg_str1(NULL, NULL, "<HIGH|LOW|1|0|on|off>", "Value to write");
    args.end = arg_end(2);
  }
    static int implementation(int argc, char** argv, struct cmdDigitalWrite_args &args) {
    int pin = resolvePin(args.pin->sval[0], PC_DOUT);
    if (pin < 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
    String v = String(args.value->sval[0]); v.toLowerCase();
    int val = (v == "high" || v == "1" || v == "on") ? HIGH : LOW;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, val);
    printf("GPIO%d = %s\n", pin, val ? "HIGH" : "LOW");
    return 0;
  }
};
static struct cmdDigitalWrite_args cmdDigitalWriteHelp;

struct cmdDigitalRead_args {
  arg_str_t *pin;
  arg_end_t *end;
  static void setArgs(struct cmdDigitalRead_args &args) {
    args.pin = arg_str0(NULL, NULL, "<pin>", "Pin to read");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdDigitalRead_args &args) {
    if (!(args.pin->count)) {
      printf("\n  " YELLOW "GPIO State:" RESET "\n\n");
      printf("  Pin     State  │  Pin  State\n");
      printf("  ───────────────┼─────────────\n");
      int printcount = 0;
      for (int i = 0; i < SOC_GPIO_PIN_COUNT; i++) {
        // Only display pins configured for digital input or output
        if ((pinAssignment[i] & (PC_DIN | PC_DOUT)) == 0) { continue; }
        int v = digitalRead(i);
        printf("  GPIO%-2d  %s%-5s" RESET, i, v ? ((pinAssignment[i] == PC_DIN) ? GREEN : RED) : ((pinAssignment[i] == PC_DIN) ? GRAY : WHITE), v ? "HIGH" : "LOW");
        if (printcount % 2 == 0) printf("  │");
        else printf("\n");
        printcount++;
      }
      printf("\n\n");
    } else {
      int pin = resolvePin(args.pin->sval[0], PC_DIN | PC_DOUT);
      if (pin < 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      int v = digitalRead(pin);
      printf("GPIO%d = %s%s" RESET "\n", pin, v ? ((pinAssignment[pin] == PC_DIN) ? GREEN : RED) : ((pinAssignment[pin] == PC_DIN) ? GRAY : WHITE), v ? "HIGH" : "LOW");
    }
    return 0;
  }
};
static struct cmdDigitalRead_args cmdDigitalReadHelp;

struct cmdAnalogRead_args {
  arg_str_t *pin;
  arg_end_t *end;
  static void setArgs(struct cmdAnalogRead_args &args) {
    args.pin = arg_str0(NULL, NULL, "<pin>", "Pin to read");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdAnalogRead_args &args) {
    // ADC on ESP32: 12-bit (0-4095), 3.3V reference
    if (!(args.pin->count)) {
      printf("\n  " YELLOW "ADC Channels:" RESET "\n\n");
      for (int i = 0; i < BOARD_ADC_COUNT; i++) {
        // Skip any pins we shouldn't use.
        if ((pinAssignment[boardAdcPins[i]] & PC_ANALOG) == 0) { continue; }
        int raw = analogRead(boardAdcPins[i]);
        float v  = raw * 3.3f / 4095.0f;
        int bar  = map(raw, 0, 4095, 0, 30);
        printf("  A%-2d/GPIO%-2d [", i, boardAdcPins[i]);
        for (int b = 0; b < 30; b++) printf(b < bar ? GREEN "█" RESET : GRAY "░" RESET);
        printf("] %4d (%.2fV)\n", raw, v);
      }
      printf("\n");
    } else {
      int pin = resolvePin(args.pin->sval[0], PC_ANALOG);
      if (pin < 0) { printf(RED "Invalid pin" RESET"\n"); return -1; }
      int raw = analogRead(pin);
      float v  = raw * 3.3f / 4095.0f;
      printf("GPIO%d ADC = %d (%.3fV / 3.3V)\n", pin, raw, v);
    }
    return 0;
  }
};
static struct cmdAnalogRead_args cmdAnalogReadHelp;

void cmdPWM(char** argv, uint8_t argc) {
  // ESP32 LEDC: channel-based PWM
  if (argc < 3) {
    Serial.println(F("Usage: pwm <pin> <duty 0-255> [freq_hz] [channel 0-15]"));
    return;
  }
  int pin  = resolvePin(argv[1], PC_LEDPWM);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  int duty = constrain(safeAtoi(argv[2]), 0, 255);
  int freq = (argc >= 4) ? safeAtoi(argv[3]) : 5000;
  int ch   = (argc >= 5) ? constrain(safeAtoi(argv[4]), 0, 15) : 0;
  ledcAttachChannel(pin, freq, 8, ch);
  ledcWrite(pin, duty);
  float pct = duty * 100.0f / 255.0f;
  Serial.printf("GPIO%d PWM ch%d: duty=%d (%.0f%%) freq=%dHz\n", pin, ch, duty, pct, freq);
}

#ifdef SOC_DAC_SUPPORTED
void cmdDAC(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: dac <25|26> <0-255>")); return; }
  int pin = resolvePin(argv[1], PC_DAC);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  int val = constrain(safeAtoi(argv[2]), 0, 255);
  dacWrite(pin, val);
  float v = val * 3.3f / 255.0f;
  Serial.printf("DAC GPIO%d = %d (%.3fV)\n", pin, val, v);
}
#endif

void cmdTouch(char** argv, uint8_t argc) {
  if (argc < 2) {
    Serial.println(F("\n  " YELLOW "Touch Sensor Readings:" RESET "\n"));
    for (int i = 0; i < BOARD_TOUCH_COUNT; i++) {
      // Only read out pins configured for touch input
      if (pinAssignment[boardTouchPins[i]] != PC_TOUCH) { continue; }
      uint16_t val = touchRead(boardTouchPins[i]);
      int bar = map(constrain(val, 0, 80), 0, 80, 30, 0);
      bool touched = val < 30;
      Serial.printf("  T%-2d/GPIO%-2d [", i, boardTouchPins[i]);
      for (int b = 0; b < 30; b++)
        Serial.print(b < bar ? (touched ? RED "█" RESET : CYAN "█" RESET) : GRAY "░" RESET);
      Serial.printf("] %3d  %s\n", val, touched ? RED "<TOUCH>" RESET : "");
    }
    Serial.println();
    return;
  }
  int pin = resolvePin(argv[1], PC_TOUCH);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  uint16_t val = touchRead(pin);
  Serial.printf("Touch GPIO%d = %d (%s)\n", pin, val, val < 30 ? RED "TOUCHED" RESET : "not touched");
}

void cmdTone(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: tone <pin> <freq_hz> [duration_ms]")); return; }
  int pin  = resolvePin(argv[1], PC_LEDPWM);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  int freq = safeAtoi(argv[2]);
  if (freq < 1 || freq > 20000) { Serial.println(RED "Frequency: 1-20000 Hz" RESET); return; }
  ledcAttachChannel(pin, freq, 8, 14);
  ledcWrite(pin, 127); // 50% duty = square wave
  if (argc >= 4) {
    int ms = safeAtoi(argv[3]);
    Serial.printf("Tone GPIO%d: %dHz for %dms\n", pin, freq, ms);
    delay(ms);
    ledcWrite(pin, 0);
    ledcDetach(pin);
  } else {
    Serial.printf("Tone GPIO%d: %dHz continuous. Use 'notone %s' to stop.\n", pin, freq, argv[1]);
  }
}

void cmdNoTone(char** argv, uint8_t argc) {
  int pin = (argc >= 2) ? resolvePin(argv[1], PC_LEDPWM) : -1;
  ledcWrite(pin, 0);
  if (pin >= 0) ledcDetach(pin);
  Serial.println("Tone stopped");
}

void cmdGPIO(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: gpio <pin> <on|off|toggle|read>")); return; }
  int pin = resolvePin(argv[1], PC_DOUT);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  String act = String(argv[2]); act.toLowerCase();
  if (act == "on"  || act == "1" || act == "high") { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); Serial.printf("GPIO%d → ON\n", pin); }
  else if (act == "off" || act == "0" || act == "low") { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); Serial.printf("GPIO%d → OFF\n", pin); }
  else if (act == "toggle") {
    pinMode(pin, OUTPUT);
    int nv = !digitalRead(pin); digitalWrite(pin, nv);
    Serial.printf("GPIO%d toggled → %s\n", pin, nv ? "HIGH" : "LOW");
  }
  else if (act == "read") {
    Serial.printf("GPIO%d = %s\n", pin, digitalRead(pin) ? "HIGH" : "LOW");
  }
  else { Serial.println(RED "Unknown action. Use: on off toggle read" RESET); }
}

void cmdDisco(char** argv, uint8_t argc) {
  int cycles = (argc >= 2) ? constrain(safeAtoi(argv[1]), 1, 30)  : 3;
  int speed  = (argc >= 3) ? constrain(safeAtoi(argv[2]), 5, 500) : 40;
  const int nPins  = BOARD_DISCO_COUNT;
  for (int p : boardDiscoPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  Serial.printf(MAGENTA "  *** DISCO MODE *** " RESET "cycles=%d speed=%dms\n", cycles, speed);
  for (int c = 0; c < cycles; c++) {
    for (int i = 0; i < nPins; i++) { digitalWrite(boardDiscoPins[i], HIGH); delay(speed); digitalWrite(boardDiscoPins[i], LOW); }
    for (int i = nPins - 2; i > 0; i--) { digitalWrite(boardDiscoPins[i], HIGH); delay(speed); digitalWrite(boardDiscoPins[i], LOW); }
    for (int i = 0; i < nPins; i++) { digitalWrite(boardDiscoPins[i], (c + i) % 2); }
    delay(speed * 4);
    for (int p : boardDiscoPins) digitalWrite(p, LOW);
    Serial.printf("\r  Cycle %d/%d", c + 1, cycles);
  }
  for (int p : boardDiscoPins) pinMode(p, INPUT);
  Serial.println(F("\n  " GREEN "Disco complete!" RESET));
}

void cmdMorse(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: morse <pin> <MESSAGE>")); return; }
  int pin = resolvePin(argv[1], PC_DOUT);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
  const char* mt[] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--.."};
  Serial.printf("Morse GPIO%d: ", pin);
  for (int a = 2; a < argc; a++) {
    for (char* c = argv[a]; *c; c++) {
      char ch = toupper(*c);
      if (ch >= 'A' && ch <= 'Z') {
        Serial.print(ch);
        const char* code = mt[ch - 'A'];
        for (const char* m = code; *m; m++) {
          digitalWrite(pin, HIGH); Serial.print(*m == '.' ? '.' : '-');
          delay(*m == '.' ? 100 : 300);
          digitalWrite(pin, LOW); delay(100);
        }
        delay(300);
      } else if (ch == ' ') { Serial.print(' '); delay(700); }
    }
    Serial.print(' ');
  }
  Serial.println("Done");
}

void cmdSensor(char** argv, uint8_t argc) {
  Serial.println(F("\n  " YELLOW "Sensor Monitor (ADC — 3.3V ref, 12-bit)" RESET "\n"));
  for (int i = 0; i < BOARD_ADC_COUNT; i++) {
    // Only read out pins configured for analog input
    if (pinAssignment[boardAdcPins[i]] != PC_ANALOG) { continue; }
    long sum = 0;
    for (int j = 0; j < 8; j++) { sum += analogRead(boardAdcPins[i]); delay(1); }
    int avg = sum / 8;
    float v  = avg * 3.3f / 4095.0f;
    int bar  = map(avg, 0, 4095, 0, 32);
    Serial.printf("  A%-2d/GPIO%-2d [", i, boardAdcPins[i]);
    for (int b = 0; b < 32; b++) Serial.print(b < bar ? GREEN "█" RESET : GRAY "░" RESET);
    Serial.printf("] %4d (%.2fV)\n", avg, v);
  }
  Serial.println();
}

void cmdScope(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: scope <pin> [samples=80] [delay_ms=5]")); return; }
  int pin     = resolvePin(argv[1], PC_ANALOG);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  int samples = (argc >= 3) ? constrain(safeAtoi(argv[2]), 10, 200) : 80;
  int dly     = (argc >= 4) ? constrain(safeAtoi(argv[3]), 1, 500)  : 5;

  int* vals = (int*)malloc(samples * sizeof(int));
  if (!vals) { Serial.println(RED "malloc failed" RESET); return; }

  Serial.printf("\n  " CYAN "Scope GPIO%d" RESET " — %d samples @ %dms\n\n", pin, samples, dly);
  for (int i = 0; i < samples; i++) { vals[i] = analogRead(pin); delay(dly); }

  int vmin = 4095, vmax = 0;
  long vsum = 0;
  for (int i = 0; i < samples; i++) {
    if (vals[i] < vmin) vmin = vals[i];
    if (vals[i] > vmax) vmax = vals[i];
    vsum += vals[i];
  }

  int height = 12;
  int range  = vmax - vmin;
  if (range == 0) range = 1;
  for (int row = height - 1; row >= 0; row--) {
    Serial.print("  ");
    for (int i = 0; i < samples && i < 100; i++) {
      int mapped = map(vals[i], vmin, vmax, 0, height - 1);
      if      (mapped == row) Serial.print(CYAN "▪" RESET);
      else if (mapped >  row) Serial.print(GRAY "│" RESET);
      else                    Serial.print(" ");
    }
    Serial.println();
  }
  Serial.printf("  Min:%d  Max:%d  Avg:%ld  (%.2f–%.2fV)\n\n",
                vmin, vmax, vsum / samples,
                vmin * 3.3f / 4095.0f, vmax * 3.3f / 4095.0f);
  free(vals);
}

void cmdMonitor(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: monitor <pin> <interval_ms> [duration_s=10]")); return; }
  int pin      = resolvePin(argv[1], PC_ANALOG | PC_DIN);
  if (pin < 0) { Serial.println(RED "Invalid pin" RESET); return; }
  int interval = max(50, safeAtoi(argv[2]));
  int duration = (argc >= 4) ? constrain(safeAtoi(argv[3]), 1, 300) : 10;

  Serial.printf("\n  " YELLOW "Monitor GPIO%d" RESET " every %dms for %ds\n\n", pin, interval, duration);
  unsigned long end = millis() + duration * 1000UL;
  while (millis() < end) {
    unsigned long t = (millis() - bootTime) / 1000;
    if (pinAssignment[pin] == PC_ANALOG) {
      int raw = analogRead(pin);
      float v = raw * 3.3f / 4095.0f;
      Serial.printf("  [t+%lus] %d (%.3fV)\n", t, raw, v);
    } else {
      Serial.printf("  [t+%lus] %s\n", t, digitalRead(pin) ? GREEN "HIGH" RESET : GRAY "LOW" RESET);
    }
    delay(interval);
  }
  Serial.println(GREEN "  Monitor done" RESET "\n");
}

struct cmdPins_args {
  // Add all command arguments here (in implicit order)
  arg_end_t *end;
  static void setArgs(struct cmdPins_args &args) {
    // Set all arg_xxx_t* that were defined
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdPins_args &args) {
    printf("\n  " YELLOW "Pin configuration:\n" RESET);
    printf("  Pin    │ Capabilities │ Mode\n");
    printf("  ───────┼──────────────┼───────────────\n");
    for (int i = 0; i < SOC_GPIO_PIN_COUNT; i++) {
      // Skip disabled/unavailable pins
      if (pinAssignment[i] == 0) { continue; }
      const char* mode;
      switch(pinAssignment[i]) {
        case PC_DIN: mode = "Digital input"; break;
        case PC_DOUT: mode = "Digital output"; break;
        case PC_LEDPWM: mode = "LED PWM output"; break;
        case PC_TOUCH: mode = "Touch input"; break;
        case PC_ANALOG: mode = "Analog input"; break;
        default: mode = RED "ERROR" RESET; break;
      }
      printf("  GPIO%-2d │ %si%so%sl%st%sa" RESET "        │ %s\n",
        i,
        (pinCapabilities[i] & PC_DIN) ? GREEN : GRAY,
        (pinCapabilities[i] & PC_DOUT) ? GREEN : GRAY,
        (pinCapabilities[i] & PC_LEDPWM) ? GREEN : GRAY,
        (pinCapabilities[i] & PC_TOUCH) ? GREEN : GRAY,
        (pinCapabilities[i] & PC_ANALOG) ? GREEN : GRAY,
        mode);
    }
    return 0;
  }
};
static struct cmdPins_args cmdPinsHelp;

// Filesystem Commands
void cmdLS(char** argv, uint8_t argc) {
  String target = (argc >= 2) ? buildPath(argv[1]) : String(currentPath);
  if (!target.endsWith("/")) target += "/";

  Serial.println(F("\n  " YELLOW "Name                 Size    Modified" RESET));
  Serial.println(F("  " GRAY "───────────────────────────────────────────" RESET));

  int count = 0;
  File root = SPIFFS.open("/");
  File f    = root.openNextFile();
  while (f) {
    String fp = String(f.name()); // e.g. "/home/file.txt"
    // Show only direct children of target
    if (fp.startsWith(target)) {
      String rest = fp.substring(target.length());
      // Skip if rest contains another slash (grandchild)
      if (rest.length() > 0 && rest.indexOf('/') < 0) {
        bool isDir = rest.endsWith(".dir") || f.isDirectory();
        if (rest == ".dir") { f = root.openNextFile(); continue; } // skip dir markers
        Serial.print("  ");
        if (isDir) {
          Serial.print(BLUE); Serial.printf("%-22s" RESET, (rest + "/").c_str());
          Serial.println(GRAY "  <DIR>" RESET);
        } else {
          Serial.printf(WHITE "%-22s" RESET, rest.c_str());
          Serial.printf("%6d bytes\n", (int)f.size());
        }
        count++;
      }
    }
    f = root.openNextFile();
  }

  if (count == 0) Serial.println(GRAY "  (empty)" RESET);
  // Show total SPIFFS usage
  size_t total = SPIFFS.totalBytes();
  size_t used  = SPIFFS.usedBytes();
  Serial.printf("\n  " GRAY "%d items  │  SPIFFS: %u / %u KB used" RESET "\n\n",
                count, used / 1024, total / 1024);
}

void cmdCD(char** argv, uint8_t argc) {
  if (argc < 2 || strcmp(argv[1], "/") == 0) {
    strncpy(currentPath, "/", PATH_LEN - 1); return;
  }
  if (strcmp(argv[1], "..") == 0) {
    if (strcmp(currentPath, "/") == 0) return;
    String p = String(currentPath);
    if (p.endsWith("/")) p = p.substring(0, p.length() - 1);
    int last = p.lastIndexOf('/');
    strncpy(currentPath, (last <= 0 ? "/" : p.substring(0, last + 1)).c_str(), PATH_LEN - 1);
    return;
  }
  // Absolute or relative
  String target = buildPath(argv[1]);
  if (!target.endsWith("/")) target += "/";
  if (isDirectory(target.substring(0, target.length() - 1)) || target == "/") {
    strncpy(currentPath, target.c_str(), PATH_LEN - 1);
  } else {
    Serial.printf(RED "cd: '%s' not found\n" RESET, argv[1]);
  }
}

void cmdMkdir(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: mkdir <name>")); return; }
  String path = buildPath(argv[1]);
  if (isDirectory(path)) { Serial.println(YELLOW "Already exists" RESET); return; }
  ensureDir(path);
  Serial.printf(GREEN "Directory '%s' created\n" RESET, argv[1]);
  klog(("mkdir " + path).c_str());
}

void cmdTouch2(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: touch <filename>")); return; }
  String path = buildPath(argv[1]);
  if (!SPIFFS.exists(path)) {
    File f = SPIFFS.open(path, FILE_WRITE);
    if (!f) { Serial.println(RED "Failed to create file" RESET); return; }
    f.close();
  }
  Serial.printf("'%s' OK\n", argv[1]);
}

void cmdCat(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: cat <filename>")); return; }
  String path = buildPath(argv[1]);
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) { Serial.printf(RED "File not found: %s\n" RESET, argv[1]); return; }
  if (f.size() == 0) { Serial.println(GRAY "(empty)" RESET); f.close(); return; }
  while (f.available()) {
    char c = f.read();
    Serial.write(c);
  }
  Serial.println();
  f.close();
}

void cmdWrite(char** argv, uint8_t argc) {
  // writefile <name> <content...>  or  append <name> <content>
  bool appendMode = (argc >= 1 && strcasecmp(argv[0], "append") == 0);
  if (argc < 3) {
    Serial.println(F("Usage: writefile <filename> <content>"));
    Serial.println(F("       append    <filename> <content>"));
    return;
  }
  String path = buildPath(argv[1]);
  File f = SPIFFS.open(path, appendMode ? FILE_APPEND : FILE_WRITE);
  if (!f) { Serial.println(RED "Cannot open file" RESET); return; }
  for (uint8_t i = 2; i < argc; i++) {
    f.print(argv[i]);
    if (i < argc - 1) f.print(' ');
  }
  f.println();
  f.close();
  Serial.printf("Written to '%s'\n", argv[1]);
}

void cmdRM(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: rm <name> [-r]")); return; }
  String path = buildPath(argv[1]);
  bool recursive = (argc >= 3 && strcmp(argv[2], "-r") == 0);

  if (isDirectory(path)) {
    if (!recursive) { Serial.println(YELLOW "Use 'rm <dir> -r' to remove directory" RESET); return; }
    // Remove all files under this path
    String prefix = path; if (!prefix.endsWith("/")) prefix += "/";
    File root = SPIFFS.open("/");
    File fi = root.openNextFile();
    while (fi) {
      if (String(fi.name()).startsWith(prefix)) SPIFFS.remove(fi.name());
      fi = root.openNextFile();
    }
    String marker = path + "/.dir";
    SPIFFS.remove(marker);
    Serial.printf(GREEN "Removed directory '%s'\n" RESET, argv[1]);
  } else if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
    Serial.printf(GREEN "Removed '%s'\n" RESET, argv[1]);
  } else {
    Serial.printf(RED "Not found: %s\n" RESET, argv[1]);
  }
}

void cmdMv(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: mv <src> <dst>")); return; }
  String src = buildPath(argv[1]);
  String dst = buildPath(argv[2]);
  if (!SPIFFS.exists(src)) { Serial.println(RED "Source not found" RESET); return; }
  // Copy then delete
  File fs_ = SPIFFS.open(src, FILE_READ);
  File fd  = SPIFFS.open(dst, FILE_WRITE);
  if (!fs_ || !fd) { Serial.println(RED "Move failed" RESET); return; }
  while (fs_.available()) fd.write(fs_.read());
  fs_.close(); fd.close();
  SPIFFS.remove(src);
  Serial.printf("Moved '%s' → '%s'\n", argv[1], argv[2]);
}

void cmdCp(char** argv, uint8_t argc) {
  if (argc < 3) { Serial.println(F("Usage: cp <src> <dst>")); return; }
  String src = buildPath(argv[1]);
  String dst = buildPath(argv[2]);
  if (!SPIFFS.exists(src)) { Serial.println(RED "Source not found" RESET); return; }
  File fs_ = SPIFFS.open(src, FILE_READ);
  File fd  = SPIFFS.open(dst, FILE_WRITE);
  if (!fs_ || !fd) { Serial.println(RED "Copy failed" RESET); return; }
  size_t bytes = 0;
  while (fs_.available()) { fd.write(fs_.read()); bytes++; }
  fs_.close(); fd.close();
  Serial.printf("Copied %u bytes → '%s'\n", bytes, argv[2]);
}

void cmdDf(char** argv, uint8_t argc) {
  size_t total = SPIFFS.totalBytes();
  size_t used  = SPIFFS.usedBytes();
  size_t free_ = total - used;
  int pct = (used * 100) / total;
  int bar = (used * 30) / total;
  Serial.println(F("\n  " YELLOW "Filesystem (SPIFFS):" RESET));
  Serial.print  (F("  ["));
  for (int i = 0; i < 30; i++) Serial.print(i < bar ? GREEN "█" RESET : GRAY "░" RESET);
  Serial.printf ("] %d%%\n", pct);
  Serial.printf ("  Total: %u KB  Used: %u KB  Free: %u KB\n\n",
                 total/1024, used/1024, free_/1024);
}

// WiFi Commands
struct cmdWifi_args {
  arg_end_t *end;
  static void setArgs(struct cmdWifi_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifi_args &args) {
    // Show status
    printf("\n  " CYAN "WiFi Status:" RESET);
    printf("  Mode : %s\n",
      WiFi.getMode() == WIFI_STA ? "Station" :
      WiFi.getMode() == WIFI_AP  ? "Access Point" :
      WiFi.getMode() == WIFI_AP_STA ? "AP+Station" : "Off");
    if (wifiConnected) {
      printf("  SSID : %s\n", WiFi.SSID().c_str());
      printf("  IP   : %s\n", WiFi.localIP().toString().c_str());
      printf("  RSSI : %d dBm\n", WiFi.RSSI());
      printf("  MAC  : %s\n", WiFi.macAddress().c_str());
      printf("  GW   : %s\n", WiFi.gatewayIP().toString().c_str());
      printf("  DNS  : %s\n", WiFi.dnsIP().toString().c_str());
    }
    if (apActive) {
      printf("  AP   : %s  IP: %s  Clients: %d\n",
                    apSSID.c_str(), WiFi.softAPIP().toString().c_str(),
                    WiFi.softAPgetStationNum());
    }
    printf("\n");
    return 0;
  }
};
static struct cmdWifi_args cmdWifiHelp;

struct cmdWifiConnect_args {
  arg_str_t *ssid;
  arg_str_t *pass;
  arg_end_t *end;
  static void setArgs(struct cmdWifiConnect_args &args) {
    args.ssid = arg_str1(NULL, NULL, "<ssid>", "The wireless network to connect to");
    args.pass = arg_str1(NULL, NULL, "<pass>", "The password for the network");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiConnect_args &args) {
    Serial.printf("  Connecting to '%s'", args.ssid->sval[0]);
    WiFi.mode(WIFI_STA);
    WiFi.begin(args.ssid->sval[0], args.pass->sval[0]);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
      delay(500); Serial.print('.'); retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      staSSID = String(argv[2]);
      printf("\n  " GREEN "Connected! IP: %s" RESET "\n\n", WiFi.localIP().toString().c_str());
      klog(("WiFi connected: " + staSSID).c_str());
    } else {
      printf("\n  " RED "Connection failed" RESET"\n");
      WiFi.disconnect();
    }
    return 0;
  }
};
static struct cmdWifiConnect_args cmdWifiConnectHelp;

struct cmdWifiDisconnect_args {
  arg_end_t *end;
  static void setArgs(struct cmdWifiDisconnect_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiDisconnect_args &args) {
    WiFi.disconnect(true);
    wifiConnected = false;
    printf("  WiFi disconnected\n");
    return 0;
  }
};
static struct cmdWifiDisconnect_args cmdWifiDisconnectHelp;

struct cmdWifiAp_args {
  arg_str_t *ssid;
  arg_str_t *pass;
  arg_end_t *end;
  static void setArgs(struct cmdWifiAp_args &args) {
    args.ssid = arg_str1(NULL, NULL, "<SSID>", "The name of the network to create");
    args.pass = arg_str0(NULL, NULL, "password", "The password for the network");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiAp_args &args) {
    apSSID = String(args.ssid->sval[0]);
    apPASS = (args.pass->count) ? String(args.pass->sval[0]) : "";
    WiFi.mode(wifiConnected ? WIFI_AP_STA : WIFI_AP);
    bool ok = apPASS.length() > 0
                ? WiFi.softAP(apSSID.c_str(), apPASS.c_str())
                : WiFi.softAP(apSSID.c_str());
    apActive = ok;
    if (ok) printf(GREEN "  AP '%s' started  IP: %s\n" RESET,
                           apSSID.c_str(), WiFi.softAPIP().toString().c_str());
    else     printf(RED "  AP failed" RESET"\n");
    return 0;
  }
};
static struct cmdWifiAp_args cmdWifiApHelp;

struct cmdWifiScan_args {
  arg_end_t *end;
  static void setArgs(struct cmdWifiScan_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiScan_args &args) {
    printf("  Scanning...\n");
    int n = WiFi.scanNetworks();
    if (n == 0) { Serial.printf("  No networks found\n"); return 0; }
    printf("\n  " YELLOW "#  SSID                           RSSI  ENC" RESET"\n");
    printf("  " GRAY "───────────────────────────────────────────────" RESET"\n");
    for (int i = 0; i < n; i++) {
      const char* enc = WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? GREEN "Open" RESET : CYAN "WPA" RESET;
      printf("  %-2d %-32s %4d  %s\n", i + 1,
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i), enc);
    }
    WiFi.scanDelete();
    printf("\n");
    return 0;
  }
};
static struct cmdWifiScan_args cmdWifiScanHelp;

struct cmdWifiPing_args {
  arg_rex_t *ip;
  arg_end_t *end;
  static void setArgs(struct cmdWifiPing_args &args) {
    args.ip = arg_rex1(NULL, NULL, "[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+", "<IP>", 0, "Address to ping");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiPing_args &args) {
    if (!wifiConnected) { printf(YELLOW "Not connected to WiFi" RESET"\n"); return -1; }
    IPAddress ip;
    if (!ip.fromString(args.ip->sval[0])) { printf(RED "Invalid IP. DNS not supported in minimal mode." RESET"\n"); return -1; }
    printf("  Pinging %s:\n", args.ip->sval[0]);
    for (int i = 0; i < 4; i++) {
      // ESP32 Arduino core does not include ping by default; send TCP probe instead
      WiFiClient client;
      unsigned long t = millis();
      bool ok = client.connect(ip, 80);
      unsigned long rtt = millis() - t;
      client.stop();
      if (ok) printf("  seq=%d time=%lums " GREEN "reachable" RESET "\n", i, rtt);
      else    printf("  seq=%d " RED "no response" RESET "\n", i);
      delay(500);
    }
    return 0;
  }
};
static struct cmdWifiPing_args cmdWifiPingHelp;

struct cmdWifiHostname_args {
  arg_str_t *name;
  arg_end_t *end;
  static void setArgs(struct cmdWifiHostname_args &args) {
    args.name = arg_str0(NULL, NULL, "hostname", "Set the system hostname");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiHostname_args &args) {
    if (args.name->count) {
      WiFi.setHostname(args.name->sval[0]);
      printf("  Hostname set to '%s'\n", args.name->sval[0]);
    } else {
      printf("  Hostname: %s\n", WiFi.getHostname());
    }
    return 0;
  }
};
static struct cmdWifiHostname_args cmdWifiHostnameHelp;

struct cmdWifiHttpStart_args {
  arg_int_t *port;
  arg_end_t *end;
  static void setArgs(struct cmdWifiHttpStart_args &args) {
    args.port = arg_int0(NULL, NULL, "port", "The port to listen on");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiHttpStart_args &args) {
    if (!wifiConnected && !apActive) { printf(YELLOW "Connect WiFi first" RESET"\n"); return -1; }
    int port = (args.port->count) ? args.port->ival[0] : 80;
    if (httpServer) { delete httpServer; httpServer = nullptr; }
    httpServer = new WebServer(port);
    httpServer->on("/", []() {
      String html = F("<!DOCTYPE html><html><head><title>KernelESP</title>"
        "<style>body{font-family:monospace;background:#0d0d0d;color:#0f0;padding:2em}"
        "h1{color:#0ff}pre{color:#fff;border:1px solid #0f0;padding:1em;}</style></head>"
        "<body><h1>KernelESP v1.0</h1><pre>Status: Online\nHost: kernelesp\n"
        "Uptime: ");
      html += (millis() - bootTime) / 1000;
      html += F("s\nFree RAM: ");
      html += ESP.getFreeHeap() / 1024;
      html += F(" KB\n</pre>"
        "<p>Control via Serial terminal.</p></body></html>");
      if (httpServer) httpServer->send(200, "text/html", html);
    });
    httpServer->begin();
    httpRunning = true;
    printf(GREEN "  HTTP server started on port %d\n  URL: http://%s/\n" RESET,
                  port, wifiConnected ? WiFi.localIP().toString().c_str()
                                      : WiFi.softAPIP().toString().c_str());
    return 0;
  }
};
static struct cmdWifiHttpStart_args cmdWifiHttpStartHelp;

struct cmdWifiHttpStop_args {
  arg_end_t *end;
  static void setArgs(struct cmdWifiHttpStop_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiHttpStop_args &args) {
    if (httpServer) { httpServer->stop(); delete httpServer; httpServer = nullptr; httpRunning = false; }
    printf("  HTTP server stopped\n");
    return 0;
  }
};
static struct cmdWifiHttpStop_args cmdWifiHttpStopHelp;

struct cmdWifiMac_args {
  arg_end_t *end;
  static void setArgs(struct cmdWifiMac_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWifiMac_args &args) {
    printf("  STA MAC: %s\n", WiFi.macAddress().c_str());
    printf("  AP  MAC: %s\n", WiFi.softAPmacAddress().c_str());
    return 0;
  }
};
static struct cmdWifiMac_args cmdWifiMacHelp;


// Scripting

void runScript(const char* text) {
  char* buf = (char*)malloc(strlen(text) + 2);
  if (!buf) { Serial.println(RED "malloc error" RESET); return; }
  strcpy(buf, text);

  char* cmd = strtok(buf, ";");
  int n = 0;
  while (cmd) {
    cmd = ltrim(cmd);
    // strip trailing whitespace
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\r' || cmd[len-1] == '\n')) cmd[--len] = '\0';
    if (len > 0 && cmd[0] != '#') {
      Serial.printf(GRAY "  [%d]" RESET " $ %s\n", ++n, cmd);
      Console.run(cmd);
      delay(20);
    }
    cmd = strtok(nullptr, ";");
  }
  free(buf);
}

struct cmdEval_args {
  struct arg_str *cmd;
  struct arg_end *end;
  static void setArgs(struct cmdEval_args &args) {
      args.cmd = arg_strn(NULL, NULL, "<cmd>", 1, MAX_ARGS, "The command to execute");
      args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdEval_args &args) {
    String code = "";

    for (int i = 1; i < argc; i++) { if (i > 1) code += " "; code += argv[i]; }
    Serial.println(F(CYAN ">>> eval" RESET));
    runScript(code.c_str());
    Serial.println(F(CYAN ">>> done" RESET));

    return 0;
  }
};
static struct cmdEval_args cmdEvalHelp;

void cmdRun(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: run <script_file>")); return; }
  String path = buildPath(argv[1]);
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) { Serial.printf(RED "Script not found: %s\n" RESET, argv[1]); return; }
  String content = f.readString();
  f.close();
  Serial.printf(CYAN ">>> run %s" RESET "\n", argv[1]);
  runScript(content.c_str());
  Serial.printf(CYAN ">>> done (%u bytes)" RESET "\n", content.length());
  klog(("run " + path).c_str());
}

struct cmdFor_args {
  arg_int_t *count;
  arg_str_t *cmd;
  arg_end_t *end;
  static void setArgs(struct cmdFor_args &args) {
    args.count = arg_int1(NULL, NULL, "<count>", "The number of times to execute the command");
    args.cmd = arg_strn(NULL, NULL, "<cmd>", 1, MAX_ARGS, "The command to execute");
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdFor_args &args) {
    int count;
    String cmd = "";

    count = args.count->ival[0];

    for (int i = 2; i < argc; i++) { if (i > 2) cmd += " "; cmd += argv[i]; }
    for (int i = 0; i < count; i++) {
      Serial.printf(GRAY "\r  [%d/%d]" RESET, i + 1, count);
      Console.run(cmd);
      delay(5);
    }
    Serial.println();

    return 0;
  }
};
static struct cmdFor_args cmdForHelp;

void cmdDelay(char** argv, uint8_t argc) {
  if (argc < 2) { Serial.println(F("Usage: delay <ms>")); return; }
  int ms = constrain(safeAtoi(argv[1]), 0, 60000);
  delay(ms);
}

// System Commands
void cmdFree(char** argv, uint8_t argc) {
  uint32_t heap    = ESP.getFreeHeap();
  uint32_t minHeap = ESP.getMinFreeHeap();
  uint32_t maxAlloc= ESP.getMaxAllocHeap();
  Serial.println(F("\n  " YELLOW "Memory:" RESET));
  Serial.printf("  Free heap      : %u bytes (%u KB)\n", heap, heap / 1024);
  Serial.printf("  Min free heap  : %u bytes\n", minHeap);
  Serial.printf("  Max alloc block: %u bytes\n", maxAlloc);
  Serial.printf("  PSRAM          : %u bytes\n", ESP.getFreePsram());
  cmdDf(argv, 0);
}

void cmdSysInfo(char** argv, uint8_t argc) {
  unsigned long up  = (millis() - bootTime) / 1000;
  uint8_t h = up / 3600, m = (up % 3600) / 60, s = up % 60;

  Serial.println(F("\n"
    CYAN  "  ██████╗ ███████╗██████╗ ██████╗ ██████╗ \n"
    BCYAN "  ██╔════╝██╔════╝██╔══██╗╚════██╗╚════██╗\n"
    CYAN  "  █████╗  ███████╗██████╔╝ █████╔╝ █████╔╝\n"
    CYAN  "  ██╔══╝  ╚════██║██╔═══╝  ╚═══██╗ ██╔═══╝\n"
    BCYAN "  ███████╗███████║██║     ███████╗███████║ \n"
    GRAY  "  ╚══════╝╚══════╝╚═╝     ╚══════╝╚══════╝" RESET "\n"
  ));

  Serial.printf("  " YELLOW "OS     " RESET ": KernelESP v1.0\n");
  Serial.printf("  " YELLOW "Host   " RESET ": %s\n", HOSTNAME);
  Serial.printf("  " YELLOW "Board  " RESET ": "ARDUINO_BOARD" @ %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("  " YELLOW "Chip   " RESET ": %s  Rev%d  Cores:%d\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores());
  Serial.printf("  " YELLOW "Flash  " RESET ": %u KB  (mode:%d  speed:%d MHz)\n",
                ESP.getFlashChipSize()/1024, ESP.getFlashChipMode(), ESP.getFlashChipSpeed()/1000000);
  Serial.printf("  " YELLOW "RAM    " RESET ": %u KB free / PSRAM: %u KB\n",
                ESP.getFreeHeap()/1024, ESP.getFreePsram()/1024);
  Serial.printf("  " YELLOW "SPIFFS " RESET ": %u / %u KB\n",
                SPIFFS.usedBytes()/1024, SPIFFS.totalBytes()/1024);
  Serial.printf("  " YELLOW "Uptime " RESET ": %dh %dm %ds\n", h, m, s);
  Serial.printf("  " YELLOW "WiFi   " RESET ": %s\n",
                wifiConnected ? (GREEN + WiFi.SSID() + "  " + WiFi.localIP().toString() + RESET).c_str()
                              : RED "Offline" RESET);
  Serial.println();
}

void cmdDmesg(char** argv, uint8_t argc) {
  Serial.println(F("\n  " YELLOW "Kernel Log:" RESET "\n"));
  for (uint8_t i = 0; i < dmesgCount; i++) {
    uint8_t idx = (dmesgHead - dmesgCount + i + DMESG_LINES) % DMESG_LINES;
    Serial.printf("  " GRAY "[%4lus]" RESET " %s\n", dmesgBuf[idx].ts, dmesgBuf[idx].msg);
  }
  Serial.println();
}

struct cmdReboot_args {
  arg_end_t *end;
  static void setArgs(struct cmdReboot_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdReboot_args &args) {

    printf("\n  Rebooting...\n");
    delay(300);
    ESP.restart();

    return 0;
  }
};
static struct cmdReboot_args cmdRebootHelp;

void cmdWhoami(char** argv, uint8_t argc) { Serial.println("root"); }
void cmdUname(char** argv, uint8_t argc)  { Serial.println("KernelESP v1.0 ESP32 xtensa"); }
void cmdUptime(char** argv, uint8_t argc) {
  unsigned long t = (millis() - bootTime) / 1000;
  Serial.printf("up %dh %dm %ds\n", (int)(t/3600), (int)((t%3600)/60), (int)(t%60));
}
void cmdPwd(char** argv, uint8_t argc) { Serial.println(currentPath); }
void cmdEcho(char** argv, uint8_t argc) {
  for (int i = 1; i < argc; i++) { if (i > 1) Serial.print(' '); Serial.print(argv[i]); }
  Serial.println();
}
void cmdClear(char** argv, uint8_t argc) { showLogo(); }

struct cmdWave_args {
  arg_end_t *end;
  static void setArgs(struct cmdWave_args &args) {
    args.end = arg_end(2);
  }
  static int implementation(int argc, char** argv, struct cmdWave_args &args) {
    printf("\n"
      CYAN "  ╭╮              ╭╮\n"
      CYAN "  ╭╯╰╮          ╭╯╰╮\n"
      CYAN " ╭╯  ╰╮        ╭╯  ╰╮\n"
      CYAN "╭╯    ╰╮──────╭╯    ╰╮\n"
      CYAN "╯      ╰╮    ╭╯      ╰\n" RESET
    );
    return 0;
  }
};
static struct cmdWave_args cmdWaveHelp;

void cmdHelp(char** argv, uint8_t argc) {
  Serial.println(F("\n  " CYAN "KernelESP v1.0 Command Reference" RESET "\n"));

  Serial.println(F("  " GREEN "Hardware:" RESET));
  Serial.println(F("    pins                          Show current pin configuration"));
  Serial.println(F("    pinmode <pin> <mode>          Set pin mode"));
  Serial.println(F("                                  (input/output/pullup/pulldown/"));
  Serial.println(F("                                  analog/touch/ledpwm/dac)"));
  Serial.println(F("    write   <pin> <HIGH|LOW>      Digital write"));
  Serial.println(F("    read    [pin]                 Digital read (all if no pin)"));
  Serial.println(F("    aread   [pin]                 ADC read (0-4095, 3.3V)"));
  Serial.println(F("    pwm     <pin> <0-255> [freq]  LEDC PWM output"));
  #ifdef SOC_DAC_SUPPORTED
  Serial.println(F("    dac     <25|26> <0-255>       DAC voltage output"));
  #endif
  Serial.println(F("    gpio    <pin> <on|off|toggle> Quick GPIO"));
  Serial.println(F("    tone    <pin> <hz> [ms]       Square wave tone"));
  Serial.println(F("    notone  [pin]                 Stop tone"));
  Serial.println(F("    tsense  [pin]                 Capacitive touch read"));
  Serial.println(F("    disco   [cycles] [speed]      LED show"));
  Serial.println(F("    morse   <pin> <MSG>            Morse code"));

  Serial.println(F("\n  " GREEN "Sensors:" RESET));
  Serial.println(F("    sensor                        All ADC channels with bar"));
  Serial.println(F("    scope   <pin> [n] [ms]        Oscilloscope plot"));
  Serial.println(F("    monitor <pin> <ms> [s]        Live pin monitor"));

  Serial.println(F("\n  " GREEN "Filesystem (SPIFFS — persistent):" RESET));
  Serial.println(F("    ls [dir]   cd <dir>   pwd   mkdir <name>   touch <name>"));
  Serial.println(F("    cat <f>    writefile <f> <text>   append <f> <text>"));
  Serial.println(F("    rm <name> [-r]   mv <src> <dst>   cp <src> <dst>   df"));

  Serial.println(F("\n  " GREEN "WiFi:" RESET));
  Serial.println(F("    wifi                          Status"));
  Serial.println(F("    wifi scan                     Scan networks"));
  Serial.println(F("    wifi connect <SSID> <PASS>    Connect"));
  Serial.println(F("    wifi ap <SSID> [PASS]         Soft Access Point"));
  Serial.println(F("    wifi ping <IP>                Connectivity check"));
  Serial.println(F("    wifi http start [port]        Web server"));
  Serial.println(F("    wifi mac / hostname           Info / rename"));

  Serial.println(F("\n  " GREEN "Scripting:" RESET));
  Serial.println(F("    eval \"cmd1; cmd2\"             Execute inline script"));
  Serial.println(F("    run  <script.sh>              Execute file script"));
  Serial.println(F("    for  <n> \"cmd\"                Loop n times"));
  Serial.println(F("    delay <ms>                    Wait"));

  Serial.println(F("\n  " GREEN "System:" RESET));
  Serial.println(F("    sysinfo / neofetch   uptime   free   df   dmesg"));
  Serial.println(F("    whoami   uname   echo <text>   clear   wave   reboot\n"));
}

// Setup
void setup() {
  Serial.begin(115200);
  bootTime = millis();

  // Brief boot flash on GPIO2 (built-in LED most boards)
  #ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 6; i++) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(60); }
  digitalWrite(LED_BUILTIN, LOW);
  #endif

  initPins();

  initFilesystem();

  showLogo();

  klog("KernelESP v1.0 booted");
  klog("SPIFFS OK");
  klog("Serial @ 115200");

  // Todo: dynamic hostname, path, wifi
  Console.setPrompt(BGREEN "root@" HOSTNAME RESET ":" BLUE "" RESET "" WHITE "$ " RESET);
  Console.begin();
  Console.setMaxHistory(16);
  // Hardware
  Command<struct cmdPinMode_args>::addCmd({"pinmode"}, "Set pin mode", cmdPinModeHelp);
  Command<struct cmdDigitalWrite_args>::addCmd({"write", "digitalwrite"}, "Digital write", cmdDigitalWriteHelp);
  Command<struct cmdDigitalRead_args>::addCmd({"read", "digitalread"}, "Digital read (all if no pin)", cmdDigitalReadHelp);
  Command<struct cmdAnalogRead_args>::addCmd({"aread", "analogread"}, "ADC read (0-4095, 3.3V)", cmdAnalogReadHelp);
  //Console.addCmd("pwm", "", cmdPWM);
  #ifdef SOC_DAC_SUPPORTED
  //Console.addCmd("dac", "", cmdDAC);
  #endif
  //Console.addCmd("gpio", "", cmdGPIO);
  //Console.addCmd("tone", "", cmdTone);
  //Console.addCmd("notone", "", cmdNoTone);
  //Console.addCmd("tsense", "", cmdTouch);
  //Console.addCmd("disco", "", cmdDisco);
  //Console.addCmd("morse", "", cmdMorse);
  //Console.addCmd("sensor", "", cmdSensor);
  //Console.addCmd("scope", "", cmdScope);
  //Console.addCmd("monitor", "", cmdMonitor);
  Command<struct cmdPins_args>::addCmd({"pins"}, "Show current pin configuration", cmdPinsHelp);

  // Filesystem
  //Console.addCmd("ls", "", cmdLS);
  //Console.addCmd("dir", "", cmdLS);
  //Console.addCmd("cd", "", cmdCD);
  //Console.addCmd("pwd", "", cmdPwd);
  //Console.addCmd("mkdir", "", cmdMkdir);
  //Console.addCmd("touch", "", cmdTouch2);
  //Console.addCmd("cat", "", cmdCat);
  //Console.addCmd("type", "", cmdCat);
  //Console.addCmd("writefile", "", cmdWrite);
  //Console.addCmd("write>", "", cmdWrite);
  //Console.addCmd("append", "", cmdWrite);
  //Console.addCmd("rm", "", cmdRM);
  //Console.addCmd("del", "", cmdRM);
  //Console.addCmd("mv", "", cmdMv);
  //Console.addCmd("cp", "", cmdCp);
  //Console.addCmd("df", "", cmdDf);
  //Console.addCmd("echo", "", cmdEcho);

  // WiFi
  //Console.addCmd("wifi", "", cmdWifi);
  Command<struct cmdWifi_args>::addCmd({"wifi"}, "Show wifi status", cmdWifiHelp);
  Command<struct cmdWifiConnect_args>::addCmd({"wifi-connect", "wifi-up"}, "Connect to AP", cmdWifiConnectHelp);
  Command<struct cmdWifiDisconnect_args>::addCmd({"wifi-disconnect", "wifi-down"}, "Disconnect from AP", cmdWifiDisconnectHelp);
  Command<struct cmdWifiAp_args>::addCmd({"wifi-ap"}, "Create an AP", cmdWifiApHelp);
  Command<struct cmdWifiScan_args>::addCmd({"wifi-scan"}, "Scan for networks", cmdWifiScanHelp);
  Command<struct cmdWifiPing_args>::addCmd({"wifi-ping"}, "TCP connectivity check", cmdWifiPingHelp);
  Command<struct cmdWifiHostname_args>::addCmd({"wifi-hostname"}, "Get/set hostname", cmdWifiHostnameHelp);
  Command<struct cmdWifiHttpStart_args>::addCmd({"wifi-http-start"}, "Start HTTP server", cmdWifiHttpStartHelp);
  Command<struct cmdWifiHttpStop_args>::addCmd({"wifi-http-stop"}, "Stop HTTP server", cmdWifiHttpStopHelp);
  Command<struct cmdWifiMac_args>::addCmd({"wifi-mac"}, "Show MAC addresses", cmdWifiMacHelp);

  // Scripting
  Command<struct cmdEval_args>::addCmd({"eval", "exec"}, "Run commands as though read from a script", cmdEvalHelp);
//  Console.addCmd("run", "", cmdRun);
//  Console.addCmd("sh", "", cmdRun);
  Command<struct cmdFor_args>::addCmd({"for", "loop"}, "Run a command multiple times", cmdForHelp);
//  Console.addCmd("delay", "", cmdDelay);
//  Console.addCmd("sleep", "", cmdDelay);

  // System
//  Console.addCmd("help", "", cmdHelp);
//  Console.addCmd("?", "", cmdHelp);
//  Console.addCmd("sysinfo", "", cmdSysInfo);
//  Console.addCmd("neofetch", "", cmdSysInfo);
//  Console.addCmd("dmesg", "", cmdDmesg);
//  Console.addCmd("log", "", cmdDmesg);
//  Console.addCmd("free", "", cmdFree);
//  Console.addCmd("mem", "", cmdFree);
//  Console.addCmd("uptime", "", cmdUptime);
//  Console.addCmd("whoami", "", cmdWhoami);
//  Console.addCmd("uname", "", cmdUname);
//  Console.addCmd("clear", "", cmdClear);
//  Console.addCmd("cls", "", cmdClear);
  Command<struct cmdReboot_args>::addCmd({"reboot", "reset"}, "Reboot the processor", cmdRebootHelp);
  Command<struct cmdWave_args>::addCmd({"wave"}, "Display a wave in ASCII art", cmdWaveHelp);
  Console.addHelpCmd();

  Console.attachToSerial(true);

}

// Loop
void loop() {
  // Handle HTTP server if active
  if (httpRunning && httpServer) httpServer->handleClient();

  // Small yield so WiFi stack can breathe
  delay(1);
}
