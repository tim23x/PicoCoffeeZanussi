// Controller for a Zanussi ZCOF637X espresso machine.
//
// Hardware: MAX6675 thermocouple (SPI0), SH1106 128x64 OLED (I2C0), active-high
// relays for heater / pump / group solenoid / grinder plus two spares, four active-high buttons,
// and a WS2812 chain (PIO): a lamp ring, then the status LEDs. Either the OLED or the
// status LEDs can be left out. With wifi_config.h present, a web page on the Pico 2 W's Wi-Fi
// shows the status and a temperature graph, and changes the settings (kept in flash). With
// MQTT_HOST set as well, the machine appears in Home Assistant through MQTT discovery.
// Logging goes to stdio: UART0 (GP0 TX, GP1 RX, 115200) for the Debug Probe, and USB.

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/flash.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include "lwip/apps/mdns.h"
#include "lwip/apps/mqtt.h"
#include "ws2812.pio.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"  // WIFI_SSID and WIFI_PASSWORD, see wifi_config.example.h
#include "web_page.h"
#define WEB_ENABLED 1
#else
#define WEB_ENABLED 0
#endif
#if WEB_ENABLED && defined(MQTT_HOST)
#define MQTT_ENABLED 1  // Home Assistant over MQTT
#else
#define MQTT_ENABLED 0
#endif

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

#define SPI_PORT spi0
#define I2C_PORT i2c0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  22
#define I2C_SDA 8
#define I2C_SCL 9

#define OLED_ADDRESS 0x3C

constexpr uint HEATER_PIN = 2;
constexpr uint GRINDER_PIN = 3;
constexpr uint SOLENOID_PIN = 4;
constexpr uint PUMP_PIN = 5;
constexpr uint SPARE1_PIN = 6;  // spare relay outputs, held off
constexpr uint SPARE2_PIN = 7;
constexpr uint LED_PIN = 10;    // one WS2812 chain: the lamp ring first, then the status LEDs
constexpr int RING_COUNT = 16;
constexpr int LED_COUNT = 7;     // status LEDs: a bar of LED_COUNT - 1, then the power LED
constexpr int BAR_COUNT = LED_COUNT - 1;
constexpr uint GRIND_BUTTON_PIN = 18;
constexpr uint COFFEE_BUTTON_PIN = 19;
constexpr uint WATER_BUTTON_PIN = 20;
constexpr uint STEAM_BUTTON_PIN = 21;

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Temporary: replaces normal operation with a relay test. Set to false to use the machine.
constexpr bool RELAY_TEST_MODE = false;

constexpr float READY_BAND_C = 2.0f;     // this close to target counts as "at temperature"
constexpr float REHEAT_BAND_C = 8.0f;    // Ready falls back to Heating this far below brew temp
constexpr float COOLED_BAND_C = 3.0f;    // after steam, brewing allowed again within this of brew temp
constexpr float OVER_TEMP_C = 165.0f;    // hard cutoff

// Heater: PID sets a power level, delta-sigma turns it into whole mains cycles. Needs an SSR:
// a mechanical relay would be switching several times a second. The PID gains are settings.
constexpr uint32_t HEATER_STEP_MS = 20;  // one mains cycle: 20 for 50 Hz, 17 for 60 Hz
constexpr uint32_t HEATER_STALE_MS = 1000; // heater off if the main loop stops updating the power

constexpr uint32_t HEAT_TIMEOUT_MS = 5 * 60 * 1000;
constexpr uint32_t STEAM_IDLE_MS = 3 * 60 * 1000;
constexpr uint32_t IDLE_MS = 30 * 60 * 1000;

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t LONG_PRESS_MS = 2000;  // hold COFFEE to switch off
constexpr uint32_t SENSOR_PERIOD_MS = 250; // MAX6675 needs >= 220 ms per conversion
constexpr int SENSOR_FAULT_COUNT = 3;
constexpr uint32_t DISPLAY_PERIOD_MS = 250;
constexpr uint32_t STATUS_LOG_MS = 2000;
constexpr uint32_t LED_PERIOD_MS = 20;
constexpr uint8_t LED_BRIGHTNESS = 64;   // 0-255; WS2812s are dazzling at full power
constexpr uint8_t RING_R = 255, RING_G = 160, RING_B = 60;  // lamp colour: warm white
constexpr uint8_t RING_BRIGHTNESS = 128; // lamp brightness, 0-255
constexpr uint8_t LED_TEST_BRIGHTNESS = 16; // start-up test; all white is ~90 mA, safe on USB power
constexpr float AMBIENT_C = 20.0f;       // where the heating bar starts from

// Settings that can be changed from the web page and are kept in flash. Safety limits such as
// OVER_TEMP_C and HEAT_TIMEOUT_MS stay as constants above.
struct Settings {
    float brew_temp_c;
    float steam_temp_c;
    float kp;   // heater power per C below target (0.08: 12.5 C below = full power)
    float ki;   // heater power per C-second of accumulated error
    float kd;   // heater power taken off per C/s of temperature rise
    uint32_t grind_ms;
    uint32_t preinfuse_ms;
    uint32_t soak_ms;
    uint32_t brew_ms;
    uint32_t hot_water_ms;
};

// Used until settings are saved, and whenever the saved copy is invalid. The PID gains are
// starting points, tune them on the machine.
static constexpr Settings DEFAULT_SETTINGS = {
    97.0f,      // brew_temp_c
    120.0f,     // steam_temp_c
    0.04f,      // kp
    0.0004f,     // ki
    0.4f,       // kd
    65 * 100,  // grind_ms
    3 * 1000,   // preinfuse_ms
    2 * 1000,   // soak_ms
    25 * 1000,  // brew_ms
    30 * 1000,  // hot_water_ms
};

static Settings cfg = DEFAULT_SETTINGS;

static uint32_t millis() { return to_ms_since_boot(get_absolute_time()); }

#define LOG(fmt, ...) printf("[%7.3f] " fmt "\r\n", millis() / 1000.0f, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Buttons: debounced press edge plus a one-shot long-press event
// ---------------------------------------------------------------------------

// COFFEE has a hold (2 s: standby), so its press is reported on release, and only if it wasn't
// held. Otherwise pressing it in Ready would start the pump before the hold could be told apart.

enum ButtonId { BTN_GRIND, BTN_COFFEE, BTN_WATER, BTN_STEAM, BTN_COUNT };

struct Button {
    const char* name;
    uint pin;
    bool raw;
    uint32_t raw_changed;
    bool down;
    uint32_t down_since;
    bool long_sent;
    bool on_release;  // report the press when the button is let go, unless it was held
};

static Button buttons[BTN_COUNT] = {
    {"GRIND", GRIND_BUTTON_PIN},
    {"COFFEE", COFFEE_BUTTON_PIN, false, 0, false, 0, false, true},
    {"WATER", WATER_BUTTON_PIN},
    {"STEAM", STEAM_BUTTON_PIN},
};

struct ButtonEvents {
    bool pressed[BTN_COUNT];
    bool held[BTN_COUNT];
};

static void buttons_init() {
    for (Button& b : buttons) {
        gpio_init(b.pin);
        gpio_set_dir(b.pin, GPIO_IN);
        gpio_pull_down(b.pin);
    }
}

static ButtonEvents buttons_poll(uint32_t now) {
    ButtonEvents ev{};
    for (int i = 0; i < BTN_COUNT; ++i) {
        Button& b = buttons[i];
        bool raw = gpio_get(b.pin);
        if (raw != b.raw) {
            b.raw = raw;
            b.raw_changed = now;
        }
        if (raw != b.down && now - b.raw_changed >= DEBOUNCE_MS) {
            b.down = raw;
            if (raw) {
                b.down_since = now;
                b.long_sent = false;
            }
            if (raw != b.on_release && !b.long_sent) {  // the down edge, or a short press's release
                ev.pressed[i] = true;
                LOG("Button %s pressed", b.name);
            }
        }
        if (b.down && !b.long_sent && now - b.down_since >= LONG_PRESS_MS) {
            b.long_sent = true;
            ev.held[i] = true;
            LOG("Button %s held", b.name);
        }
    }
    return ev;
}

// ---------------------------------------------------------------------------
// MAX6675 thermocouple
// ---------------------------------------------------------------------------

static void thermocouple_init() {
    spi_init(SPI_PORT, 1000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

// Returns false on an open thermocouple or a missing/unwired chip.
static bool thermocouple_read(float* celsius) {
    uint8_t rx[2] = {};
    gpio_put(PIN_CS, 0);
    sleep_us(1);
    spi_read_blocking(SPI_PORT, 0x00, rx, 2);
    gpio_put(PIN_CS, 1);
    uint16_t word = (uint16_t)((rx[0] << 8) | rx[1]);
    if (word == 0xFFFF || (word & 0x8000)) return false;  // MISO floating / no chip
    if (word & 0x0004) return false;                      // open thermocouple
    *celsius = (word >> 3) * 0.25f;
    return true;
}

// ---------------------------------------------------------------------------
// SH1106 / SSD1306 OLED with a 5x7 font
// ---------------------------------------------------------------------------

// The 1.3" SH1106 has 132 columns of RAM with the visible 128 starting at column 2.
// Set to 0 for a 0.96" SSD1306.
constexpr uint8_t OLED_COLUMN_OFFSET = 2;

static uint8_t oled_buf[8][128];
static bool oled_ok = false;

static bool oled_cmds(const uint8_t* cmds, size_t n) {
    uint8_t pkt[2] = {0x00, 0};
    for (size_t i = 0; i < n; ++i) {
        pkt[1] = cmds[i];
        if (i2c_write_timeout_us(I2C_PORT, OLED_ADDRESS, pkt, 2, false, 2000) != 2) return false;
    }
    return true;
}

static bool oled_init() {
    static const uint8_t init[] = {
        0xAE,        // display off
        0xD5, 0x80,  // clock divide
        0xA8, 0x3F,  // multiplex 64
        0xD3, 0x00,  // display offset 0
        0x40,        // start line 0
        0x8D, 0x14,  // charge pump on (SSD1306; ignored by SH1106)
        0xA1, 0xC8,  // flip so (0,0) is top-left
        0xDA, 0x12,  // COM pins
        0x81, 0x7F,  // contrast
        0xD9, 0xF1,  // precharge
        0xDB, 0x40,  // VCOMH
        0xA4, 0xA6,  // RAM display, normal polarity
        0xAF,        // display on
    };
    return oled_cmds(init, sizeof(init));
}

// Writes one of the 8 pages (~3 ms at 400 kHz). The main loop sends one page per pass, so a
// whole-screen update doesn't hold everything else up for 25 ms and make the LEDs stutter.
// Page addressing: the one mode both SH1106 and SSD1306 support.
static bool oled_flush_page(int page) {
    uint8_t pkt[129];
    pkt[0] = 0x40;
    const uint8_t addr[] = {
        (uint8_t)(0xB0 | page),                       // page
        (uint8_t)(0x00 | (OLED_COLUMN_OFFSET & 0x0F)), // column low nibble
        (uint8_t)(0x10 | (OLED_COLUMN_OFFSET >> 4)),   // column high nibble
    };
    if (!oled_cmds(addr, sizeof(addr))) return false;
    memcpy(pkt + 1, oled_buf[page], 128);
    return i2c_write_timeout_us(I2C_PORT, OLED_ADDRESS, pkt, sizeof(pkt), false, 10000) == (int)sizeof(pkt);
}

// Column-major 5x7 glyphs, bit 0 = top row.
static const uint8_t* glyph(char c) {
    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46},
        {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, {0x36,0x49,0x49,0x49,0x36},
        {0x06,0x49,0x49,0x29,0x1E},
    };
    static const uint8_t letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
        {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
        {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
        {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
        {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
    };
    static const uint8_t dash[5] = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
    static const uint8_t blank[5] = {};
    if (c >= '0' && c <= '9') return digits[c - '0'];
    if (c >= 'a' && c <= 'z') c -= 32;
    if (c >= 'A' && c <= 'Z') return letters[c - 'A'];
    if (c == '-') return dash;
    if (c == ':') return colon;
    if (c == '.') return dot;
    return blank;
}

// Normal text: 6 px per char, 21 chars per line, one page tall.
static void draw_text(int x, int page, const char* s) {
    for (; *s && x <= 122; ++s, x += 6) {
        memcpy(&oled_buf[page][x], glyph(*s), 5);
    }
}

// Double-size text: 12 px per char, 10 chars per line, two pages tall.
static void draw_big(int x, int page, const char* s) {
    for (; *s && x <= 116; ++s, x += 12) {
        const uint8_t* g = glyph(*s);
        for (int col = 0; col < 5; ++col) {
            uint16_t tall = 0;
            for (int bit = 0; bit < 8; ++bit) {
                if (g[col] & (1 << bit)) tall |= 3u << (bit * 2);
            }
            for (int dx = 0; dx < 2; ++dx) {
                oled_buf[page][x + col * 2 + dx] = tall & 0xFF;
                oled_buf[page + 1][x + col * 2 + dx] = tall >> 8;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

enum class State {
    Standby, Heating, Ready, Grinding, Preinfuse, Soak, Brewing,
    HotWater, SteamHeat, SteamReady, CoolDown, Fault, COUNT
};

enum class Event {
    SensorFailed, OverTemp,
    GrindPressed, CoffeePressed, WaterPressed, SteamPressed, CoffeeHeld,
    TimerExpired, AtTemp, TooCold, CooledDown, Idle,
    RemoteStart, RemoteStandby,  // from Home Assistant
};

enum class FaultReason { None, Sensor, OverTemp, HeatTimeout };

// Everything a state drives, plus its timer and idle limit. The target and timer point at the
// live settings so changes take effect straight away; nullptr / 0 = none.
struct StateInfo {
    const char* name;          // max 10 chars (shown double size)
    const float* target_c;     // heater setpoint, nullptr = heater off
    bool pump, valve, grinder;
    const uint32_t* timer_ms;  // raises TimerExpired
    uint32_t idle_ms;          // raises Idle with no button activity
    const char* hint;
};

static const StateInfo STATES[] = {
    //  name          target              pump   valve  grind  timer              idle           hint
    {"STANDBY",    nullptr,            false, false, false, nullptr,           0,             "PRESS ANY BUTTON"},
    {"HEATING",    &cfg.brew_temp_c,   false, false, false, &HEAT_TIMEOUT_MS,  IDLE_MS,       "PLEASE WAIT"},
    {"READY",      &cfg.brew_temp_c,   false, false, false, nullptr,           IDLE_MS,       "HOLD COFFEE: OFF"},
    {"GRINDING",   &cfg.brew_temp_c,   false, false, true,  &cfg.grind_ms,     0,             "GRIND TO STOP"},
    {"PREINFUSE",  &cfg.brew_temp_c,   true,  true,  false, &cfg.preinfuse_ms, 0,             "COFFEE TO STOP"},
    {"SOAK",       &cfg.brew_temp_c,   false, true,  false, &cfg.soak_ms,      0,             "COFFEE TO STOP"},
    {"BREWING",    &cfg.brew_temp_c,   true,  true,  false, &cfg.brew_ms,      0,             "COFFEE TO STOP"},
    {"HOT WATER",  &cfg.brew_temp_c,   true,  false, false, &cfg.hot_water_ms, 0,             "WATER TO STOP"},
    {"STEAM HEAT", &cfg.steam_temp_c,  false, false, false, &HEAT_TIMEOUT_MS,  STEAM_IDLE_MS, "STEAM TO CANCEL"},
    {"STEAM RDY",  &cfg.steam_temp_c,  false, false, false, nullptr,           STEAM_IDLE_MS, "OPEN STEAM KNOB"},
    {"COOL DOWN",  &cfg.brew_temp_c,   false, false, false, nullptr,           0,             "WATER TO FLUSH"},
    {"FAULT",      nullptr,            false, false, false, nullptr,           0,             "PRESS TO RESET"},
};
static_assert(sizeof(STATES) / sizeof(STATES[0]) == (size_t)State::COUNT, "STATES table out of sync");

static const StateInfo& info(State s) { return STATES[(int)s]; }
static float target_c(const StateInfo& si) { return si.target_c ? *si.target_c : 0.0f; }
static uint32_t timer_ms(const StateInfo& si) { return si.timer_ms ? *si.timer_ms : 0; }

static const char* event_name(Event e) {
    switch (e) {
        case Event::SensorFailed:  return "sensor failed";
        case Event::OverTemp:      return "over temperature";
        case Event::GrindPressed:  return "GRIND pressed";
        case Event::CoffeePressed: return "COFFEE pressed";
        case Event::WaterPressed:  return "WATER pressed";
        case Event::SteamPressed:  return "STEAM pressed";
        case Event::CoffeeHeld:    return "COFFEE held";
        case Event::TimerExpired:  return "timer expired";
        case Event::AtTemp:        return "at temperature";
        case Event::TooCold:       return "too cold";
        case Event::CooledDown:    return "cooled down";
        case Event::Idle:          return "idle timeout";
        case Event::RemoteStart:   return "remote start";
        case Event::RemoteStandby: return "remote standby";
    }
    return "?";
}

static const char* fault_text(FaultReason r) {
    switch (r) {
        case FaultReason::Sensor:      return "THERMOCOUPLE";
        case FaultReason::OverTemp:    return "OVER TEMPERATURE";
        case FaultReason::HeatTimeout: return "HEAT TIMEOUT";
        case FaultReason::None:        break;
    }
    return "";
}

struct Machine {
    State state = State::Standby;
    uint32_t entered_ms = 0;
    uint32_t activity_ms = 0;
    FaultReason fault = FaultReason::None;
    float temp_c = 0.0f;
    bool temp_valid = false;
    int sensor_errors = 0;
};

static bool fault_cleared(const Machine& m) {
    return m.temp_valid && m.sensor_errors == 0 && m.temp_c < OVER_TEMP_C;
}

// The transition table. Returns the current state when the event is ignored.
static State next_state(const Machine& m, Event e) {
    State s = m.state;

    if (s == State::Fault) {
        bool button = e == Event::GrindPressed || e == Event::CoffeePressed ||
                      e == Event::WaterPressed || e == Event::SteamPressed;
        return button && fault_cleared(m) ? State::Standby : s;
    }
    if (e == Event::SensorFailed || e == Event::OverTemp) return State::Fault;
    if (e == Event::CoffeeHeld || e == Event::RemoteStandby) return State::Standby;

    switch (s) {
        case State::Standby:
            if (e == Event::GrindPressed || e == Event::CoffeePressed ||
                e == Event::WaterPressed || e == Event::SteamPressed ||
                e == Event::RemoteStart) return State::Heating;
            break;

        case State::Heating:
            if (e == Event::AtTemp)       return State::Ready;
            if (e == Event::GrindPressed) return State::Grinding;
            if (e == Event::SteamPressed) return State::SteamHeat;
            if (e == Event::TimerExpired) return State::Fault;
            if (e == Event::Idle)         return State::Standby;
            break;

        case State::Ready:
            if (e == Event::CoffeePressed) return State::Preinfuse;
            if (e == Event::GrindPressed)  return State::Grinding;
            if (e == Event::WaterPressed)  return State::HotWater;
            if (e == Event::SteamPressed)  return State::SteamHeat;
            if (e == Event::TooCold)       return State::Heating;
            if (e == Event::Idle)          return State::Standby;
            break;

        case State::Grinding:
            // Heating hands over to Ready straight away if already hot.
            if (e == Event::GrindPressed || e == Event::TimerExpired) return State::Heating;
            break;

        case State::Preinfuse:
            if (e == Event::CoffeePressed) return State::Ready;
            if (e == Event::TimerExpired)  return State::Soak;
            break;

        case State::Soak:
            if (e == Event::CoffeePressed) return State::Ready;
            if (e == Event::TimerExpired)  return State::Brewing;
            break;

        case State::Brewing:
            if (e == Event::CoffeePressed || e == Event::TimerExpired) return State::Ready;
            break;

        case State::HotWater:
            // CoolDown passes straight to Ready unless the block is still steam-hot.
            if (e == Event::WaterPressed || e == Event::TimerExpired) return State::CoolDown;
            break;

        case State::SteamHeat:
            if (e == Event::AtTemp)       return State::SteamReady;
            if (e == Event::SteamPressed || e == Event::Idle) return State::CoolDown;
            if (e == Event::TimerExpired) return State::Fault;
            break;

        case State::SteamReady:
            if (e == Event::SteamPressed || e == Event::Idle) return State::CoolDown;
            break;

        case State::CoolDown:
            if (e == Event::CooledDown)   return State::Ready;
            if (e == Event::WaterPressed) return State::HotWater;
            if (e == Event::SteamPressed) return State::SteamHeat;
            break;

        default:
            break;
    }
    return s;
}

static void enter_state(Machine& m, State next, Event cause, uint32_t now) {
    if (next == State::Fault) {
        m.fault = cause == Event::SensorFailed ? FaultReason::Sensor :
                  cause == Event::OverTemp     ? FaultReason::OverTemp :
                                                 FaultReason::HeatTimeout;
        LOG("FAULT: %s", fault_text(m.fault));
    } else if (m.state == State::Fault) {
        m.fault = FaultReason::None;
        LOG("Fault cleared");
    }
    LOG("%s -> %s (%s, %.2f C)", info(m.state).name, info(next).name, event_name(cause), m.temp_c);
    m.state = next;
    m.entered_ms = now;
    m.activity_ms = now;
}

// Remote start / standby from Home Assistant, picked up by the next state machine tick. Remote
// start only wakes the machine from Standby; a fault still needs a button press on the machine.
enum class RemoteCommand { None, Start, Standby };
static RemoteCommand remote_command = RemoteCommand::None;

// Collect this tick's events in priority order.
static int collect_events(const Machine& m, const ButtonEvents& b, uint32_t now, Event* out) {
    int n = 0;
    const StateInfo& si = info(m.state);
    uint32_t in_state = now - m.entered_ms;

    if (m.sensor_errors >= SENSOR_FAULT_COUNT)       out[n++] = Event::SensorFailed;
    if (m.temp_valid && m.temp_c >= OVER_TEMP_C)    out[n++] = Event::OverTemp;
    if (b.held[BTN_COFFEE])                          out[n++] = Event::CoffeeHeld;
    if (b.pressed[BTN_GRIND])                        out[n++] = Event::GrindPressed;
    if (b.pressed[BTN_COFFEE])                       out[n++] = Event::CoffeePressed;
    if (b.pressed[BTN_WATER])                        out[n++] = Event::WaterPressed;
    if (b.pressed[BTN_STEAM])                        out[n++] = Event::SteamPressed;
    if (remote_command == RemoteCommand::Start)      out[n++] = Event::RemoteStart;
    if (remote_command == RemoteCommand::Standby)    out[n++] = Event::RemoteStandby;
    uint32_t timer = timer_ms(si);
    float target = target_c(si);
    if (timer && in_state >= timer)                  out[n++] = Event::TimerExpired;
    if (m.temp_valid) {
        if (target > 0 && m.temp_c >= target - READY_BAND_C)  out[n++] = Event::AtTemp;
        if (m.temp_c < cfg.brew_temp_c - REHEAT_BAND_C)       out[n++] = Event::TooCold;
        if (m.temp_c <= cfg.brew_temp_c + COOLED_BAND_C)      out[n++] = Event::CooledDown;
    }
    if (si.idle_ms && now - m.activity_ms >= si.idle_ms) out[n++] = Event::Idle;
    return n;
}

// One transition per tick at most, so later events are judged against the new state next tick.
static void run_state_machine(Machine& m, const ButtonEvents& b, uint32_t now) {
    for (int i = 0; i < BTN_COUNT; ++i) {
        if (b.pressed[i]) m.activity_ms = now;
    }
    Event events[16];
    int n = collect_events(m, b, now, events);
    remote_command = RemoteCommand::None;
    for (int i = 0; i < n; ++i) {
        State next = next_state(m, events[i]);
        if (next != m.state) {
            enter_state(m, next, events[i], now);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------

static void relays_init() {
    const uint pins[] = {HEATER_PIN, PUMP_PIN, SOLENOID_PIN, GRINDER_PIN, SPARE1_PIN, SPARE2_PIN};
    for (uint pin : pins) {
        gpio_init(pin);
        gpio_put(pin, 0);  // off before the pin becomes an output
        gpio_set_dir(pin, GPIO_OUT);
    }
}

// Heater power, 0..1. Set by update_heater() in the main loop, used by the heater timer interrupt.
static volatile float heater_power = 0.0f;
static volatile uint32_t heater_power_ms = 0;  // when heater_power was last set

struct Pid {
    float integral;      // the I term, already scaled by ki
    float last_temp_c;
    float rate_c_per_s;  // smoothed rate of temperature change
    bool primed;         // last_temp_c is valid
};
static Pid pid;

// PID toward the current state's target. Called on every new reading (sampled) and on every
// state change. The D term works on the temperature, not the error, so a target change such as
// brew -> steam doesn't kick the output. Fails safe to zero power when there's no target or
// the reading is invalid.
static void update_heater(const Machine& m, bool sampled) {
    float target = target_c(info(m.state));
    float power = 0.0f;
    if (target <= 0 || !m.temp_valid) {
        pid = {};
    } else {
        constexpr float dt = SENSOR_PERIOD_MS / 1000.0f;
        float error = target - m.temp_c;
        if (sampled) {
            if (pid.primed) {
                // The MAX6675 reads in 0.25 C steps, so smooth the rate before using it.
                float rate = (m.temp_c - pid.last_temp_c) / dt;
                pid.rate_c_per_s += 0.2f * (rate - pid.rate_c_per_s);
            }
            pid.last_temp_c = m.temp_c;
            pid.primed = true;

            // Anti-windup: don't integrate further into a limit the output is already at.
            float out = cfg.kp * error + pid.integral - cfg.kd * pid.rate_c_per_s;
            if ((out < 1.0f || error < 0) && (out > 0.0f || error > 0)) {
                pid.integral += cfg.ki * error * dt;
                if (pid.integral < 0.0f) pid.integral = 0.0f;
                if (pid.integral > 1.0f) pid.integral = 1.0f;
            }
        }
        power = cfg.kp * error + pid.integral - cfg.kd * pid.rate_c_per_s;
        if (power < 0.0f) power = 0.0f;
        if (power > 1.0f) power = 1.0f;
    }
    heater_power = power;
    heater_power_ms = millis();
    if (power == 0.0f) gpio_put(HEATER_PIN, 0);  // off now, not at the next mains cycle
}

// Delta-sigma, once per mains cycle from a timer interrupt: add the demanded power to an
// accumulator, and switch on for this cycle whenever it reaches a whole cycle. This spreads
// the on-cycles as evenly as possible. Stepping whole cycles keeps the current symmetrical.
static bool heater_step(repeating_timer_t*) {
    static float accumulator = 0.0f;
    float power = heater_power;
    if (millis() - heater_power_ms > HEATER_STALE_MS) power = 0.0f;  // main loop has stopped
    accumulator += power;
    bool on = accumulator >= 1.0f;
    if (on) accumulator -= 1.0f;
    if (power == 0.0f) accumulator = 0.0f;
    gpio_put(HEATER_PIN, on);
    return true;
}

static void drive_relays(const Machine& m) {
    const StateInfo& si = info(m.state);
    gpio_put(PUMP_PIN, si.pump);
    gpio_put(SOLENOID_PIN, si.valve);
    gpio_put(GRINDER_PIN, si.grinder);
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

// Time left in a timed step (grind, shot, hot water), 0 otherwise. The heat timeout isn't shown.
static uint32_t time_left_ms(const Machine& m, uint32_t now) {
    const StateInfo& si = info(m.state);
    if (si.timer_ms == &HEAT_TIMEOUT_MS) return 0;
    uint32_t timer = timer_ms(si);
    uint32_t in_state = now - m.entered_ms;
    return in_state < timer ? timer - in_state : 0;
}

static void render(const Machine& m, uint32_t now) {
    const StateInfo& si = info(m.state);
    char line[22];
    memset(oled_buf, 0, sizeof(oled_buf));

    draw_big(0, 0, si.name);

    if (m.temp_valid) snprintf(line, sizeof(line), "TEMP %5.1f C", m.temp_c);
    else snprintf(line, sizeof(line), "TEMP ---");
    draw_text(0, 3, line);

    if (target_c(si) > 0) {
        snprintf(line, sizeof(line), "SET  %3d C  %3d%%", (int)target_c(si), (int)(heater_power * 100.0f + 0.5f));
        draw_text(0, 4, line);
    }

    if (m.state == State::Fault) {
        draw_text(0, 5, fault_text(m.fault));
    } else if (uint32_t left = time_left_ms(m, now)) {
        snprintf(line, sizeof(line), "TIME %2lu S", (unsigned long)((left + 999) / 1000));
        draw_text(0, 5, line);
    }

    draw_text(0, 7, si.hint);
}

// ---------------------------------------------------------------------------
// LEDs: the lamp ring, then the status LEDs, where colour says which state
// and the bar says how far along it is
// ---------------------------------------------------------------------------

enum class LedPattern {
    Off,
    Breathe,    // whole bar pulses slowly
    Solid,      // whole bar on
    WarmUp,     // bar fills as the temperature moves from from_c to the state's target
    Countdown,  // bar empties as the state's timer runs out
    Shot,       // bar fills across the whole shot: pre-infuse, soak and extract
    Flash,      // blinks; the number of LEDs is the FaultReason (1 sensor, 2 over temp, 3 timeout)
};

struct LedStyle {
    uint8_t r, g, b;
    LedPattern pattern;
    const float* from_c;  // WarmUp only
};

static const LedStyle LED_STYLES[] = {
    //  r    g    b    pattern                 from
    {  0,   0,   0,  LedPattern::Off,       nullptr},            // STANDBY     off (the power LED breathes)
    {255,  70,   0,  LedPattern::WarmUp,    &AMBIENT_C},         // HEATING     orange
    {  0, 255,   0,  LedPattern::Solid,     nullptr},            // READY       green
    {255, 200,   0,  LedPattern::Countdown, nullptr},            // GRINDING    yellow
    {255, 255, 255,  LedPattern::Shot,      nullptr},            // PREINFUSE   white
    {255, 255, 255,  LedPattern::Shot,      nullptr},            // SOAK        white
    {255, 255, 255,  LedPattern::Shot,      nullptr},            // BREWING     white
    {  0, 255, 255,  LedPattern::Countdown, nullptr},            // HOT WATER   cyan
    {  0,   0, 255,  LedPattern::WarmUp,    &cfg.brew_temp_c},   // STEAM HEAT  blue
    {  0,   0, 255,  LedPattern::Solid,     nullptr},            // STEAM RDY   blue
    {255,   0, 255,  LedPattern::WarmUp,    &cfg.steam_temp_c},  // COOL DOWN   magenta, fills as it cools
    {255,   0,   0,  LedPattern::Flash,     nullptr},            // FAULT       red
};
static_assert(sizeof(LED_STYLES) / sizeof(LED_STYLES[0]) == (size_t)State::COUNT, "LED_STYLES table out of sync");

static PIO led_pio;
static uint led_sm;
static bool leds_ok = false;

static bool leds_init() {
    uint offset;
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &led_pio, &led_sm, &offset,
                                                          LED_PIN, 1, true)) {
        return false;
    }
    ws2812_program_init(led_pio, led_sm, offset, LED_PIN, 800000);
    return true;
}

static void put_pixel(uint8_t r, uint8_t g, uint8_t b, float k) {
    uint32_t rr = (uint32_t)(r * k + 0.5f), gg = (uint32_t)(g * k + 0.5f), bb = (uint32_t)(b * k + 0.5f);
    pio_sm_put_blocking(led_pio, led_sm, (gg << 24) | (rr << 16) | (bb << 8));  // GRB, MSB first
}

// Breathes from dim to full and back over period_ms. A cosine eases over both ends, and the
// gamma of 2.2 makes the brightness change evenly to the eye; without it the LEDs seem to rush
// through the dim part and linger at the top. Bottom of the breath: 0.35^2.2 = 10%.
static float pulse(uint32_t now, uint32_t period_ms) {
    float phase = (float)(now % period_ms) / period_ms;
    float eye = 0.35f + 0.65f * (0.5f - 0.5f * cosf(6.2831853f * phase));
    return powf(eye, 2.2f);
}

static void leds_render(const Machine& m, uint32_t now) {
    const LedStyle& st = LED_STYLES[(int)m.state];
    const StateInfo& si = info(m.state);
    uint32_t in_state = now - m.entered_ms;
    float level[BAR_COUNT] = {};
    float fill = -1.0f;  // >= 0 for the bar patterns
    float warm = 1.0f;   // power LED: 0 = cold (red) .. 1 = at temperature (green)

    switch (st.pattern) {
        case LedPattern::Off:
            break;
        case LedPattern::Breathe:
            for (float& l : level) l = pulse(now, 4000);
            break;
        case LedPattern::Solid:
            for (float& l : level) l = 1.0f;
            break;
        case LedPattern::WarmUp:
            fill = m.temp_valid ? (m.temp_c - *st.from_c) / (target_c(si) - *st.from_c) : 0.0f;
            // Power LED: heating (brew or steam) runs from cold to this target, so steam
            // temperature is green and the LED doesn't drop back to red when Steam is pressed.
            // Cool Down falls towards its target, so it follows the bar instead.
            if (target_c(si) > *st.from_c) {
                warm = m.temp_valid ? (m.temp_c - AMBIENT_C) / (target_c(si) - AMBIENT_C) : 0.0f;
            } else {
                warm = fill;
            }
            break;
        case LedPattern::Countdown:
            fill = 1.0f - (float)in_state / timer_ms(si);
            break;
        case LedPattern::Shot: {
            // Earlier stages always ran their full time, or we'd have gone back to Ready.
            uint32_t done = in_state;
            if (m.state != State::Preinfuse) done += cfg.preinfuse_ms;
            if (m.state == State::Brewing) done += cfg.soak_ms;
            fill = (float)done / (cfg.preinfuse_ms + cfg.soak_ms + cfg.brew_ms);
            break;
        }
        case LedPattern::Flash:
            if (now % 1000 < 500) {
                for (int i = 0; i < (int)m.fault && i < BAR_COUNT; ++i) level[i] = 1.0f;
            }
            break;
    }

    if (fill >= 0.0f) {
        // Always at least one LED, and the leading one pulses so the bar never looks stuck.
        int lit = (int)ceilf(fill * BAR_COUNT);
        if (lit < 1) lit = 1;
        if (lit > BAR_COUNT) lit = BAR_COUNT;
        for (int i = 0; i < lit - 1; ++i) level[i] = 1.0f;
        level[lit - 1] = pulse(now, 1000);
    }

    // The ring replaces the original lamp: on whenever the machine is.
    float ring = m.state == State::Standby ? 0.0f : RING_BRIGHTNESS / 255.0f;
    for (int i = 0; i < RING_COUNT; ++i) put_pixel(RING_R, RING_G, RING_B, ring);
    for (float l : level) put_pixel(st.r, st.g, st.b, l * LED_BRIGHTNESS / 255.0f);

    // Power LED, the last on the strip: breathing red in standby, flashing red in a fault, otherwise
    // red -> orange -> yellow -> green as the temperature reaches the target (the hue of
    // warm: 0 to 120 degrees), and green once it's there.
    float k = LED_BRIGHTNESS / 255.0f;
    if (m.state == State::Standby) {
        put_pixel(255, 0, 0, pulse(now, 4000) * k);
    } else if (m.state == State::Fault) {
        put_pixel(255, 0, 0, now % 1000 < 500 ? k : 0.0f);  // in step with the bar's fault flash
    } else {
        if (warm < 0.0f) warm = 0.0f;
        if (warm > 1.0f) warm = 1.0f;
        float r = warm < 0.5f ? 1.0f : 2.0f - 2.0f * warm;
        float g = warm < 0.5f ? 2.0f * warm : 1.0f;
        put_pixel((uint8_t)(r * 255.0f), (uint8_t)(g * 255.0f), 0, k);
    }
}

// Start-up check of the whole chain: every LED red, green, blue and white for 1 s each
// (one colour channel at a time), then one white LED walks from the first ring LED to the last
// status LED (wiring order and count). Repeats until a button is pressed.
static void leds_test_render(uint32_t elapsed) {
    static const uint8_t colours[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};
    constexpr int COLOURS = sizeof(colours) / sizeof(colours[0]);
    constexpr int TOTAL = RING_COUNT + LED_COUNT;
    constexpr uint32_t COLOUR_MS = 1000, STEP_MS = 150;
    uint32_t t = elapsed % (COLOURS * COLOUR_MS + TOTAL * STEP_MS);
    float k = LED_TEST_BRIGHTNESS / 255.0f;

    for (int i = 0; i < TOTAL; ++i) {
        if (t < COLOURS * COLOUR_MS) {
            const uint8_t* c = colours[t / COLOUR_MS];
            put_pixel(c[0], c[1], c[2], k);
        } else {
            bool lit = i == (int)((t - COLOURS * COLOUR_MS) / STEP_MS);
            put_pixel(255, 255, 255, lit ? k : 0.0f);
        }
    }
}

// ---------------------------------------------------------------------------
// Relay test (RELAY_TEST_MODE): relays switched from the web page
// ---------------------------------------------------------------------------

constexpr uint32_t RELAY_TEST_MAX_ON_MS = 30 * 1000;  // each relay switches itself off after this
constexpr uint32_t RELAY_TEST_PAGE_MS = 3000;         // all off if the page stops polling for this long

// Runs instead of the state machine: the buttons do nothing and the heater stays off.
struct TestRelay {
    const char* key;   // API name
    const char* name;  // display name
    uint pin;
    bool on;
    uint32_t since;
};
static TestRelay test_relays[] = {
    {"pump", "PUMP", PUMP_PIN},
    {"valve", "SOLENOID", SOLENOID_PIN},
    {"grinder", "GRINDER", GRINDER_PIN},
};
static uint32_t test_page_seen_ms = 0;  // last status poll from the web page

static void test_relay_set(TestRelay& r, bool on, uint32_t now, const char* why) {
    r.since = now;  // switching on again restarts the 30 s
    if (r.on == on) return;
    r.on = on;
    gpio_put(r.pin, on);
    LOG("Relay test: %s %s (%s)", r.name, on ? "ON" : "OFF", why);
}

static uint32_t test_relay_left_ms(const TestRelay& r, uint32_t now) {
    uint32_t on_for = now - r.since;
    return r.on && on_for < RELAY_TEST_MAX_ON_MS ? RELAY_TEST_MAX_ON_MS - on_for : 0;
}

static void relay_test_tick(uint32_t now) {
    bool page_gone = now - test_page_seen_ms > RELAY_TEST_PAGE_MS;
    for (TestRelay& r : test_relays) {
        if (!r.on) continue;
        if (page_gone) test_relay_set(r, false, now, "web page stopped polling");
        else if (!test_relay_left_ms(r, now)) test_relay_set(r, false, now, "30 s limit");
    }
}

static void relay_test_render(uint32_t now) {
    char line[22];
    memset(oled_buf, 0, sizeof(oled_buf));
    draw_big(0, 0, "RELAY TEST");
    for (int i = 0; i < (int)count_of(test_relays); ++i) {
        const TestRelay& r = test_relays[i];
        if (r.on) snprintf(line, sizeof(line), "%-9s ON  %2lu S", r.name, (unsigned long)((test_relay_left_ms(r, now) + 999) / 1000));
        else snprintf(line, sizeof(line), "%-9s OFF", r.name);
        draw_text(0, 3 + i, line);
    }
    draw_text(0, 7, "SWITCH FROM WEB PAGE");
}

// Ring off. Status LED 1 cyan = pump, 2 white = solenoid, 3 yellow = grinder, the rest dim magenta.
static void relay_test_leds() {
    static const uint8_t colours[][3] = {{0, 255, 255}, {255, 255, 255}, {255, 200, 0}};
    float k = LED_BRIGHTNESS / 255.0f;
    for (int i = 0; i < RING_COUNT; ++i) put_pixel(0, 0, 0, 0.0f);
    for (int i = 0; i < LED_COUNT; ++i) {
        if (i < (int)count_of(test_relays)) put_pixel(colours[i][0], colours[i][1], colours[i][2], test_relays[i].on ? k : 0.0f);
        else put_pixel(255, 0, 255, k * 0.25f);
    }
}

// ---------------------------------------------------------------------------
// Settings: limits, validation and storage in the last flash sector
// ---------------------------------------------------------------------------

// One row per setting: drives validation, the JSON API and the web form. Times are stored in
// ms and shown in seconds.
struct SettingField {
    const char* key;
    const char* group;
    const char* label;
    const char* unit;
    float min, max, step;
    size_t offset;  // into Settings
    bool ms;        // a uint32_t ms field shown in seconds; otherwise a float
};

#define FIELD(key, group, label, unit, min, max, step, ms) \
    {#key, group, label, unit, min, max, step, offsetof(Settings, key), ms}

static const SettingField SETTING_FIELDS[] = {
    FIELD(brew_temp_c,  "Temperatures", "Brew temperature",  "°C", 85,  100,   0.5f,    false),
    FIELD(steam_temp_c, "Temperatures", "Steam temperature", "°C", 110, 150,   1,       false),
    FIELD(kp,           "Heater PID",   "Kp (per °C)",  "",        0,   1,     0.001f,  false),
    FIELD(ki,           "Heater PID",   "Ki (per °C·s)", "",  0,   0.05f, 0.0001f, false),
    FIELD(kd,           "Heater PID",   "Kd (per °C/s)", "",       0,   5,     0.01f,   false),
    FIELD(grind_ms,     "Times",        "Grind",             "s",       1,   60,    0.5f,    true),
    FIELD(preinfuse_ms, "Times",        "Pre-infuse",        "s",       1,   10,    0.5f,    true),
    FIELD(soak_ms,      "Times",        "Soak",              "s",       1,   20,    0.5f,    true),
    FIELD(brew_ms,      "Times",        "Extract",           "s",       5,   60,    1,       true),
    FIELD(hot_water_ms, "Times",        "Hot water",         "s",       5,   120,   1,       true),
};
#undef FIELD
static_assert(OVER_TEMP_C > 150 + 10, "steam temperature limit must stay well below the cut-out");

// The value as shown on the web page.
static float field_get(const Settings& s, const SettingField& f) {
    const uint8_t* p = (const uint8_t*)&s + f.offset;
    return f.ms ? *(const uint32_t*)p / 1000.0f : *(const float*)p;
}

static void field_set(Settings& s, const SettingField& f, float v) {
    uint8_t* p = (uint8_t*)&s + f.offset;
    if (f.ms) *(uint32_t*)p = (uint32_t)(v * 1000.0f + 0.5f);
    else *(float*)p = v;
}

static bool field_ok(const SettingField& f, float v) { return isfinite(v) && v >= f.min && v <= f.max; }

static bool settings_valid(const Settings& s) {
    for (const SettingField& f : SETTING_FIELDS) {
        if (!field_ok(f, field_get(s, f))) return false;
    }
    return true;
}

constexpr uint32_t SETTINGS_MAGIC = 0x5A434F46;  // "ZCOF"
constexpr uint32_t SETTINGS_FLASH_OFFSET = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;

struct StoredSettings {
    uint32_t magic;
    uint32_t size;  // sizeof(Settings), so a changed layout reads as invalid
    Settings settings;
    uint32_t check;
};
static_assert(sizeof(StoredSettings) <= FLASH_PAGE_SIZE, "settings must fit in one flash page");

static uint32_t settings_check(const Settings& s) {
    uint32_t h = 2166136261u;  // FNV-1a
    const uint8_t* p = (const uint8_t*)&s;
    for (size_t i = 0; i < sizeof(s); ++i) h = (h ^ p[i]) * 16777619u;
    return h;
}

static void settings_load() {
    const StoredSettings* stored = (const StoredSettings*)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    if (stored->magic == SETTINGS_MAGIC && stored->size == sizeof(Settings) &&
        stored->check == settings_check(stored->settings) && settings_valid(stored->settings)) {
        cfg = stored->settings;
        LOG("Settings loaded from flash");
    } else {
        cfg = DEFAULT_SETTINGS;
        LOG("No saved settings, using defaults");
    }
}

// Runs with interrupts off, so the heater timer can't run: make sure the heater is off.
static void settings_flash_write(void* page) {
    gpio_put(HEATER_PIN, 0);
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, (const uint8_t*)page, FLASH_PAGE_SIZE);
}

static void settings_save() {
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    StoredSettings stored = {SETTINGS_MAGIC, sizeof(Settings), cfg, settings_check(cfg)};
    memcpy(page, &stored, sizeof(stored));
    int rc = flash_safe_execute(settings_flash_write, page, 100);
    if (rc == PICO_OK) LOG("Settings saved to flash");
    else LOG("Settings save failed (%d)", rc);
}

// ---------------------------------------------------------------------------
// Temperature history for the web graph: one point a second
// ---------------------------------------------------------------------------

constexpr int HISTORY_LEN = 900;  // 15 minutes
constexpr uint32_t HISTORY_PERIOD_MS = 1000;
constexpr int16_t HISTORY_NO_TEMP = INT16_MIN;

struct HistoryPoint {
    int16_t temp_x10;    // HISTORY_NO_TEMP when there was no valid reading
    int16_t target_x10;  // 0 = heater off
    uint8_t power;       // %
};
static HistoryPoint history[HISTORY_LEN];
static int history_next = 0, history_count = 0;

static void history_add(const Machine& m) {
    HistoryPoint& h = history[history_next];
    h.temp_x10 = m.temp_valid ? (int16_t)lroundf(m.temp_c * 10.0f) : HISTORY_NO_TEMP;
    h.target_x10 = (int16_t)lroundf(target_c(info(m.state)) * 10.0f);
    h.power = (uint8_t)lroundf(heater_power * 100.0f);
    history_next = (history_next + 1) % HISTORY_LEN;
    if (history_count < HISTORY_LEN) ++history_count;
}

// ---------------------------------------------------------------------------
// Web interface: Wi-Fi, mDNS (zanussi.local) and a small HTTP server on lwIP's raw API.
// Everything runs from cyw43_arch_poll() in the main loop, never from an interrupt.
// ---------------------------------------------------------------------------

#if WEB_ENABLED

constexpr const char* WEB_HOSTNAME = "zanussi";
constexpr uint32_t WIFI_RETRY_MS = 10 * 1000;
constexpr int HTTP_MAX_CONNS = 4;
constexpr int HTTP_REQ_MAX = 2048;
constexpr int HTTP_BODY_MAX = 16 * 1024;  // the history JSON is the largest, about 13 KB
constexpr uint32_t HTTP_TIMEOUT_MS = 10 * 1000;

struct HttpConn {
    tcp_pcb* pcb;  // nullptr = free slot
    uint32_t opened_ms;
    int req_len;
    bool answered;
    char req[HTTP_REQ_MAX + 1];
    char head[192];
    int head_len;
    const char* body;
    int body_len;
    int sent;      // bytes of head + body queued so far
    bool closing;  // lwIP couldn't close yet; retried from the poll callback
    char buf[HTTP_BODY_MAX];
};
static HttpConn http_conns[HTTP_MAX_CONNS];

static bool web_up = false;  // Wi-Fi chip initialised
static const Machine* web_machine = nullptr;
static bool settings_save_pending = false;  // saved from the main loop, not inside lwIP

static err_t http_poll(void* arg, tcp_pcb* pcb);
static void http_err(void* arg, err_t err);

// Graceful close: lwIP still sends whatever is queued. The callbacks are cleared first because
// lwIP may free the pcb inside tcp_close(). If it can't close yet, try again from the poll.
static void http_close(HttpConn* c) {
    tcp_pcb* pcb = c->pcb;
    if (!pcb) return;
    tcp_arg(pcb, nullptr);
    tcp_recv(pcb, nullptr);
    tcp_sent(pcb, nullptr);
    tcp_err(pcb, nullptr);
    tcp_poll(pcb, nullptr, 0);
    if (tcp_close(pcb) == ERR_OK) {
        c->pcb = nullptr;
        c->closing = false;
    } else {
        c->closing = true;
        tcp_arg(pcb, c);
        tcp_err(pcb, http_err);
        tcp_poll(pcb, http_poll, 2);
    }
}

// Queue as much of the response as lwIP has room for; the rest goes from the sent callback.
static void http_send_more(HttpConn* c) {
    if (!c->pcb || c->closing) return;
    int total = c->head_len + c->body_len;
    while (c->sent < total) {
        const char* p;
        int n;
        if (c->sent < c->head_len) {
            p = c->head + c->sent;
            n = c->head_len - c->sent;
        } else {
            p = c->body + (c->sent - c->head_len);
            n = total - c->sent;
        }
        int room = tcp_sndbuf(c->pcb);
        if (n > room) n = room;
        if (n <= 0) break;
        if (tcp_write(c->pcb, p, n, TCP_WRITE_FLAG_COPY) != ERR_OK) break;
        c->sent += n;
    }
    tcp_output(c->pcb);
    if (c->sent >= total) http_close(c);
}

static void http_respond(HttpConn* c, int status, const char* type, const char* body, int len) {
    const char* reason = status == 200 ? "OK" : status == 400 ? "Bad Request" :
                         status == 404 ? "Not Found" : status == 405 ? "Method Not Allowed" :
                         status == 409 ? "Conflict" : status == 413 ? "Payload Too Large" : "Error";
    c->head_len = snprintf(c->head, sizeof(c->head),
                           "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                           "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
                           status, reason, type, len);
    c->body = body;
    c->body_len = len;
    c->sent = 0;
    c->answered = true;
    http_send_more(c);
}

static void http_text(HttpConn* c, int status, const char* text) {
    http_respond(c, status, "text/plain", text, (int)strlen(text));
}

// Appends to c->buf; returns false once it's full.
struct JsonOut {
    char* buf;
    int len, cap;
    bool ok = true;
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (!ok) return;
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(buf + len, cap - len, fmt, ap);
        va_end(ap);
        if (n < 0 || n >= cap - len) ok = false;
        else len += n;
    }
};

static void http_json(HttpConn* c, JsonOut& j) {
    if (j.ok) http_respond(c, 200, "application/json", j.buf, j.len);
    else http_text(c, 500, "response too large");
}

static void api_status(HttpConn* c) {
    const Machine& m = *web_machine;
    if (RELAY_TEST_MODE) test_page_seen_ms = millis();  // the page is still open
    const StateInfo& si = info(m.state);
    uint32_t now = millis();
    float target = target_c(si);
    int32_t rssi = 0;
    cyw43_wifi_get_rssi(&cyw43_state, &rssi);
    JsonOut j{c->buf, 0, HTTP_BODY_MAX};
    j.add("{\"state\":\"%s\",\"fault\":\"%s\",\"temp_valid\":%s,\"temp\":%.2f,\"target\":%.1f,",
          RELAY_TEST_MODE ? "RELAY TEST" : si.name, fault_text(m.fault), m.temp_valid ? "true" : "false",
          m.temp_c, target);
    j.add("\"power\":%.1f,\"p\":%.3f,\"i\":%.3f,\"rate\":%.3f,\"time_left\":%.1f,",
          heater_power * 100.0f, target > 0 ? cfg.kp * (target - m.temp_c) : 0.0f, pid.integral,
          pid.rate_c_per_s, time_left_ms(m, now) / 1000.0f);
    bool pump = si.pump, valve = si.valve, grinder = si.grinder;
    if (RELAY_TEST_MODE) {
        pump = test_relays[0].on;
        valve = test_relays[1].on;
        grinder = test_relays[2].on;
    }
    j.add("\"pump\":%s,\"valve\":%s,\"grinder\":%s,\"rssi\":%ld,\"uptime\":%lu,\"test_mode\":%s",
          pump ? "true" : "false", valve ? "true" : "false", grinder ? "true" : "false",
          (long)rssi, (unsigned long)(now / 1000), RELAY_TEST_MODE ? "true" : "false");
    if (RELAY_TEST_MODE) {
        j.add(",\"test_limit_s\":%lu,\"relays\":[", (unsigned long)(RELAY_TEST_MAX_ON_MS / 1000));
        for (size_t i = 0; i < count_of(test_relays); ++i) {
            const TestRelay& r = test_relays[i];
            j.add("%s{\"key\":\"%s\",\"name\":\"%s\",\"on\":%s,\"left\":%.1f}", i ? "," : "", r.key, r.name,
                  r.on ? "true" : "false", test_relay_left_ms(r, now) / 1000.0f);
        }
        j.add("]");
    }
    j.add("}");
    http_json(c, j);
}

static void api_history(HttpConn* c) {
    JsonOut j{c->buf, 0, HTTP_BODY_MAX};
    int first = (history_next - history_count + HISTORY_LEN) % HISTORY_LEN;
    const char* names[] = {"temp", "target", "power"};
    j.add("{\"period_s\":%lu", (unsigned long)(HISTORY_PERIOD_MS / 1000));
    for (int series = 0; series < 3; ++series) {
        j.add(",\"%s\":[", names[series]);
        for (int k = 0; k < history_count; ++k) {
            const HistoryPoint& h = history[(first + k) % HISTORY_LEN];
            const char* sep = k ? "," : "";
            if (series == 0) {
                if (h.temp_x10 == HISTORY_NO_TEMP) j.add("%snull", sep);
                else j.add("%s%d.%d", sep, h.temp_x10 / 10, abs(h.temp_x10 % 10));
            } else if (series == 1) {
                if (h.target_x10 <= 0) j.add("%snull", sep);
                else j.add("%s%d.%d", sep, h.target_x10 / 10, h.target_x10 % 10);
            } else {
                j.add("%s%u", sep, h.power);
            }
        }
        j.add("]");
    }
    j.add("}");
    http_json(c, j);
}

static void add_values(JsonOut& j, const Settings& s) {
    j.add("{");
    for (size_t i = 0; i < count_of(SETTING_FIELDS); ++i) {
        const SettingField& f = SETTING_FIELDS[i];
        j.add("%s\"%s\":%.6g", i ? "," : "", f.key, field_get(s, f));
    }
    j.add("}");
}

static void api_settings(HttpConn* c) {
    JsonOut j{c->buf, 0, HTTP_BODY_MAX};
    j.add("{\"values\":");
    add_values(j, cfg);
    j.add(",\"defaults\":");
    add_values(j, DEFAULT_SETTINGS);
    j.add(",\"fields\":[");
    for (size_t i = 0; i < count_of(SETTING_FIELDS); ++i) {
        const SettingField& f = SETTING_FIELDS[i];
        j.add("%s{\"key\":\"%s\",\"group\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",\"min\":%g,\"max\":%g,\"step\":%g}",
              i ? "," : "", f.key, f.group, f.label, f.unit, f.min, f.max, f.step);
    }
    j.add("]}");
    http_json(c, j);
}

// Body: key=value&key=value (application/x-www-form-urlencoded, numbers only). Settings left
// out keep their current value. Nothing changes unless every value is valid.
static void api_settings_post(HttpConn* c, char* body) {
    Settings next = cfg;
    for (char* pair = strtok(body, "&"); pair; pair = strtok(nullptr, "&")) {
        char* eq = strchr(pair, '=');
        if (!eq) continue;
        *eq = 0;
        const SettingField* field = nullptr;
        for (const SettingField& f : SETTING_FIELDS) {
            if (strcmp(f.key, pair) == 0) field = &f;
        }
        char* end;
        float v = strtof(eq + 1, &end);
        if (!field || end == eq + 1 || *end) {
            snprintf(c->buf, HTTP_BODY_MAX, "Unknown setting or bad number: %s", pair);
            http_text(c, 400, c->buf);
            return;
        }
        if (!field_ok(*field, v)) {
            snprintf(c->buf, HTTP_BODY_MAX, "%s must be between %g and %g", field->label, field->min, field->max);
            http_text(c, 400, c->buf);
            return;
        }
        field_set(next, *field, v);
    }
    cfg = next;
    settings_save_pending = true;
    LOG("Settings changed from the web page");
    api_settings(c);
}

// Body: relay=pump&on=1. Only in relay test mode.
static void api_relay_post(HttpConn* c, char* body) {
    if (!RELAY_TEST_MODE) return http_text(c, 409, "Relays can only be switched in relay test mode");
    TestRelay* relay = nullptr;
    int on = -1;
    for (char* pair = strtok(body, "&"); pair; pair = strtok(nullptr, "&")) {
        char* eq = strchr(pair, '=');
        if (!eq) continue;
        *eq = 0;
        if (strcmp(pair, "relay") == 0) {
            for (TestRelay& r : test_relays) {
                if (strcmp(r.key, eq + 1) == 0) relay = &r;
            }
        } else if (strcmp(pair, "on") == 0) {
            on = atoi(eq + 1) ? 1 : 0;
        }
    }
    if (!relay || on < 0) return http_text(c, 400, "Need relay=pump|valve|grinder and on=0|1");
    uint32_t now = millis();
    test_page_seen_ms = now;
    test_relay_set(*relay, on, now, "web page");
    api_status(c);
}

static void http_handle(HttpConn* c, const char* method, char* path, char* body) {
    if (char* q = strchr(path, '?')) *q = 0;
    bool get = strcmp(method, "GET") == 0, post = strcmp(method, "POST") == 0;

    if (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0) {
        if (!get) return http_text(c, 405, "GET only");
        return http_respond(c, 200, "text/html; charset=utf-8", WEB_PAGE, sizeof(WEB_PAGE) - 1);
    }
    if (strcmp(path, "/api/status") == 0 && get) return api_status(c);
    if (strcmp(path, "/api/history") == 0 && get) return api_history(c);
    if (strcmp(path, "/api/settings") == 0 && get) return api_settings(c);
    if (strcmp(path, "/api/settings") == 0 && post) return api_settings_post(c, body);
    if (strcmp(path, "/api/relay") == 0 && post) return api_relay_post(c, body);
    if (strcmp(path, "/api/settings/reset") == 0 && post) {
        cfg = DEFAULT_SETTINGS;
        settings_save_pending = true;
        LOG("Settings reset to defaults from the web page");
        return api_settings(c);
    }
    http_text(c, 404, "Not found");
}

// Called whenever more of the request has arrived. Answers once the headers and the whole
// body (Content-Length) are in.
static void http_try_request(HttpConn* c) {
    char* end = strstr(c->req, "\r\n\r\n");
    if (!end) {
        if (c->req_len >= HTTP_REQ_MAX) http_text(c, 413, "Request too large");
        return;
    }
    char* body = end + 4;
    int content_length = 0;
    for (char* line = strstr(c->req, "\r\n"); line && line < end; line = strstr(line + 2, "\r\n")) {
        if (strncasecmp(line + 2, "Content-Length:", 15) == 0) content_length = atoi(line + 17);
    }
    if (content_length < 0) content_length = 0;
    if (body + content_length > c->req + HTTP_REQ_MAX) return http_text(c, 413, "Request too large");
    if (body + content_length > c->req + c->req_len) return;  // wait for the rest
    body[content_length] = 0;

    char* method = strtok(c->req, " ");
    char* path = strtok(nullptr, " ");
    if (!method || !path) return http_text(c, 400, "Bad request");
    http_handle(c, method, path, body);
}

static err_t http_recv(void* arg, tcp_pcb* pcb, pbuf* p, err_t err) {
    HttpConn* c = (HttpConn*)arg;
    if (!p) {  // the browser closed its end
        http_close(c);
        return ERR_OK;
    }
    if (err == ERR_OK && !c->answered) {
        int n = p->tot_len;
        if (n > HTTP_REQ_MAX - c->req_len) n = HTTP_REQ_MAX - c->req_len;
        pbuf_copy_partial(p, c->req + c->req_len, n, 0);
        c->req_len += n;
        c->req[c->req_len] = 0;
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    if (!c->answered) http_try_request(c);
    return ERR_OK;
}

static err_t http_sent(void* arg, tcp_pcb*, u16_t) {
    http_send_more((HttpConn*)arg);
    return ERR_OK;
}

static void http_err(void* arg, err_t) {
    ((HttpConn*)arg)->pcb = nullptr;  // lwIP has already freed the pcb
}

static err_t http_poll(void* arg, tcp_pcb* pcb) {
    HttpConn* c = (HttpConn*)arg;
    if (c->closing) {
        http_close(c);
        return ERR_OK;
    }
    if (millis() - c->opened_ms > HTTP_TIMEOUT_MS) {
        c->pcb = nullptr;
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t http_accept(void*, tcp_pcb* pcb, err_t err) {
    if (err != ERR_OK || !pcb) return ERR_VAL;
    HttpConn* c = nullptr;
    for (HttpConn& slot : http_conns) {
        if (!slot.pcb) {
            c = &slot;
            break;
        }
    }
    if (!c) {
        tcp_abort(pcb);
        return ERR_ABRT;
    }
    c->pcb = pcb;
    c->opened_ms = millis();
    c->req_len = 0;
    c->req[0] = 0;
    c->answered = false;
    c->closing = false;
    tcp_arg(pcb, c);
    tcp_recv(pcb, http_recv);
    tcp_sent(pcb, http_sent);
    tcp_err(pcb, http_err);
    tcp_poll(pcb, http_poll, 2);  // every second
    return ERR_OK;
}

static void mdns_txt(mdns_service*, void*) {}

static void wifi_connect() {
    LOG("Wi-Fi: connecting to \"%s\"", WIFI_SSID);
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK) != 0) {
        LOG("Wi-Fi: connect request failed");
    }
}

static void web_init(const Machine& m) {
    web_machine = &m;
    if (cyw43_arch_init() != 0) {
        LOG("Wi-Fi: chip init failed, web interface off");
        return;
    }
    cyw43_arch_enable_sta_mode();
    netif* nif = &cyw43_state.netif[CYW43_ITF_STA];
    netif_set_hostname(nif, WEB_HOSTNAME);

    mdns_resp_init();
    mdns_resp_add_netif(nif, WEB_HOSTNAME);
    mdns_resp_add_service(nif, "Zanussi Controller", "_http", DNSSD_PROTO_TCP, 80, mdns_txt, nullptr);

    tcp_pcb* listener = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!listener || tcp_bind(listener, IP_ANY_TYPE, 80) != ERR_OK) {
        LOG("Web: can't listen on port 80");
        return;
    }
    listener = tcp_listen_with_backlog(listener, HTTP_MAX_CONNS);
    tcp_accept(listener, http_accept);

    web_up = true;
    wifi_connect();
}

// Logs link changes and reconnects when the link is lost or a connect attempt fails.
static void web_tick(uint32_t now) {
    static int last_status = CYW43_LINK_DOWN;
    static uint32_t last_attempt = now;
    int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (status != last_status) {
        if (status == CYW43_LINK_UP) {
            LOG("Wi-Fi: connected, http://%s/ or http://%s.local/",
                ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA])), WEB_HOSTNAME);
        } else if (status == CYW43_LINK_BADAUTH) {
            LOG("Wi-Fi: wrong password");
        } else if (status == CYW43_LINK_NONET) {
            LOG("Wi-Fi: network \"%s\" not found", WIFI_SSID);
        } else if (status == CYW43_LINK_FAIL || status == CYW43_LINK_DOWN) {
            LOG("Wi-Fi: disconnected");
        }
        last_status = status;
    }
    bool stuck = status == CYW43_LINK_DOWN || status == CYW43_LINK_FAIL ||
                 status == CYW43_LINK_NONET || status == CYW43_LINK_BADAUTH;
    if (stuck && now - last_attempt >= WIFI_RETRY_MS) {
        last_attempt = now;
        wifi_connect();
    }
}


// ---------------------------------------------------------------------------
// MQTT for Home Assistant (needs MQTT_HOST in wifi_config.h). The machine announces itself with
// Home Assistant's MQTT discovery, publishes its state, and takes power and settings commands.
// ---------------------------------------------------------------------------

#if MQTT_ENABLED

#ifndef MQTT_BROKER_PORT  // not MQTT_PORT: lwIP defines that name itself
#define MQTT_BROKER_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER nullptr
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD nullptr
#endif

constexpr const char* MQTT_BASE = "zanussi";  // topics are zanussi/...
constexpr const char* MQTT_AVAILABILITY = "zanussi/availability";
constexpr const char* MQTT_STATE = "zanussi/state";
constexpr const char* MQTT_SETTINGS = "zanussi/settings";
constexpr const char* MQTT_POWER_SET = "zanussi/power/set";
constexpr const char* MQTT_SETTING_SET = "zanussi/settings/";  // + key + "/set"
constexpr const char* MQTT_HA_STATUS = "homeassistant/status";  // Home Assistant's birth message
constexpr uint32_t MQTT_RETRY_MS = 10 * 1000;
constexpr uint32_t MQTT_STATE_MIN_MS = 1000;       // at most this often
constexpr uint32_t MQTT_STATE_REFRESH_MS = 30 * 1000;  // at least this often

enum class MqttPhase { Idle, Resolving, Connecting, Connected };
static mqtt_client_t* mqtt = nullptr;
static MqttPhase mqtt_phase = MqttPhase::Idle;
static uint32_t mqtt_retry_at = 0;
static int mqtt_discovery_next = -1;  // next discovery message to send, -1 = none
static bool mqtt_settings_sent = false;
static Settings mqtt_settings_last;   // what was last published, to spot changes from the web page
static char mqtt_state_last[384];
static uint32_t mqtt_state_ms = 0;     // when the state was last sent, 0 = send now
static uint32_t mqtt_checked_ms = 0;   // when the state was last checked for changes
static int32_t mqtt_rssi = 0;          // refreshed with each full state refresh, not on every check
static ip_addr_t mqtt_ip;

static char mqtt_in_topic[96];
static char mqtt_in_data[64];
static int mqtt_in_len = 0;

static bool mqtt_send(const char* topic, const char* payload, bool retain) {
    err_t err = mqtt_publish(mqtt, topic, payload, (u16_t)strlen(payload), 0, retain, nullptr, nullptr);
    return err == ERR_OK;  // ERR_MEM: output buffer full, try again on the next loop
}

static void mqtt_device(JsonOut& j) {
    j.add("\"avty_t\":\"%s\",\"dev\":{\"ids\":[\"zanussi_controller\"],\"name\":\"Coffee Machine\","
          "\"mf\":\"Zanussi\",\"mdl\":\"ZCOF637X (Pico 2 W controller)\"}}", MQTT_AVAILABILITY);
}

// Discovery message i into topic and j. Returns false once there are no more.
static bool mqtt_discovery(int i, char* topic, size_t topic_cap, JsonOut& j) {
    struct Entity {
        const char* component;
        const char* id;
        const char* name;
        const char* extra;  // more JSON fields, each with a leading comma
    };
    static const Entity ENTITIES[] = {
        {"sensor", "state", "State", ",\"icon\":\"mdi:coffee-maker\""},
        {"sensor", "temp", "Temperature", ",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"stat_cla\":\"measurement\""},
        {"sensor", "target", "Target temperature", ",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\""},
        {"sensor", "power", "Heater power", ",\"unit_of_meas\":\"%\",\"stat_cla\":\"measurement\",\"icon\":\"mdi:fire\""},
        {"sensor", "time_left", "Time left", ",\"dev_cla\":\"duration\",\"unit_of_meas\":\"s\""},
        {"sensor", "fault", "Fault", ",\"icon\":\"mdi:alert\""},
        {"sensor", "rssi", "Wi-Fi signal", ",\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\",\"ent_cat\":\"diagnostic\""},
        {"binary_sensor", "pump", "Pump", ",\"dev_cla\":\"running\""},
        {"binary_sensor", "valve", "Group solenoid", ",\"dev_cla\":\"opening\""},
        {"binary_sensor", "grinder", "Grinder", ",\"dev_cla\":\"running\""},
        {"binary_sensor", "heating", "Heating", ",\"dev_cla\":\"heat\""},
        {"switch", "on", "Power", ",\"cmd_t\":\"zanussi/power/set\",\"icon\":\"mdi:power\""},
    };
    constexpr int N = (int)count_of(ENTITIES);

    if (i < N) {
        const Entity& e = ENTITIES[i];
        snprintf(topic, topic_cap, "homeassistant/%s/zanussi/%s/config", e.component, e.id);
        j.add("{\"name\":\"%s\",\"uniq_id\":\"zanussi_%s\",\"stat_t\":\"%s\",\"val_tpl\":\"{{ value_json.%s }}\"%s,",
              e.name, e.id, MQTT_STATE, e.id, e.extra);
        mqtt_device(j);
        return true;
    }
    i -= N;
    if (i < (int)count_of(SETTING_FIELDS)) {
        const SettingField& f = SETTING_FIELDS[i];
        snprintf(topic, topic_cap, "homeassistant/number/zanussi/%s/config", f.key);
        j.add("{\"name\":\"%s\",\"uniq_id\":\"zanussi_%s\",\"stat_t\":\"%s\",\"val_tpl\":\"{{ value_json.%s }}\","
              "\"cmd_t\":\"%s%s/set\",\"min\":%g,\"max\":%g,\"step\":%g,\"mode\":\"box\",\"ent_cat\":\"config\"",
              f.label, f.key, MQTT_SETTINGS, f.key, MQTT_SETTING_SET, f.key, f.min, f.max, f.step);
        if (f.unit[0]) j.add(",\"unit_of_meas\":\"%s\"", f.unit);
        j.add(",");
        mqtt_device(j);
        return true;
    }
    return false;
}

static void mqtt_state_json(JsonOut& j) {
    const Machine& m = *web_machine;
    const StateInfo& si = info(m.state);
    bool pump = si.pump, valve = si.valve, grinder = si.grinder;
    if (RELAY_TEST_MODE) {
        pump = test_relays[0].on;
        valve = test_relays[1].on;
        grinder = test_relays[2].on;
    }
    auto on_off = [](bool b) { return b ? "ON" : "OFF"; };
    j.add("{\"state\":\"%s\",", RELAY_TEST_MODE ? "RELAY TEST" : si.name);
    if (m.temp_valid) j.add("\"temp\":%.1f,", m.temp_c);
    else j.add("\"temp\":null,");
    j.add("\"target\":%.1f,\"power\":%.0f,\"time_left\":%lu,\"fault\":\"%s\",",
          target_c(si), heater_power * 100.0f, (unsigned long)((time_left_ms(m, millis()) + 999) / 1000),
          m.fault == FaultReason::None ? "none" : fault_text(m.fault));
    j.add("\"pump\":\"%s\",\"valve\":\"%s\",\"grinder\":\"%s\",\"heating\":\"%s\",\"on\":\"%s\",\"rssi\":%ld}",
          on_off(pump), on_off(valve), on_off(grinder), on_off(heater_power > 0.0f),
          on_off(m.state != State::Standby && m.state != State::Fault), (long)mqtt_rssi);
}

static void mqtt_settings_json(JsonOut& j) {
    j.add("{");
    for (size_t i = 0; i < count_of(SETTING_FIELDS); ++i) {
        const SettingField& f = SETTING_FIELDS[i];
        j.add("%s\"%s\":%.6g", i ? "," : "", f.key, field_get(cfg, f));
    }
    j.add("}");
}

static void mqtt_handle_power(const char* payload) {
    const Machine& m = *web_machine;
    if (RELAY_TEST_MODE) {
        LOG("MQTT: power command ignored in relay test mode");
    } else if (strcmp(payload, "ON") == 0) {
        if (m.state == State::Standby) remote_command = RemoteCommand::Start;
        else if (m.state == State::Fault) LOG("MQTT: remote start refused, fault needs a button press on the machine");
    } else if (strcmp(payload, "OFF") == 0) {
        if (m.state != State::Standby && m.state != State::Fault) remote_command = RemoteCommand::Standby;
    } else {
        LOG("MQTT: power command \"%s\" not understood", payload);
    }
    mqtt_state_ms = 0;  // send the state soon, so Home Assistant's switch doesn't bounce
}

static void mqtt_handle_setting(const char* key, const char* payload) {
    for (const SettingField& f : SETTING_FIELDS) {
        if (strcmp(f.key, key) != 0) continue;
        char* end;
        float v = strtof(payload, &end);
        if (end == payload || !field_ok(f, v)) {
            LOG("MQTT: %s = \"%s\" rejected, must be %g to %g", f.key, payload, f.min, f.max);
            mqtt_settings_sent = false;  // republish, so Home Assistant shows the real value again
            return;
        }
        field_set(cfg, f, v);
        settings_save_pending = true;
        LOG("MQTT: %s set to %g", f.key, v);
        return;
    }
    LOG("MQTT: unknown setting \"%s\"", key);
}

static void mqtt_incoming_publish(void*, const char* topic, u32_t) {
    snprintf(mqtt_in_topic, sizeof(mqtt_in_topic), "%s", topic);
    mqtt_in_len = 0;
}

static void mqtt_incoming_data(void*, const u8_t* data, u16_t len, u8_t flags) {
    int room = (int)sizeof(mqtt_in_data) - 1 - mqtt_in_len;
    if (len > room) len = (u16_t)room;
    memcpy(mqtt_in_data + mqtt_in_len, data, len);
    mqtt_in_len += len;
    if (!(flags & MQTT_DATA_FLAG_LAST)) return;
    mqtt_in_data[mqtt_in_len] = 0;

    size_t prefix = strlen(MQTT_SETTING_SET);
    if (strcmp(mqtt_in_topic, MQTT_POWER_SET) == 0) {
        mqtt_handle_power(mqtt_in_data);
    } else if (strncmp(mqtt_in_topic, MQTT_SETTING_SET, prefix) == 0) {
        char key[32];
        snprintf(key, sizeof(key), "%s", mqtt_in_topic + prefix);
        if (char* slash = strchr(key, '/')) *slash = 0;
        mqtt_handle_setting(key, mqtt_in_data);
    } else if (strcmp(mqtt_in_topic, MQTT_HA_STATUS) == 0 && strcmp(mqtt_in_data, "online") == 0) {
        // Home Assistant restarted: announce everything again.
        mqtt_discovery_next = 0;
        mqtt_settings_sent = false;
        mqtt_state_ms = 0;
    }
}

static void mqtt_connected(mqtt_client_t* client, void*, mqtt_connection_status_t status) {
    if (status != MQTT_CONNECT_ACCEPTED) {
        LOG("MQTT: connection to %s failed (%d)%s", MQTT_HOST, (int)status,
            status == MQTT_CONNECT_REFUSED_USERNAME_PASS || status == MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_
                ? ", check MQTT_USER and MQTT_PASSWORD" : "");
        mqtt_phase = MqttPhase::Idle;
        mqtt_retry_at = millis() + MQTT_RETRY_MS;
        return;
    }
    LOG("MQTT: connected to %s", MQTT_HOST);
    mqtt_phase = MqttPhase::Connected;
    mqtt_subscribe(client, MQTT_POWER_SET, 1, nullptr, nullptr);
    mqtt_subscribe(client, "zanussi/settings/+/set", 1, nullptr, nullptr);
    mqtt_subscribe(client, MQTT_HA_STATUS, 1, nullptr, nullptr);
    mqtt_publish(client, MQTT_AVAILABILITY, "online", 6, 1, 1, nullptr, nullptr);
    mqtt_discovery_next = 0;
    mqtt_settings_sent = false;
    mqtt_state_ms = 0;
}

static void mqtt_connect_to(const ip_addr_t* ip) {
    static mqtt_connect_client_info_t ci = {};
    ci.client_id = MQTT_BASE;
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASSWORD;
    ci.keep_alive = 30;
    ci.will_topic = MQTT_AVAILABILITY;  // the broker publishes "offline" if the machine drops off
    ci.will_msg = "offline";
    ci.will_qos = 1;
    ci.will_retain = 1;
    mqtt_ip = *ip;
    mqtt_phase = MqttPhase::Connecting;
    if (mqtt_client_connect(mqtt, &mqtt_ip, MQTT_BROKER_PORT, mqtt_connected, nullptr, &ci) != ERR_OK) {
        LOG("MQTT: can't start connection to %s", MQTT_HOST);
        mqtt_phase = MqttPhase::Idle;
        mqtt_retry_at = millis() + MQTT_RETRY_MS;
    }
}

static void mqtt_resolved(const char*, const ip_addr_t* ip, void*) {
    if (ip) {
        mqtt_connect_to(ip);
    } else {
        LOG("MQTT: can't find broker \"%s\"", MQTT_HOST);
        mqtt_phase = MqttPhase::Idle;
        mqtt_retry_at = millis() + MQTT_RETRY_MS;
    }
}

// Called on every main loop pass while Wi-Fi is up: connects, sends one queued message at a
// time (the output buffer holds only a few), and publishes the state when it changes.
static void mqtt_poll(uint32_t now) {
    if (!mqtt) {
        mqtt = mqtt_client_new();
        if (!mqtt) return;
        mqtt_set_inpub_callback(mqtt, mqtt_incoming_publish, mqtt_incoming_data, nullptr);
    }
    bool link_up = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;

    if (mqtt_phase == MqttPhase::Connected && !mqtt_client_is_connected(mqtt)) {
        LOG("MQTT: disconnected");
        mqtt_phase = MqttPhase::Idle;
        mqtt_retry_at = now + MQTT_RETRY_MS;
    }
    if (mqtt_phase == MqttPhase::Idle) {
        if (!link_up || (int32_t)(now - mqtt_retry_at) < 0) return;
        mqtt_phase = MqttPhase::Resolving;
        ip_addr_t ip;
        err_t err = dns_gethostbyname(MQTT_HOST, &ip, mqtt_resolved, nullptr);
        if (err == ERR_OK) mqtt_connect_to(&ip);
        else if (err != ERR_INPROGRESS) mqtt_resolved(MQTT_HOST, nullptr, nullptr);
        return;
    }
    if (mqtt_phase != MqttPhase::Connected) return;

    static char topic[96];
    static char payload[640];
    JsonOut j{payload, 0, sizeof(payload)};

    if (mqtt_discovery_next >= 0) {
        if (!mqtt_discovery(mqtt_discovery_next, topic, sizeof(topic), j)) {
            mqtt_discovery_next = -1;
        } else if (j.ok && mqtt_send(topic, payload, true)) {
            ++mqtt_discovery_next;
        }
        return;
    }

    if (!mqtt_settings_sent || memcmp(&mqtt_settings_last, &cfg, sizeof(cfg)) != 0) {
        mqtt_settings_json(j);
        if (j.ok && mqtt_send(MQTT_SETTINGS, payload, true)) {
            mqtt_settings_last = cfg;
            mqtt_settings_sent = true;
        }
        return;
    }

    if (mqtt_state_ms != 0 && now - mqtt_checked_ms < MQTT_STATE_MIN_MS) return;
    mqtt_checked_ms = now;
    bool refresh = mqtt_state_ms == 0 || now - mqtt_state_ms >= MQTT_STATE_REFRESH_MS;
    if (refresh) cyw43_wifi_get_rssi(&cyw43_state, &mqtt_rssi);
    mqtt_state_json(j);
    if (!j.ok || (!refresh && strcmp(payload, mqtt_state_last) == 0)) return;
    if (mqtt_send(MQTT_STATE, payload, true)) {
        snprintf(mqtt_state_last, sizeof(mqtt_state_last), "%s", payload);
        mqtt_state_ms = now ? now : 1;
    } else {
        mqtt_checked_ms = now - MQTT_STATE_MIN_MS;  // buffer full: try again on the next loop
    }
}

#endif  // MQTT_ENABLED

#endif  // WEB_ENABLED

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
    relays_init();
    stdio_init_all();
    sleep_ms(1000);  // let a USB serial host attach; UART output is unaffected

    printf("\r\n\r\n=== Zanussi ZCOF637X controller ===\r\n");
    LOG("stdio up (UART0 GP0/GP1 @ 115200 and USB)");
#if PICO_RP2350
    // A2 has erratum E9: inputs held low only by the internal pull-down can read high.
    LOG("RP2350 stepping %s", rp2350_chip_version() == 2 ? "A2 (erratum E9 present)" : "A3/A4 (E9 fixed)");
#endif

    settings_load();
    buttons_init();
    thermocouple_init();

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
        uint8_t dummy;
        if (i2c_read_timeout_us(I2C_PORT, addr, &dummy, 1, false, 2000) >= 0) {
            LOG("I2C device found at 0x%02X", addr);
            ++found;
        }
    }
    if (!found) LOG("I2C scan: no devices on SDA=GP%d SCL=GP%d", I2C_SDA, I2C_SCL);

    oled_ok = oled_init();
    LOG("OLED at 0x%02X %s", OLED_ADDRESS, oled_ok ? "OK" : "not responding");

    leds_ok = leds_init();
    LOG("Lamp ring and status LEDs on GP%d %s", LED_PIN, leds_ok ? "OK" : "failed: no free PIO state machine");

    static repeating_timer_t heater_timer;
    // Negative period: a fixed interval between starts, so the cycles don't drift.
    add_repeating_timer_ms(-(int32_t)HEATER_STEP_MS, heater_step, nullptr, &heater_timer);

    Machine m;
    uint32_t now = millis();
    m.entered_ms = m.activity_ms = now;
    uint32_t next_sample = now, next_display = now, next_status = now, next_oled_retry = now + 5000;
    uint32_t next_leds = now, next_history = now, next_web = now;
    int oled_page = 8;  // next OLED page to send; 8 = the screen is up to date
    uint32_t led_test_start = now;
    bool led_test = leds_ok && !RELAY_TEST_MODE;
    if (led_test) LOG("LED test pattern running, press any button to stop");

    if (RELAY_TEST_MODE) LOG("RELAY TEST MODE: switch the pump, solenoid and grinder from the web page");
    else LOG("State: %s", info(m.state).name);

#if WEB_ENABLED
    web_init(m);
#else
    LOG("Web interface off: no wifi_config.h");
#endif

    while (true) {
        now = millis();

        ButtonEvents b = buttons_poll(now);
        if (led_test) {
            for (bool pressed : b.pressed) {
                if (pressed) led_test = false;
            }
            if (!led_test) {
                b = {};  // the press that ends the test doesn't also wake the machine
                LOG("LED test stopped");
            }
        }

        bool sampled = false;
        if ((int32_t)(now - next_sample) >= 0) {
            next_sample += SENSOR_PERIOD_MS;
            sampled = true;
            float t;
            if (thermocouple_read(&t)) {
                m.temp_c = t;
                m.temp_valid = true;
                m.sensor_errors = 0;
            } else {
                m.temp_valid = false;
                if (m.sensor_errors < SENSOR_FAULT_COUNT) {
                    ++m.sensor_errors;
                    LOG("Thermocouple read failed (%d/%d)", m.sensor_errors, SENSOR_FAULT_COUNT);
                }
            }
        }

        if (RELAY_TEST_MODE) {
            // No state machine: buttons do nothing, and the heater stays off because
            // update_heater() never runs (the heater timer treats the power as stale).
            relay_test_tick(now);
        } else {
            State before = m.state;
            run_state_machine(m, b, now);
            if (sampled || m.state != before) update_heater(m, sampled);
            drive_relays(m);
        }

        if ((int32_t)(now - next_status) >= 0 && RELAY_TEST_MODE) {
            next_status += STATUS_LOG_MS;
            LOG("RELAY TEST temp=%s%.2f C pump=%d valve=%d grinder=%d", m.temp_valid ? "" : "INVALID ",
                m.temp_c, test_relays[0].on, test_relays[1].on, test_relays[2].on);
        } else if ((int32_t)(now - next_status) >= 0) {
            next_status += STATUS_LOG_MS;
            const StateInfo& si = info(m.state);
            LOG("%-10s temp=%s%.2f C set=%.0f power=%.0f%% (P=%.2f I=%.2f rate=%.2f C/s) pump=%d valve=%d grinder=%d oled=%d",
                si.name, m.temp_valid ? "" : "INVALID ", m.temp_c, target_c(si), heater_power * 100.0f,
                target_c(si) > 0 ? cfg.kp * (target_c(si) - m.temp_c) : 0.0f, pid.integral, pid.rate_c_per_s,
                si.pump, si.valve, si.grinder, oled_ok);
        }

        if ((int32_t)(now - next_display) >= 0) {
            next_display += DISPLAY_PERIOD_MS;
            if (!oled_ok && (int32_t)(now - next_oled_retry) >= 0) {
                next_oled_retry = now + 5000;
                oled_ok = oled_init();
                if (oled_ok) LOG("OLED reconnected");
            }
            if (oled_ok && oled_page == 8) {
                if (RELAY_TEST_MODE) relay_test_render(now);
                else render(m, now);
                oled_page = 0;
            }
        }
        if (oled_ok && oled_page < 8 && !oled_flush_page(oled_page++)) {
            oled_ok = false;
            oled_page = 8;
            LOG("OLED write failed");
        }

        if (leds_ok && (int32_t)(now - next_leds) >= 0) {
            next_leds += LED_PERIOD_MS;
            if (RELAY_TEST_MODE) relay_test_leds();
            else if (led_test) leds_test_render(now - led_test_start);
            else leds_render(m, now);
        }

        if ((int32_t)(now - next_history) >= 0) {
            next_history += HISTORY_PERIOD_MS;
            history_add(m);
        }

#if WEB_ENABLED
        if (web_up) {
            cyw43_arch_poll();
            if ((int32_t)(now - next_web) >= 0) {
                next_web += 1000;
                web_tick(now);
            }
#if MQTT_ENABLED
            mqtt_poll(now);
#endif
            if (settings_save_pending) {
                settings_save_pending = false;
                settings_save();
            }
        }
#else
        (void)next_web;
#endif

        sleep_ms(2);
    }
}
