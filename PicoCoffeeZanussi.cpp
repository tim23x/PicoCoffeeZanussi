// Controller for a Zanussi ZCOF637X espresso machine.
//
// Hardware: MAX6675 thermocouple (SPI0), SH1106 128x64 OLED (I2C0), active-high
// relays for heater / pump / group solenoid / grinder, four active-high buttons.
// Logging goes to stdio: UART0 (GP0 TX, GP1 RX, 115200) for the Debug Probe, and USB.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

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
constexpr uint PUMP_PIN = 3;
constexpr uint SOLENOID_PIN = 4;
constexpr uint GRINDER_PIN = 5;
constexpr uint GRIND_BUTTON_PIN = 18;
constexpr uint COFFEE_BUTTON_PIN = 19;
constexpr uint WATER_BUTTON_PIN = 20;
constexpr uint STEAM_BUTTON_PIN = 21;

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

constexpr float BREW_TEMP_C = 95.0f;
constexpr float STEAM_TEMP_C = 140.0f;
constexpr float HYSTERESIS_C = 1.0f;     // heater on below target - this, off at target
constexpr float READY_BAND_C = 2.0f;     // this close to target counts as "at temperature"
constexpr float REHEAT_BAND_C = 8.0f;    // Ready falls back to Heating this far below brew temp
constexpr float COOLED_BAND_C = 3.0f;    // after steam, brewing allowed again within this of brew temp
constexpr float OVER_TEMP_C = 165.0f;    // hard cutoff

constexpr uint32_t GRIND_MS = 12 * 1000;
constexpr uint32_t PREINFUSE_MS = 3 * 1000;
constexpr uint32_t SOAK_MS = 2 * 1000;
constexpr uint32_t BREW_MS = 25 * 1000;
constexpr uint32_t HOT_WATER_MS = 30 * 1000;
constexpr uint32_t HEAT_TIMEOUT_MS = 5 * 60 * 1000;
constexpr uint32_t STEAM_IDLE_MS = 3 * 60 * 1000;
constexpr uint32_t IDLE_MS = 30 * 60 * 1000;

constexpr uint32_t DEBOUNCE_MS = 30;
constexpr uint32_t LONG_PRESS_MS = 2000;  // hold COFFEE to switch off
constexpr uint32_t SENSOR_PERIOD_MS = 250; // MAX6675 needs >= 220 ms per conversion
constexpr int SENSOR_FAULT_COUNT = 3;
constexpr uint32_t DISPLAY_PERIOD_MS = 250;
constexpr uint32_t STATUS_LOG_MS = 2000;

static uint32_t millis() { return to_ms_since_boot(get_absolute_time()); }

#define LOG(fmt, ...) printf("[%7.3f] " fmt "\r\n", millis() / 1000.0f, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// Buttons: debounced press edge plus a one-shot long-press event
// ---------------------------------------------------------------------------

enum ButtonId { BTN_GRIND, BTN_COFFEE, BTN_WATER, BTN_STEAM, BTN_COUNT };

struct Button {
    const char* name;
    uint pin;
    bool raw;
    uint32_t raw_changed;
    bool down;
    uint32_t down_since;
    bool long_sent;
};

static Button buttons[BTN_COUNT] = {
    {"GRIND", GRIND_BUTTON_PIN},
    {"COFFEE", COFFEE_BUTTON_PIN},
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

// Page addressing: the one mode both SH1106 and SSD1306 support.
static bool oled_flush() {
    uint8_t pkt[129];
    pkt[0] = 0x40;
    for (int page = 0; page < 8; ++page) {
        const uint8_t addr[] = {
            (uint8_t)(0xB0 | page),                       // page
            (uint8_t)(0x00 | (OLED_COLUMN_OFFSET & 0x0F)), // column low nibble
            (uint8_t)(0x10 | (OLED_COLUMN_OFFSET >> 4)),   // column high nibble
        };
        if (!oled_cmds(addr, sizeof(addr))) return false;
        memcpy(pkt + 1, oled_buf[page], 128);
        if (i2c_write_timeout_us(I2C_PORT, OLED_ADDRESS, pkt, sizeof(pkt), false, 10000) != (int)sizeof(pkt)) {
            return false;
        }
    }
    return true;
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
};

enum class FaultReason { None, Sensor, OverTemp, HeatTimeout };

// Everything a state drives, plus its timer and idle limit (0 = none).
struct StateInfo {
    const char* name;     // max 10 chars (shown double size)
    float target_c;       // heater setpoint, 0 = heater off
    bool pump, valve, grinder;
    uint32_t timer_ms;    // raises TimerExpired
    uint32_t idle_ms;     // raises Idle with no button activity
    const char* hint;
};

static const StateInfo STATES[] = {
    //  name          target         pump   valve  grind  timer            idle           hint
    {"STANDBY",    0,            false, false, false, 0,               0,             "PRESS ANY BUTTON"},
    {"HEATING",    BREW_TEMP_C,  false, false, false, HEAT_TIMEOUT_MS, IDLE_MS,       "PLEASE WAIT"},
    {"READY",      BREW_TEMP_C,  false, false, false, 0,               IDLE_MS,       "HOLD COFFEE: OFF"},
    {"GRINDING",   BREW_TEMP_C,  false, false, true,  GRIND_MS,        0,             "GRIND TO STOP"},
    {"PREINFUSE",  BREW_TEMP_C,  true,  true,  false, PREINFUSE_MS,    0,             "COFFEE TO STOP"},
    {"SOAK",       BREW_TEMP_C,  false, true,  false, SOAK_MS,         0,             "COFFEE TO STOP"},
    {"BREWING",    BREW_TEMP_C,  true,  true,  false, BREW_MS,         0,             "COFFEE TO STOP"},
    {"HOT WATER",  BREW_TEMP_C,  true,  false, false, HOT_WATER_MS,    0,             "WATER TO STOP"},
    {"STEAM HEAT", STEAM_TEMP_C, false, false, false, HEAT_TIMEOUT_MS, STEAM_IDLE_MS, "STEAM TO CANCEL"},
    {"STEAM RDY",  STEAM_TEMP_C, false, false, false, 0,               STEAM_IDLE_MS, "OPEN STEAM KNOB"},
    {"COOL DOWN",  BREW_TEMP_C,  false, false, false, 0,               0,             "WATER TO FLUSH"},
    {"FAULT",      0,            false, false, false, 0,               0,             "PRESS TO RESET"},
};
static_assert(sizeof(STATES) / sizeof(STATES[0]) == (size_t)State::COUNT, "STATES table out of sync");

static const StateInfo& info(State s) { return STATES[(int)s]; }

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
    bool heater = false;
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
    if (e == Event::CoffeeHeld) return State::Standby;

    switch (s) {
        case State::Standby:
            if (e == Event::GrindPressed || e == Event::CoffeePressed ||
                e == Event::WaterPressed || e == Event::SteamPressed) return State::Heating;
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
    if (si.timer_ms && in_state >= si.timer_ms)      out[n++] = Event::TimerExpired;
    if (m.temp_valid) {
        if (si.target_c > 0 && m.temp_c >= si.target_c - READY_BAND_C) out[n++] = Event::AtTemp;
        if (m.temp_c < BREW_TEMP_C - REHEAT_BAND_C)                    out[n++] = Event::TooCold;
        if (m.temp_c <= BREW_TEMP_C + COOLED_BAND_C)                   out[n++] = Event::CooledDown;
    }
    if (si.idle_ms && now - m.activity_ms >= si.idle_ms) out[n++] = Event::Idle;
    return n;
}

// One transition per tick at most, so later events are judged against the new state next tick.
static void run_state_machine(Machine& m, const ButtonEvents& b, uint32_t now) {
    for (int i = 0; i < BTN_COUNT; ++i) {
        if (b.pressed[i]) m.activity_ms = now;
    }
    Event events[12];
    int n = collect_events(m, b, now, events);
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
    const uint pins[] = {HEATER_PIN, PUMP_PIN, SOLENOID_PIN, GRINDER_PIN};
    for (uint pin : pins) {
        gpio_init(pin);
        gpio_put(pin, 0);  // off before the pin becomes an output
        gpio_set_dir(pin, GPIO_OUT);
    }
}

// Bang-bang with hysteresis. Fails safe to off whenever the reading is invalid.
static void update_heater(Machine& m) {
    float target = info(m.state).target_c;
    if (target <= 0 || !m.temp_valid) m.heater = false;
    else if (m.temp_c >= target) m.heater = false;
    else if (m.temp_c <= target - HYSTERESIS_C) m.heater = true;
}

static void drive_relays(const Machine& m) {
    const StateInfo& si = info(m.state);
    gpio_put(HEATER_PIN, m.heater);
    gpio_put(PUMP_PIN, si.pump);
    gpio_put(SOLENOID_PIN, si.valve);
    gpio_put(GRINDER_PIN, si.grinder);
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

static void render(const Machine& m, uint32_t now) {
    const StateInfo& si = info(m.state);
    char line[22];
    memset(oled_buf, 0, sizeof(oled_buf));

    draw_big(0, 0, si.name);

    if (m.temp_valid) snprintf(line, sizeof(line), "TEMP %5.1f C", m.temp_c);
    else snprintf(line, sizeof(line), "TEMP ---");
    draw_text(0, 3, line);

    if (si.target_c > 0) {
        snprintf(line, sizeof(line), "SET  %3d C  %s", (int)si.target_c, m.heater ? "HEAT" : "");
        draw_text(0, 4, line);
    }

    if (m.state == State::Fault) {
        draw_text(0, 5, fault_text(m.fault));
    } else if (si.timer_ms && si.timer_ms < HEAT_TIMEOUT_MS) {
        uint32_t left = si.timer_ms - (now - m.entered_ms);
        if (left > si.timer_ms) left = 0;
        snprintf(line, sizeof(line), "TIME %2lu S", (unsigned long)((left + 999) / 1000));
        draw_text(0, 5, line);
    }

    draw_text(0, 7, si.hint);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {
    relays_init();
    stdio_init_all();
    sleep_ms(1000);  // let a USB serial host attach; UART output is unaffected

    printf("\r\n\r\n=== Zanussi ZCOF637X controller ===\r\n");
    LOG("stdio up (UART0 GP0/GP1 @ 115200 and USB)");

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

    Machine m;
    uint32_t now = millis();
    m.entered_ms = m.activity_ms = now;
    uint32_t next_sample = now, next_display = now, next_status = now, next_oled_retry = now + 5000;

    LOG("State: %s", info(m.state).name);

    while (true) {
        now = millis();

        ButtonEvents b = buttons_poll(now);

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

        State before = m.state;
        run_state_machine(m, b, now);
        if (sampled || m.state != before) update_heater(m);
        drive_relays(m);

        if ((int32_t)(now - next_status) >= 0) {
            next_status += STATUS_LOG_MS;
            const StateInfo& si = info(m.state);
            LOG("%-10s temp=%s%.2f C set=%.0f heater=%d pump=%d valve=%d grinder=%d oled=%d",
                si.name, m.temp_valid ? "" : "INVALID ", m.temp_c, si.target_c,
                m.heater, si.pump, si.valve, si.grinder, oled_ok);
        }

        if ((int32_t)(now - next_display) >= 0) {
            next_display += DISPLAY_PERIOD_MS;
            if (!oled_ok && (int32_t)(now - next_oled_retry) >= 0) {
                next_oled_retry = now + 5000;
                oled_ok = oled_init();
                if (oled_ok) LOG("OLED reconnected");
            }
            if (oled_ok) {
                render(m, now);
                if (!oled_flush()) {
                    oled_ok = false;
                    LOG("OLED write failed");
                }
            }
        }

        sleep_ms(2);
    }
}
