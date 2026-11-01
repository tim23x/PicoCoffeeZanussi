# PicoCoffeeZanussi

Replacement controller for a **Zanussi ZCOF637X** espresso machine, built on a Raspberry Pi **Pico 2 W**. The firmware is a single C++ file running a state machine. It handles heating, grinding, a three-stage brew (pre-infuse, soak, extract), hot water and steam, and it has safety cut-outs.

> [!WARNING]
> This project switches a mains-powered heater and pump. Build it only if you are competent with mains wiring. Use relays rated for the load, keep low-voltage and mains wiring separated, and keep the machine's original thermal fuses and thermostats in circuit. The firmware's safety checks are an extra layer, not a replacement for them.

## Features

- Brew sequence: pre-infuse, soak, extract, with a countdown on the display.
- Timed grinding for one dose.
- Hot water, and steam with its own higher temperature setting.
- Cool-down after steaming, so you can't brew with an overheated block.
- PID heater control with delta-sigma output through a solid-state relay, driven by a K-type thermocouple.
- Safety faults that turn everything off and stay off until reset: thermocouple failure, over-temperature, and heating that takes too long.
- Automatic standby when the machine is left idle.
- Status shown on a strip of 7 WS2812 (NeoPixel) LEDs, a 128x64 OLED, or both.
- A 16-LED NeoPixel ring in place of the original halogen lamp.
- Serial log of every button press, state change and fault.

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 W (RP2350) | The original Pico (RP2040) also works if you change `PICO_BOARD` in `CMakeLists.txt` |
| MAX6675 module + K-type thermocouple | Mounted on the boiler/thermoblock |
| 16-LED WS2812B ring | Replaces the halogen lamp. First in the LED chain |
| 7 x WS2812B LEDs (NeoPixel strip or individual pixels) | Status display, chained after the ring. Optional if you fit the OLED |
| 1.3" SH1106 128x64 I2C OLED | Optional if you fit the LEDs. A 0.96" SSD1306 also works, see [Configuration](#configuration) |
| SSR-25 DA solid-state relay, on a heatsink | Heater. Must be a solid-state relay, see [Heater control](#heater-control) |
| Relay module, active-high | Grinder, group solenoid, pump. Must be rated for mains loads |
| Carrier PCB with 5 V rail + 5 V 5 A DC supply | Powers the Pico, relay coils and LEDs |
| 4 momentary push buttons | Grind, Coffee, Water, Steam |
| Raspberry Pi Debug Probe (optional) | For flashing, debugging and the serial log |

## Wiring

All signals are 3.3 V. Physical pin numbers are for the Pico 2 W.

| Function | GPIO | Pin | Connects to |
|---|---|---|---|
| Heater SSR | GP2 | 4 | Darlington array input. The array output sinks the SSR's input −, and the SSR's input + goes to 5 V |
| Grinder relay | GP3 | 5 | Relay IN2 |
| Group solenoid relay | GP4 | 6 | Relay IN3 |
| Pump relay | GP5 | 7 | Relay IN4 |
| Spare relay 1 | GP6 | 9 | Spare relay IN (held off) |
| Spare relay 2 | GP7 | 10 | Spare relay IN (held off) |
| LED data | GP10 | 14 | Ring DIN, through a 330 Ω resistor. Ring DOUT goes to the first status LED's DIN |
| OLED SDA | GP8 | 11 | OLED SDA |
| OLED SCL | GP9 | 12 | OLED SCL |
| Thermocouple data | GP16 (SPI0 RX) | 21 | MAX6675 SO |
| Thermocouple chip select | GP17 | 22 | MAX6675 CS |
| Thermocouple clock | GP22 (SPI0 SCK) | 29 | MAX6675 SCK |
| Grind button | GP18 | 24 | Button to 3V3 |
| Coffee button | GP19 | 25 | Button to 3V3 |
| Water button | GP20 | 26 | Button to 3V3 |
| Steam button | GP21 | 27 | Button to 3V3 |
| Serial log TX | GP0 | 1 | Debug Probe UART RX |
| Serial log RX | GP1 | 2 | Debug Probe UART TX |

Power: the Pico sits on a carrier PCB with a 5 V rail, fed by a 5 V 5 A DC supply. The rail powers the Pico, the relay coils and the LEDs.

- **Pico:** 5 V rail to **VBUS (pin 40)**. The Pico's USB connector isn't used. For debugging, the board's VSYS (pin 39) connects to the debug board's VSYS, so the debug board can power the Pico when the supply is off. Both Picos have a diode from VBUS to VSYS, so with the supply and the debug board both connected, neither can back-feed the other's USB or the 5 V rail.
- **OLED and MAX6675:** VCC to **3V3(OUT), pin 36**, GND to any GND pin. Don't power the OLED from 5 V: its on-board pull-ups would put 5 V on GP8/GP9.
- **Relay module:** the coils run from the 5 V rail, not the Pico's 3.3 V regulator. Only the IN signals connect to the GPIOs.
- **Ring and status LEDs:** 5 V and GND from the rail, not through the Pico. They draw about 0.4 A at the default brightness and up to about 1.4 A at full white, well within the 5 A supply. A 100-1000 µF capacitor across the LEDs' 5 V and GND is good practice. Most WS2812Bs accept the Pico's 3.3 V data signal on a short wire. If yours flicker or show the wrong colours, add a 74AHCT125 level shifter.
- **Buttons:** each one connects its GPIO to 3V3. The firmware turns on the internal pull-downs, so you don't need resistors.

## Using the machine

| Button | Short press | Hold 2 s |
|---|---|---|
| **Grind** | Grind one dose (press again to stop early) | |
| **Coffee** | Start a shot (press again to stop) | Switch to standby from any state |
| **Water** | Hot water through the wand (press again to stop) | |
| **Steam** | Heat for steam; press again to leave steam mode | |

Any button wakes the machine from standby. It heats to brew temperature and shows **READY** when it's there. For steam, wait for **STEAM RDY**, then open the steam knob. When you leave steam mode the machine shows **COOL DOWN** until it's back to brew temperature. Pressing Water during cool-down flushes water through the wand to speed this up.

The display shows the state, the current and target temperature, a countdown for timed steps, and a hint for what to press next.

### Relay test

While `RELAY_TEST_MODE` is `true` at the top of `PicoCoffeeZanussi.cpp`, the firmware runs a relay test instead of the normal state machine. Each relay is on only while you hold its button:

| Hold | Relay |
|---|---|
| **Steam** | Group solenoid |
| **Water** | Pump |

The heater and grinder stay off. The OLED and status LEDs show which relays are on: LED 1 is cyan for the pump and LED 2 is white for the solenoid. The other status LEDs glow dim magenta to show you're in test mode. The relay coils run from the 5 V rail, so the PSU must be on.

Before running the pump, fill the water tank and put a cup under the group head. With the solenoid closed the pump feeds the steam wand, so open the steam knob and hold a jug under it too.

Set `RELAY_TEST_MODE` to `false` and rebuild to use the machine normally.

### Status LEDs

The colour shows the state. On a progress bar, the number of lit LEDs shows how far along it is, and the leading LED pulses.

| State | Colour | Pattern |
|---|---|---|
| Standby | Dim white | Whole strip breathes slowly |
| Heating | Orange | Bar fills as the temperature rises from 20 °C to brew temperature |
| Ready | Green | All on |
| Grinding | Yellow | Bar empties as the grind time runs out |
| Pre-infuse, Soak, Brewing | White | Bar fills across the whole shot |
| Hot Water | Cyan | Bar empties as the run time runs out |
| Steam Heat | Blue | Bar fills as the temperature rises from brew to steam temperature |
| Steam Ready | Blue | All on |
| Cool Down | Magenta | Bar fills as the block cools back to brew temperature |
| Fault | Red | Flashing. 1 LED: thermocouple, 2 LEDs: over-temperature, 3 LEDs: heat timeout |

Bars fill from the status LED nearest the ring.

The ring lights warm white whenever the machine is on, and goes off in standby.

At start-up all the LEDs run a test pattern until you press any button. That press only stops the test, it doesn't wake the machine. The test lights every LED red, green, blue, then white for 1 s each, which checks every colour channel. Then a single white LED walks from the first ring LED to the last status LED, which checks the wiring order and the LED count. The test runs at low brightness (`LED_TEST_BRIGHTNESS`), so it draws only about 90 mA and can run from the Pico's USB alone.

## State machine

```mermaid
flowchart TD
    Standby(["Standby"]) -->|"any button"| Heating
    Heating -->|"at brew temp"| Ready
    Heating -->|"Grind"| Grinding
    Heating -->|"Steam"| SteamHeat["Steam Heat"]
    Ready -->|"Coffee"| Preinfuse
    Preinfuse -->|"3 s"| Soak
    Soak -->|"2 s"| Brewing
    Brewing -->|"25 s / Coffee"| Ready
    Ready -->|"Grind"| Grinding
    Grinding -->|"12 s / Grind"| Heating
    Ready -->|"Water"| HotWater["Hot Water"]
    HotWater -->|"30 s / Water"| CoolDown["Cool Down"]
    Ready -->|"Steam"| SteamHeat
    SteamHeat -->|"at steam temp"| SteamReady["Steam Ready"]
    SteamHeat -->|"Steam / idle"| CoolDown
    SteamReady -->|"Steam / 3 min idle"| CoolDown
    CoolDown -->|"back at brew temp"| Ready
    CoolDown -->|"Water (flush)"| HotWater
    Ready -->|"temp dropped 8 C"| Heating
    Ready -->|"30 min idle"| Standby
    Fault{{"Fault"}} -->|"button, once cleared"| Standby
```

These transitions are left out of the diagram:

- **Fault:** any state goes to Fault on a thermocouple failure (3 bad reads in a row) or at 165 °C. Heating and Steam Heat also go to Fault if they don't reach temperature within 5 minutes.
- **Standby:** holding Coffee for 2 s goes to Standby from any state except Fault.

### How the code is organised

Everything is in [`PicoCoffeeZanussi.cpp`](PicoCoffeeZanussi.cpp):

- **`STATES` table:** one row per state. It sets the heater target, which relays are on, the state's timer and idle timeout, and the hint shown on the display. To change what a state does, edit its row.
- **`collect_events()`:** turns button presses, timers, temperature readings and fault conditions into events, with safety events first.
- **`next_state()`:** the transition table, and the only place where states change. The machine makes at most one transition per loop.
- **`LED_STYLES` table:** the colour and pattern for each state on the status LEDs.
- **`update_heater()`:** the PID controller. It sets the heater power toward the current state's target, and sets zero power whenever the temperature reading is invalid.
- **`heater_step()`:** the delta-sigma output, run from a timer interrupt once per mains cycle.

## Heater control

The heater uses two layers of control:

1. **PID:** on every thermocouple reading (every 250 ms), `update_heater()` sets a heater power from 0 to 100% toward the current state's target. The D term works on the temperature, not the error, so changing target (brew to steam) doesn't make the power jump. The I term stops growing while the output is already at 0% or 100%.
2. **Delta-sigma:** once per mains cycle, a timer interrupt adds the power to an accumulator and switches the heater on for that cycle whenever the accumulator reaches a whole cycle. For 30% power, 3 cycles in every 10 are on, spread as evenly as possible. The interrupt runs on time even while the main loop is busy, and it switches the heater off if the main loop hasn't updated the power for 1 s.

> [!IMPORTANT]
> The heater must be on a solid-state relay. Delta-sigma switching would wear out a mechanical relay within days.

The SSR is switched from 5 V through a Darlington array (ULN2003 or similar), so the Pico's pin only drives the array's input. The array turns on when GP2 is high, so the firmware's active-high output works without change. The array drops about 1 V, which leaves the SSR about 4 V, within its 3-32 V input range.

### Tuning

The default gains are starting points. Tune them with the machine cold, filled with water, and the serial log open. Each status line shows the power, the P and I terms, and the rate of temperature change.

1. Set `HEATER_KI` and `HEATER_KD` to 0. Raise `HEATER_KP` until the temperature settles near the target without oscillating. It will settle a little below the target.
2. Raise `HEATER_KD` to cut the overshoot when heating from cold. Thermoblocks overshoot because the thermocouple lags the heater.
3. Raise `HEATER_KI` until the temperature settles on the target within a minute or two. Too much makes it oscillate slowly.

## Building and flashing

This project uses the [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) with Pico SDK 2.3.1.

1. Open the folder in VS Code. The extension installs the SDK and toolchain.
2. Click **Compile** in the status bar, or run `ninja` in `build/`.
3. Flash by one of these methods:
   - **Debug Probe:** use **Run → Start Debugging**, or the extension's **Run** button.
   - **USB:** with the 5 V supply off, hold BOOTSEL while plugging in the Pico, then copy `build/PicoCoffeeZanussi.uf2` to the drive that appears.

## Serial log

Logging goes to both UART0 (GP0/GP1, 115200 baud) and USB. With a Debug Probe, open its COM port. With the Pico plugged into USB, open the Pico's own COM port.

```
=== Zanussi ZCOF637X controller ===
[  1.002] I2C device found at 0x3C
[  1.004] OLED at 0x3C OK
[  1.004] State: STANDBY
[  4.512] Button COFFEE pressed
[  4.512] STANDBY -> HEATING (COFFEE pressed, 21.50 C)
[  5.004] HEATING    temp=22.25 C set=95 power=100% (P=5.82 I=0.00 rate=0.00 C/s) pump=0 valve=0 grinder=0 oled=1
```

## Configuration

All settings are constants at the top of `PicoCoffeeZanussi.cpp`.

| Constant | Default | Purpose |
|---|---|---|
| `RELAY_TEST_MODE` | `true` | Temporary relay test instead of normal operation, see [Relay test](#relay-test) |
| `BREW_TEMP_C` | 95 °C | Brew temperature |
| `STEAM_TEMP_C` | 140 °C | Steam temperature |
| `OVER_TEMP_C` | 165 °C | Safety cut-out |
| `GRIND_MS` | 12 s | Grind time for one dose |
| `PREINFUSE_MS` / `SOAK_MS` / `BREW_MS` | 3 s / 2 s / 25 s | Brew stages |
| `HOT_WATER_MS` | 30 s | Maximum hot water run |
| `HEATER_KP` / `HEATER_KI` / `HEATER_KD` | 0.08 / 0.002 / 0.2 | PID gains, see [Heater control](#heater-control) |
| `HEATER_STEP_MS` | 20 ms | One mains cycle: 20 for 50 Hz, 17 for 60 Hz |
| `HEAT_TIMEOUT_MS` | 5 min | Fault if not at temperature by then |
| `IDLE_MS` / `STEAM_IDLE_MS` | 30 min / 3 min | Auto standby / leave steam mode |
| `RING_COUNT` | 16 | Number of LEDs in the lamp ring |
| `RING_R` / `RING_G` / `RING_B` | 255 / 160 / 60 | Lamp colour (warm white) |
| `RING_BRIGHTNESS` | 128 | Lamp brightness, 0-255 |
| `LED_COUNT` | 7 | Number of status LEDs |
| `LED_BRIGHTNESS` | 64 | Status LED brightness, 0-255 |
| `LED_TEST_BRIGHTNESS` | 16 | Brightness of the start-up LED test, 0-255 |
| `OLED_COLUMN_OFFSET` | 2 | 2 for an SH1106 (1.3"), 0 for an SSD1306 (0.96") |
| `OLED_ADDRESS` | `0x3C` | Some modules use `0x3D` |

The times and temperatures are starting points. Tune them for your grind and beans.

The firmware assumes the group solenoid opens the path to the group head only while brewing, and that the steam/water knob on the wand is opened by hand. If your machine is plumbed differently, change the `valve` column in the `STATES` table.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| No serial output on the Debug Probe | The Probe's **U** (UART) port isn't wired to GP0/GP1, or the D and U cables are swapped |
| OpenOCD: `cannot read IDR` | The Pico isn't powered, or the SWD wiring is wrong (Probe **D** port to the Pico's DEBUG header) |
| Pico regulator gets hot | A short on 3V3, such as a solder bridge, reversed OLED VCC/GND, or relay coils powered from 3V3 |
| OLED shows random dots with garbled text at the top | SH1106 display with `OLED_COLUMN_OFFSET` or the driver set up for SSD1306 |
| `I2C scan: no devices` | Check SDA/SCL wiring and the OLED's power |
| `FAULT: THERMOCOUPLE` | Thermocouple open circuit, or the MAX6675 isn't wired or powered |
| `FAULT: HEAT TIMEOUT` | Heater relay or heater not working, or the thermocouple isn't attached to the block |

## License

[MIT](LICENSE)
