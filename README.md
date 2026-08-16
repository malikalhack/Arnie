# Arnie

Firmware for **Arnie**, a differential-drive mobile robot built around an
**STM32F103C8T6 (Blue Pill)**. The robot senses its surroundings with a
**Delta-2B rotating laser radar**, finds its heading with a **GY-271
magnetometer**, drives two brushed DC motors through a custom **H-bridge
(2× IR2184 per motor)** and closes the loop with **×4 quadrature wheel
encoders**. Every activity runs as a task under the **AcroSched** cooperative
scheduler.

## Highlights

- **MCU** — STM32F103C8T6 (Cortex-M3), 72 MHz from an 8 MHz HSE (PLL ×9);
  APB1 = 36 MHz, APB2 = 72 MHz. Fits the 64 KB Flash / 20 KB RAM budget.
- **Dual toolchain** — builds clean under **Arm Compiler 6** and **GCC**
  (arm-none-eabi) through the CMSIS-Toolbox; toolchain-specific options are
  isolated in `for-compiler` blocks.
- **No libm** — transcendental maths (heading arctangent) is approximated
  in-tree, so the image stays small and link-order independent.
- **Static memory only** — no `malloc`/`free` anywhere.
- **Cooperative-safe drivers** — every bus wait is bounded by a timeout; a
  stuck peripheral raises a fault flag instead of hanging the dispatcher.
- **Portable BSP** — `bsp/bsp.c` is the only board-specific file; drivers
  include `bsp.h` and never touch an MCU register directly.

## Hardware

| Subsystem | Interface | Pins | Timer / DMA |
|---|---|---|---|
| Lidar (Delta-2B) | USART1 RX, 230400 8N1, simplex | PA10 (RX, 5 V tolerant) | DMA1 Ch5 |
| Magnetometer | I2C1, 100 kHz | PB6 SCL, PB7 SDA | polled |
| Motor A | TIM1 PWM ~20 kHz | PA8 / PA11, SD PB12 | TIM1_CH1/CH4 |
| Motor B | TIM4 PWM ~20 kHz | PB8 / PB9, SD PB14 | TIM4_CH3/CH4 |
| Encoder A | ×4 quadrature | PA0 / PA1 | TIM2 |
| Encoder B | ×4 quadrature | PA6 / PA7 | TIM3 |
| Debug console | USART2 TX, 115200 8N1 | PA2 | — |
| LED | GPIO (active-low) | PC13 | — |

The magnetometer driver **auto-detects** the chip: it probes an HMC5883L at
`0x1E`, then a QMC5883L-class clone (e.g. "HA5883") at `0x0D`, and configures
whichever answers.

## Software architecture

- **Scheduler** — AcroSched is linked as a per-compiler prebuilt library
  (`lib/stm32f1-ac6/acrosched.lib`, `lib/stm32f1-gcc/libacrosched.a`); its
  headers are vendored in `lib/inc/`. Start-up order is
  `bspStart()` → `acroInit(&sys_tick)` → `acroAddTask(...)` → `acroRun()`.
- **Timebase** — a 1 ms `SysTick` increments a BSP-owned tick counter used as
  the scheduler time source. `SystemInit()` configures clocks, GPIO and the
  USARTs before `main()`; it does **not** enable SysTick or interrupts —
  `bspStart()` does that.
- **Drivers** — `lidar`, `compass`, `motor`, `encoder` under `navigation/`,
  each exposing a non-blocking `*Process()` entry that does one bounded unit of
  work per dispatcher visit.

## Repository layout

```
app/            application entry point and tasks (main.c, syscalls.c)
bsp/            board support package (only platform-specific code)
navigation/     drivers: lidar, compass, motor, encoder
lib/            AcroSched prebuilt libraries + vendored headers
arnie.*.yml     CMSIS-Toolbox solution / project files
HISTORY.md      milestone log of decisions and changes
requirements.md functional / non-functional requirements (REQ-NNN)
```

## Building

The project uses the CMSIS-Toolbox (`cbuild`):

```powershell
cbuild arnie.csolution.yml --rebuild
```

Both Debug and Release contexts build for the `STM32F103C8` target under each
configured toolchain.

## Flashing

With a CMSIS-DAP / ST-Link probe and pyOCD:

```powershell
pyocd flash -t stm32f103c8 --erase chip .\out\navigation\STM32F103C8\Debug\navigation.hex
```

## Status

Lidar acquisition, the magnetometer (auto-detect HMC/QMC), the cooperative
runtime and the dual-toolchain build are working. Motor and encoder drivers are
implemented and compile-time gated (`MOTOR_ENABLED` / `ENCODER_ENABLED` in
`app/main.c`) while the hardware is wired. Closed-loop speed control,
magnetometer calibration and obstacle-avoidance navigation are planned (see
`requirements.md`, REQ-015 onward). The independent watchdog is integrated via
the AcroSched dispatcher hook (`acroWatchdogRefresh()` -> `bspWatchdogKick()`).

## License

See [LICENSE](LICENSE).