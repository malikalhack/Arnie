# Arnie — Project History

Milestone log for the Arnie lidar-robot firmware.
Records key decisions, structural changes, and completed phases.

---

## 2026-06-19

### Project inception
- Firmware project **Arnie** started for a differential-drive mobile robot on
  **STM32F103C8T6 (Blue Pill)**, sensing with a **Delta-2B laser radar** and an
  **HMC5883L magnetometer**.
- Solution `arnie.csolution.yml` (CMSIS-Toolbox), single project
  `navigation/navigation.cproject.yml`.

### Architecture decisions
- **Dual toolchain** — every source must build clean under **AC6 6.20.1** and
  **GCC 15.2.1**; toolchain-specific options isolated in `for-compiler` blocks.
- **Device header** — Keil `STM32F1xx_DFP` 2.4.1 maps `CMSIS_device_header` to
  the **old StdPeriph `stm32f10x.h`** (register bits named `DMA_CCR1_*`,
  `I2C_CR1_*`, etc.), not the LL `stm32f103xb.h`.
- **Clocking** — HSE 8 MHz → PLL ×9 = 72 MHz; APB1 = 36 MHz, APB2 = 72 MHz.
- **Static memory only** — no `malloc`/`free`.
- **Code style** — `.github/copilot-instructions.md` (file headers, 80 ± 2
  columns, single entry/exit, CMSIS structs only, Doxygen).
- **Date format** — ISO 8601 (`YYYY-MM-DD`). The first `@date` is the file
  creation date (never changed); the second `@date @showdate` is the Doxygen
  build date.

### BSP — board bring-up
- `bsp/bsp.c` / `bsp.h` (v0.1.0): `SystemInit()` configures Flash latency +
  prefetch, RCC (72 MHz, records `system_clock_hz`), GPIO (PC13 LED active-low),
  USART1 and the vector table — all before `main()`. It deliberately does
  **not** start SysTick or enable interrupts.
- `bspStart()` (called from `main()`): starts the 1 ms SysTick, enables the
  configurable fault exceptions (MemManage/BusFault/UsageFault) and global IRQs.
- `SysTick_Handler()` increments `volatile AcroTick_t sys_tick` (strong
  override of the weak startup handler).
- **Lesson** — SysTick cannot be enabled inside `SystemInit()`: on AC6,
  `SystemInit()` runs before the `__main` scatter-load, and the weak handler
  would hang on the first tick. The timebase belongs in `bspStart()`.

### Lidar driver
- `navigation/lidar.c` / `lidar.h` (v0.1.0): USART1 RX via circular **DMA1
  Ch5**, byte-fed frame state machine, checksum validation, and a 360-sector
  polar histogram (minimum distance per 1° sector). A revolution wrap
  (start-angle 360°→0°) publishes a double-buffered scan.
- API: `lidarInit`, `lidarProcess`, `lidarGetScan`, `lidarGetScanCount`,
  `lidarGetSpeedRaw`, `lidarGetFault`.

---

## 2026-06-20

### Scheduler integration (AcroSched)
- AcroSched linked as a **prebuilt per-compiler library**: `lib/stm32f1-ac6/
  acrosched.lib` (AC6) and `lib/stm32f1-gcc/libacrosched.a` (GCC), in a
  `Scheduler` group with `for-compiler` conditions.
- Headers **vendored locally** in `lib/inc/` (acrosched.h, _config.h, _defs.h,
  _ipc.h, _kernel.h, acrosched_port.h) with add-path `../lib/inc`. The Cortex-M
  AC6 and GCC ports are byte-identical, so one vendored `acrosched_port.h`
  suffices.
- Contract: `bspStart()` → `acroInit(&sys_tick)` → `acroAddTask(...)` →
  `acroRun()`. Default config: `TICK = uint32_t`, `MAX_TASKS = 8`,
  watchdog = 0, ipc = 0 — must match the library build.

### Application skeleton
- `app/main.c` (v0.3.0): defines tasks `taskLidar` (eRealtime) and
  `taskHeartbeat` (ePeriodic, 500 ms). Uses the AcroSched macros
  `CHECK_STATUS()` (Error_Handler on non-`eAcroOk`), `IGNORE_RETURN()` and
  `UNUSED()`.

### Code-style pass
- Enforced the 80 ± 2 column limit across edited files (split long `if`
  conditions, shortened comments). Verified by a UTF-8-aware line scan.

### Hardware notes (5 V tolerance)
- Confirmed from the datasheet that the lidar's 5 V UART TX can drive the MCU
  RX directly: **PA10 (USART1_RX) is `FT` (5 V tolerant)** — no level shifter
  needed. (Non-FT pins on this part are only the ADC-shared ones: PA0–PA7,
  PB0/PB1, PC0–PC5.) PB10/PB11 are also FT.

---

## 2026-06-21

### Compass driver (HMC5883L)
- `navigation/compass.c` / `compass.h` (v0.1.0): polled **I2C1** driver
  (PB6 SCL / PB7 SDA), standard mode 100 kHz (CR2 FREQ = 36, CCR = 180,
  TRISE = 37). Configured for 8-sample averaging at 15 Hz, continuous mode.
  Device identity ('H','4','3') verified at init.
- All bus waits are **bounded by a timeout** (`COMPASS_I2C_TIMEOUT`) so a stuck
  bus can never hang the cooperative dispatcher.
- Data registers stream X, Z, Y (signed 16-bit big-endian). Heading is exposed
  in deci-degrees (0..3599).
- **No libm** — heading uses an in-tree cubic four-quadrant arctangent
  (`compassAtan2`, ~0.3° error) built from basic float arithmetic (libgcc
  soft-float). Coefficients are exact at 0°/45°/90° (C1 = 5π/16, C3 = π/16).
- **Lesson** — `atan2f` from libm failed to link under GCC: CMSIS-Toolbox
  places `misc → Link` flags **before** the object files, and GNU `ld` resolves
  left-to-right, so `-lm` was never consulted (`undefined reference to atan2f`).
  Avoiding libm side-steps the ordering problem and keeps the image small.
- `compass.c` added to the build in a `Compass` group; integrated into `main.c`
  (v0.4.0): `compassInit()` at start-up, heading printed by the heartbeat task.

### Debug console moved to USART2
- The lidar occupies **both** USART1 lines (PA9 + PA10), so the debug output
  cannot share PA9.
- `uartSend*` retargeted to **USART2 TX on PA2**, 115200 8N1 (PCLK1 = 36 MHz,
  BRR = 313). USART1 stays dedicated to the lidar. `bsp.c` bumped to v0.2.0.
- Pin map updated: lidar = PA9+PA10 (both USART1 lines), debug = USART2/PA2.

### Documentation
- `requirements.md` created — 12 active requirements (REQ-001..012) plus 7
  planned (REQ-013..019) for the motion / navigation / hardening phases.
- `HISTORY.md` created (this file).

### Lidar checksum fix — all-zero telemetry resolved
- **Root cause** — the frame parser validated frames with a cumulative byte
  **sum**, but the Delta-2B actually uses a **Modbus CRC16** (poly 0xA001,
  init 0xFFFF) over bytes 0xAA … last param byte. Every frame failed the sum
  check, so no scan was ever published (telemetry read all zeros).
- **Fix** — incremental CRC16 in `lidarChecksumByte()` (running `parser.crc`),
  with the cumulative `parser.sum` kept in parallel. `lidarFrameChecksum()`
  selects CRC when the address byte (`parser.proto`, buf[3]) `< 1`, else the
  sum (the device-info frame 0xAC is sent once at start-up with PROTO = 0x01
  and uses the sum). Compared big-endian (`CheckHi << 8 | CheckLo`).
- After the fix: scans increment ~7 Hz, ~250–308 of 360 sectors populated,
  min distance tracks real obstacles (133–263 mm) — **lidar fully working**.
- **Doc discrepancy recorded** — the workspace `.odt` protocol doc wrongly
  describes a cumulative-sum checksum (its worked example verifies as a sum);
  real hardware + SDK use CRC16. Captured in repo memory.
- **Lesson** — the earlier slow ~110 B/s "garbage" stream was environmental
  noise (mouse/laptop nearby injecting spurious 0xAA bytes), not the lidar; the
  real lidar streams ~13.6 KB/s.

### Debug-console bug fixed (`uartSendUint8`)
- For values ≥ 100 the routine only handled 0–99, so `'0' + (n / 10)` produced
  punctuation (`:`,`;`,`<`,`=`,`>`). Raw speed ~139 printed as `=8`.
- Rewritten to emit hundreds / tens / units correctly for the full 0–255 range.
  Speed now prints as `139` (≈ 6.9 r/s ≈ 414 rpm).

### Diagnostics cleanup
- Removed all lidar bring-up instrumentation now that the link works: the
  `dbg rx=` / `raw=` / `rxpulse=` telemetry lines, the `lidarGetRxBytes` and
  `lidarGetRawCapture` getters, `bspMeasureRxBitCycles`, and the parser scratch
  fields (`rx_total`, `dbg_raw`, `dbg_raw_len`, `dbg_armed`).
- Kept `uartSendHex8` (general I2C register-dump utility) and the `cfault`
  field for future compass work.
- Telemetry reduced to one clean line:
  `scans=N pts=P min=M mm  speed=S`.

### Compass disabled until hardware arrives
- The **HMC5883L** is not yet on the board; a unit was ordered and arrives end
  of week. The driver (`compass.c`, I2C 0x1E) is complete and ready to run.
- Compass made compile-time optional via `COMPASS_ENABLED` in `main.c`
  (set to `0` for now). With it off, `compassInit`/`compassProcess` and the
  heading telemetry are skipped and the linker drops `compass.c` (AC6 Debug
  Code 8722 → 5186, GCC Debug ROM 8280 → 4916 B). Flip to `1` when the
  magnetometer is fitted.

### Build status
- Both toolchains pass throughout: **AC6 2 succeeded, GCC 2 succeeded.**

---

## 2026-06-22

### BSP refactor — all platform-dependent code centralised in the BSP
- Established the rule that **`bsp/bsp.c` / `bsp.h` is the only board-specific
  translation unit**. Every driver (`lidar`, `compass`, `motor`, `encoder`)
  now includes **`"bsp.h"`** instead of `CMSIS_device_header` and never
  touches an MCU register, peripheral struct or raw address directly.
- Peripheral access that previously lived in the drivers was **moved behind
  named BSP entry points**:
  - Lidar — `bspLidarRxDmaInit()` / `bspLidarRxDmaIndex()` own the USART1 +
    DMA1 Ch5 setup and the circular write index; `lidar.c` keeps only the
    protocol/state-machine logic.
  - Compass — the I2C1 transfers were lifted into `bspI2c1Init()`,
    `bspI2c1WriteReg()` and `bspI2c1ReadRegs()` (polled, bounded by a
    timeout); `compass.c` keeps only register maps and the heading maths.
- Rationale: keeps the drivers **portable and register-free**, isolates the
  StdPeriph `stm32f10x.h` coupling to one file, and gives each port a single
  place to implement. `bsp.h` bumped to **v0.6.0**, `bsp.c` to **v0.5.0**.

### Motor driver (H-bridge, IR2184)
- `navigation/motor.c` / `motor.h` (v0.1.0 / v0.2.0): **sign-magnitude PWM**
  drive for two DC motors. `motorSetA()` / `motorSetB()` take a signed command
  in `[-MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX]` (per-mille); the sign picks the
  driven leg, the magnitude the duty. `motorEnable()` / `motorStop()` arm and
  coast both bridges through the IR2184 **SD** lines (default OFF on reset).
- BSP side (`bspMotorInit`, `bspMotorSetDutyA/B`, `bspMotorEnable`,
  `BSP_MOTOR_DUTY_FULL = 3599`): **20 kHz** PWM (above audible) from 72 MHz
  timers. Motor A on **TIM1_CH1 (PA8) / CH4 (PA11)**, Motor B on
  **TIM4_CH3 (PB8) / CH4 (PB9)**; SD/EN on **PB12 / PB14**.
  - Pin conflicts resolved: USART1 (lidar) occupies PA9/PA10, blocking
    TIM1_CH2/CH3, so motor A uses TIM1_CH1+CH4; I2C1 keeps PB6/PB7, so motor B
    uses TIM4_CH3/CH4 (not CH1/CH2).
  - **TIM1 is an advanced timer** — its outputs stay high-Z until
    `BDTR.MOE = 1`, so `bspMotorInit()` sets the main-output-enable bit.

### Wheel encoder driver (×4 quadrature)
- `navigation/encoder.c` / `encoder.h` (v0.1.0): signed speed + odometry from
  the **hardware ×4 quadrature counters**. Encoder A on **TIM2 (PA0/PA1)**,
  encoder B on **TIM3 (PA6/PA7)**, encoder mode 3 (counts both edges of both
  channels) so direction and travel are tracked with no CPU load.
- `encoderProcess()` takes the **signed 16-bit counter delta** between calls
  (wrap-safe), accumulates it into an odometry position and converts it into a
  signed **rev/min** (frequency method); all non-blocking. Geometry constants:
  100 ppr × 4 (quadrature) × ≈15 (gear) = **6000 counts / wheel-rev**
  (gear ratio to be calibrated on the bench).
- BSP side (`bspEncoderInit`, raw count getters): brings up TIM2/TIM3 as the
  encoder interfaces.

### Integration
- `main.c` gained compile-time switches **`MOTOR_ENABLED`** and
  **`ENCODER_ENABLED`** (both `0` until the hardware is wired), mirroring
  `COMPASS_ENABLED`: while off, the peripherals are not initialised and the
  linker drops the unused drivers.

### Build status
- Both toolchains pass: **AC6 2 succeeded, GCC 2 succeeded.**

---

