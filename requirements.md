# Arnie — Requirements & Task Backlog

**Project:** Arnie (lidar-guided mobile robot firmware)
**Author:** Anton Chernov
**Date:** 2026-06-21
**Version:** 0.1.0

---

## 0. Overview

Arnie is the firmware for a differential-drive mobile robot built around a
**STM32F103C8T6 (Blue Pill)** MCU. The robot senses its surroundings with a
**Delta-2B rotating laser radar** and its orientation with an **HMC5883L
magnetometer**, drives two brushed DC motors through a custom H-bridge, and
runs every activity as a task under the **AcroSched** cooperative scheduler.

This document captures the functional and non-functional requirements.
Each requirement has a stable identifier (`REQ-NNN`) and explicit acceptance
criteria so derived documents (test plans, datasheets, design notes) can refer
to them unambiguously.

---

## 1. Requirements

### REQ-001 — Target Platform

Arnie shall target the STM32F103C8T6 (ARM Cortex-M3) on the Blue Pill board.

**Acceptance criteria:**
- SYSCLK is 72 MHz from the 8 MHz HSE via PLL ×9; APB1 = 36 MHz, APB2 = 72 MHz.
- Flash latency, prefetch and vector-table relocation are configured before
  `main()` in `SystemInit()`.
- The firmware fits the 64 KB Flash / 20 KB RAM budget with margin.

---

### REQ-002 — Dual-toolchain Build

Arnie shall build cleanly with both Arm Compiler 6 (AC6) and GCC (arm-none-eabi)
through the CMSIS-Toolbox (`cbuild`).

**Acceptance criteria:**
- `cbuild arnie.csolution.yml --rebuild --toolchain AC6` reports
  `2 succeeded, 0 failed` (Debug + Release).
- `cbuild arnie.csolution.yml --rebuild --toolchain GCC` reports the same.
- Toolchain-specific options live in `for-compiler` blocks; sources are shared
  unchanged between toolchains.
- No dependency on libm (transcendental math is approximated in-tree).

---

### REQ-003 — Cooperative Scheduler Runtime

Arnie shall run all periodic and event-driven work as tasks under the AcroSched
cooperative scheduler.

**Acceptance criteria:**
- The 1 ms tick is produced by `SysTick_Handler()` incrementing
  `volatile AcroTick_t sys_tick` (BSP-owned, not the library).
- Start-up order is `bspStart()` → `acroInit(&sys_tick)` → `acroAddTask(...)`
  → `acroRun()` (never returns).
- Every scheduler API call is checked; a failure routes to `Error_Handler()`
  via the `CHECK_STATUS()` macro. Deliberate discards use `IGNORE_RETURN()`.
- The scheduler library is linked per-compiler (`acrosched.lib` for AC6,
  `libacrosched.a` for GCC); its headers are vendored in `lib/inc/`.

---

### REQ-004 — Board Support Package

Arnie shall provide a platform BSP that brings up clocks, GPIO, the timebase,
fault exceptions and basic I/O services.

**Acceptance criteria:**
- `SystemInit()` configures clocks, GPIO and the USARTs before `main()`;
  it does **not** enable SysTick or global interrupts.
- `bspStart()` (called from `main()`) starts the 1 ms SysTick, enables the
  configurable fault exceptions (MemManage/BusFault/UsageFault) and global
  interrupts.
- The BSP exposes `bspGetTick()`, `get_system_core_clock()`, `reset_mcu()`,
  LED control and UART transmit helpers.

---

### REQ-005 — Lidar Acquisition

Arnie shall receive and decode the Delta-2B laser radar stream and expose a
polar obstacle map.

**Acceptance criteria:**
- The radar is received on **USART1** at 230400 8N1 over circular **DMA1
  Channel 5**; the link is simplex (radar → MCU).
- A byte-fed state machine validates the frame header, length, type, command
  and 16-bit checksum before accepting a frame.
- Measurements are folded into a 360-sector polar histogram (minimum distance
  per 1° sector); a completed revolution (start-angle wrap) publishes a scan.
- The driver exposes scan data, scan count, rotation speed and fault status.

---

### REQ-006 — Heading / Magnetometer

Arnie shall measure heading using an HMC5883L magnetometer.

**Acceptance criteria:**
- The sensor is on **I2C1** (PB6 SCL / PB7 SDA), standard mode 100 kHz, using
  the module's own pull-ups.
- The driver verifies the device identity ('H','4','3'); on mismatch or bus
  error it raises a fault flag and skips further reads.
- A sample yields raw X/Y/Z counts and a heading in deci-degrees (0..3599).
- The heading uses an in-tree four-quadrant arctangent approximation; **no
  libm dependency**.

---

### REQ-007 — Debug Telemetry Console

Arnie shall provide a human-readable debug console independent of the sensor
links.

**Acceptance criteria:**
- The console is **USART2 TX on PA2**, 115200 8N1 (PCLK1 = 36 MHz, BRR = 313).
- It does not share pins with the lidar (which occupies both USART1 lines).
- The heartbeat task periodically emits scan count, front distance, radar speed
  and heading.

---

### REQ-008 — Cooperative-safe Drivers (no indefinite blocking)

No task shall block the dispatcher indefinitely.

**Acceptance criteria:**
- All peripheral polling loops (e.g. I2C status waits) are bounded by a
  timeout; on timeout the operation aborts and reports failure.
- Bus faults set a fault flag rather than spinning forever.
- Drivers expose a `*Process()`-style entry that performs one bounded unit of
  work per dispatcher visit.

---

### REQ-009 — Static Memory Management

Arnie shall use static allocation only; dynamic allocation (`malloc`/`free`)
is forbidden in application and driver code.

**Acceptance criteria:**
- Buffers (DMA buffer, histograms, parser context) are statically allocated.
- No heap usage appears in BSP, lidar, compass or application sources.

---

### REQ-010 — Resource Allocation Map

Arnie shall maintain a single authoritative pin / timer / DMA allocation map.

**Acceptance criteria:**
- Every peripheral pin, timer channel and DMA channel in use is recorded with
  its function and any conflict resolution.
- New peripherals are assigned from the documented free-pin list only.
- The map is kept consistent with the implemented code.

---

### REQ-011 — Code Style & Documentation

All sources shall follow the project style guide and carry Doxygen
documentation.

**Acceptance criteria:**
- File headers, section banners, naming conventions, single entry/exit and
  declarations-at-block-top follow `.github/copilot-instructions.md`.
- Lines stay within 80 ± 2 columns.
- Public `.h` declarations carry full Doxygen blocks; `.c` definitions carry
  the `/** @fn name */` tag.
- Cortex-M hardware is accessed through CMSIS structures, never raw addresses.

---

### REQ-012 — Versioning

Arnie shall version each source file independently using SemVer in its header
block.

**Acceptance criteria:**
- Each `.c`/`.h` header carries `@version MAJOR.MINOR.PATCH`.
- The first `@date` is the file creation date and is never altered on edits;
  the second `@date @showdate` is the Doxygen build date.
- A meaningful change bumps the file's `@version`.

---

## 2. Planned Requirements (future phases)

### REQ-013 — Motor Drive

Arnie shall drive two brushed DC motors through a custom H-bridge (2× IR2184
per motor) using sign-magnitude PWM.

**Target:** Motion phase

**Acceptance criteria:**
- Motor A PWM on TIM1 (PA8/PA11), Motor B PWM on TIM4 (PB8/PB9), ~20 kHz.
- Hardware enable/stop (SD) lines default to the safe OFF state on reset.
- Direction is selected by swapping which leg is PWM'd; bootstrap refresh is
  preserved on the switching leg.

---

### REQ-014 — Wheel Odometry

Arnie shall measure wheel speed from single-channel tachometer encoders.

**Target:** Motion phase

**Acceptance criteria:**
- Tacho A on TIM2_CH1 (PA0), Tacho B on TIM3_CH1 (PA6) via input capture.
- Speed is derived by the period method; direction is not available
  (single-channel).

---

### REQ-015 — Closed-loop Speed Control

Arnie shall regulate wheel speed with a PID controller on a fixed tick.

**Target:** Motion phase

**Acceptance criteria:**
- The control loop runs at a fixed rate (SysTick-derived) as a scheduler task.
- Setpoint, measured speed and output are observable over the debug console.

---

### REQ-016 — Magnetometer Calibration

Arnie shall apply hard-iron and soft-iron correction to the magnetometer.

**Target:** Navigation phase

**Acceptance criteria:**
- Calibration offsets/scales are applied to raw X/Y/Z before heading
  computation.
- A documented procedure produces the calibration constants from logged data.

---

### REQ-017 — Tilt Compensation

Arnie shall compensate the heading for chassis tilt once an accelerometer /
inertial reference is available.

**Target:** Navigation phase

**Acceptance criteria:**
- Heading remains accurate within the specified tilt range.
- Falls back to flat-plane heading when no tilt reference is present.

---

### REQ-018 — Obstacle Avoidance / Navigation

Arnie shall combine the polar lidar map and heading to navigate while avoiding
obstacles.

**Target:** Navigation phase

**Acceptance criteria:**
- A navigation task consumes the published scan and heading to produce motor
  setpoints.
- A minimum stand-off distance is enforced; the robot stops or turns rather
  than collide.

---

### REQ-019 — Watchdog

Arnie shall service the MCU independent watchdog via the AcroSched watchdog
hook in the dispatcher loop.

**Target:** Hardening phase

**Acceptance criteria:**
- `ACROSCHED_USE_WATCHDOG` is enabled, so the scheduler calls
  `acroWatchdogRefresh()` from the dispatcher loop.
- The firmware provides a strong `acroWatchdogRefresh()` implementation that
  performs the hardware IWDG reload.
- A stalled dispatcher (no refresh) resets the MCU within the configured
  timeout.

---
