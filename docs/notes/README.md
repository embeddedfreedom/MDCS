# Engineering notes

Short write-ups of problems that were actually hit while building MDCS, what caused them, and what fixed them. They are written for someone who is currently stuck on the same thing — symptom first, no overview, no marketing.

Also published at [embeddedfreedom.dev/notes](https://embeddedfreedom.dev/notes/). These files are the source; the site pages are generated from them.

| # | Note | The problem it answers |
|---|---|---|
| 1 | [A 20 kHz integrator that heats but never cools](01-float-precision-slow-plant.md) | A slow thermal plant converges in simulation until the step size falls below a `float` ULP, and then stops moving entirely. |
| 2 | [Fixed-rate loops on Linux](02-fixed-rate-timing-and-jitter.md) | Why `usleep(50)` drifts, how absolute deadlines fix it, and what the jitter alert actually measures. |
| 3 | [A 12-byte HIL frame over USB CDC](03-hil-frame-protocol.md) | Framing, sync hunting, XOR checksums, a 3 ms poll deadline, and what happens when the host stops sending. |
| 4 | [Porting a compensator to Cortex-M4](04-bare-metal-float-only-port.md) | The float-only rules, and the three linker checks that catch a `double` before it reaches hardware. |
| 5 | [What HIL validates — and what it does not](05-what-hil-validates.md) | Real controller, simulated plant: the exact scope of a "hardware-verified" claim. |
| 6 | [Lockstep HIL: the host owns the clock](06-lockstep-host-owns-the-clock.md) | Why the plant and controller share one time base, and why USB CDC was chosen to keep them in phase when the host is preempted. |

## Conventions used in these notes

- Numbers are from the code in this repository, on the hardware described. Where a figure is a single local measurement rather than a guarantee, the note says so.
- Code excerpts are trimmed to the relevant lines, but they are real excerpts, not pseudocode.
- Each note ends with its own scope: what the fix does not cover.

## Related material

- `models/furnace_control/docs/LQI_DERIVATION.md` — the full derivation behind the thermal controller.
- `README.md` §6 — HIL build, flash, and pin mapping.
- `hil/hil_STM32F4/` — shared firmware: USB CDC adapter, packet definitions, PWM timers, watchdog path (not enabled in the current build).

---

Part of MDCS. Documentation is licensed GPLv3 — see `LICENSE-GPLv3`.
