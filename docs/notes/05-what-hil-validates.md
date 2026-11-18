# What HIL validates — and what it does not

**Claim under test.** "The control law runs on real hardware."

That sentence is true and easy to over-read. This note states exactly what a passing MDCS HIL run proves, and what it leaves unproven, so the result is not asked to carry more weight than it can.

## The setup

- The **plant** is simulated on the host: RK4 at 20 kHz, fixed 50 µs step.
- The **controller** is real firmware on an STM32F411, receiving sensor values over USB CDC and returning control effort.
- The **interface** is the 12-byte frame described in `03-hil-frame-protocol.md`.

This is a controller-in-the-loop configuration: real control code, simulated physics. It is the standard use of the term HIL, but the hardware under test is the controller, not the plant.

## What a passing run does establish

| Property | How it is exercised |
|---|---|
| The firmware builds and boots | Board enumerates as `cafe:4001 MDCS_HIL` |
| The control law is numerically identical to the SIL law | Same source, same gains, compared outputs |
| Floating-point behaviour on the target | Single precision only, no `__aeabi_d` in the ELF |
| The transport works under timing pressure | ≈ 0.4 ms round trip inside a 1 ms control period |
| Framing and checksums reject corruption | Deliberate bad frames are dropped, not acted on |
| Host-side disconnect handling | Any transport error zeroes the command and stops the loop — within ~4 ms of the board going silent (3 ms poll timeout plus one control period) |
| Output saturation | Duty clamped to `[0, 1]` or `[-Vmax, +Vmax]` on the MCU side |
| Recovery after disconnect | Host closes and reopens the port; the next valid frame resumes control |

Those are real guarantees. They are also *all* of the guarantees.

## What a passing run does not establish

**Plant fidelity.** The plant is the host simulation. HIL validates the code path around it, not the physics inside it. If the Lagrangian has the wrong inertia, or the friction model is linear when the real bearing is not, HIL will happily validate a controller for a plant that does not exist.

**Sensor realism.** Encoder edges, missing pulses, and injected jitter are generated in software. They are useful for stressing the decoder, but they are not a model of a real optical encoder's behaviour at temperature or under vibration.

**Actuator dynamics.** The plant applies a mean-value PWM duty cycle. The firmware writes a timer compare register, but no gate driver, no current loop, no dead time, and no switching ripple are in the loop. A controller that is stable here can still oscillate against a real H-bridge.

**Electrical reality.** No EMC, no ground bounce, no cable length, no supply sag, no thermal drift of the electronics.

**MCU-side failsafe.** The firmware stops computing when frames stop arriving, and the PWM compare register holds its last value. A SysTick watchdog path exists in source but is not enabled in the current build. For a student bench whose actuator is not driving the simulated plant, "hold the last output and wait" is a reasonable behaviour; it is not a safety function, and it should not be treated as one.

**Safety.** Nothing in this bench is certified, redundant, or fail-safe in the SIL-2/ASIL sense. It is a teaching tool.

## The measurement is local

The ≈ 0.4 ms round trip (0.32–0.50 ms observed) was measured on one host, one board, one cable, with the host otherwise idle. It is an existence proof — the link is fast enough for a 1 kHz loop — not a specification. A slower host, a hub, or a busy USB bus will change it.

## How the gap would be closed

In rough order of cost:

1. **Bench step response.** Apply a known input to the real plant, record the response, compare with the simulation. One afternoon, and it catches gross parameter errors immediately.
2. **Parameter identification.** Fit inertia, friction, and thermal mass from measured data instead of datasheet values.
3. **Torque constant calibration.** Measure current versus torque on the actual motor rather than assuming it.
4. **Gate-driver and current-loop characterisation.** Scope the PWM output, measure dead time, check the current loop bandwidth.
5. **Long-duration runs.** Thermal drift and supply sag only show up over hours.

None of these are done in this repository. The honest summary is: **MDCS HIL proves the firmware and the link; it does not prove the plant.**

## Why this note exists

Because "hardware-verified" is a phrase that can mean anything from "flashed once and it blinked" to "certified against a real engine". Being precise about which one applies is cheaper than being corrected later — and a result that is honestly scoped is more useful than one that is oversold.
