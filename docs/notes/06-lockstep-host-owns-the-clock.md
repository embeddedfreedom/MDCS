# Lockstep HIL: the host owns the clock

**Problem.** A simulated plant and a real controller are two separate processors. If each keeps its own notion of time, they slip — and a slip between a plant and its controller is not a cosmetic timing error. It changes the effective sample rate of the derivative term, it corrupts integrator wind-up, and in the worst case it turns a stable loop into an unstable one without any code being wrong.

This note describes the mechanism that prevents it, why USB CDC was chosen for it, and the exact condition under which the mechanism is valid.

## The arrangement

- The **plant** runs on the host at 20 kHz, fixed 50 µs step.
- Every **20 plant steps** (1 ms of simulated time), the host calls the compensator.
- That call is **blocking**: it writes a 12-byte frame, waits up to 3 ms for the reply, validates it, and returns the control effort.
- The **MCU has no control clock.** It does not schedule anything. It sits in `wfi`; a USB packet wakes it, the RX callback runs the control law once, and the reply is flushed immediately.

```c
/* host, inside the 1 kHz slot of the plant loop */
if (i >= 20) {
    if (runloop) {
        pwm = run_compensator_cycle(&bus, &p, &dir);   /* blocks for the round trip */
    }
    ...
    i = 0;
}
```

```c
/* MCU, inside the USB receive callback */
if (Extract_HostToMcuPacket(rx_packet, &pos1, &pos2, &cmd)) {
    float out1 = run_compensator_cycle_HIL(pos1, pos2);
    Write_McuToHostPacket(tx_packet, out1, pos2, 0x01);
    tud_cdc_write(tx_packet, HIL_PACKET_SIZE);
    tud_cdc_write_flush();
}
```

One frame in, exactly one control update out. The MCU cannot run ahead, because it has nothing to run on; it cannot fall behind, because it does no work between frames.

## Why this is a common clock

The plant's time and the controller's execution are the same time base. Simulated time advances only when a frame completes — not approximately, structurally. That is a phase lock between plant and controller, in the same sense as a shared clock line between two chips: there is no second oscillator to drift against.

This is not an aesthetic preference. The MCU code contains hardcoded assumptions that only hold under lockstep:

```c
/* velocity from finite difference */
float d1 = (t1 - t1_prev) / 0.001f;

/* integrator */
float q1_next = clampf(q1 + e1 * 0.001f, -INT_CLAMP, INT_CLAMP);
```

Both constants mean "one call per millisecond of plant time". Under lockstep that is guaranteed by construction. Under a free-running MCU timer it would be an assumption about the host's scheduling — and a wrong one the first time Linux preempts the process.

## What happens when the host is preempted

This is the case the design is built for.

Linux can stall a userspace process for tens or hundreds of milliseconds. Under lockstep, the plant is frozen during that stall, because it lives in the same thread that is blocked on the serial exchange. When the thread is scheduled again, the plant resumes at exactly the simulated instant it stopped, and the MCU resumes from exactly the controller state it held. Wall-clock time was lost; **simulated phase was not**.

Under a free-running MCU clock, the same stall produces one of two failures:

- the controller keeps firing at 1 kHz against a sensor value that is no longer updating — the integrator winds up, the derivative goes to zero, and the loop is effectively running open-loop on stale data;
- or the plant advances while the controller does not, and every subsequent sample carries a phase error that the loop interprets as plant dynamics.

Both are silent. Nothing in the telemetry says "the controller and the plant disagreed about time". That is why the lockstep is worth the cost.

## Why USB CDC, interrupt-driven

The requirement is not "a serial link". It is "a link whose latency is a single round trip and whose timing does not depend on the MCU polling".

- The MCU idles in `wfi` and wakes on packet arrival. There is no polling loop, no timer tick, no baud-rate-driven sample clock to reconcile — the frame arrival *is* the controller's clock.
- One round trip fits inside the 1 ms control period with room to spare. The measured budget and the framing that makes it possible are in `03-hil-frame-protocol.md`.

A UART implementation *can* work, and this repository retains one, but a UART receiver is usually serviced either by polling (adds a sampling interval to every exchange) or by an interrupt that still needs a framing layer above it. USB CDC gives enumeration, a bulk endpoint with CRC and retry underneath, and a standard host driver — so the only thing left to design is the 12-byte frame.

## The costs, stated plainly

1. **The control rate is capped by the round trip.** 1 kHz fits; 10 kHz would not — the link would become the controller's sample rate, not the plant's.
2. **Every control cycle blocks the host.** The physics loop cannot proceed while waiting for the reply. That is the price of phase lock: worst-case link latency becomes part of the cycle budget, and the poll timeout bounds how long a dead board can stall the loop.
3. **Losing the link means losing the controller.** The host treats any transport error as a fault: it zeroes the command within one poll timeout, stops the loop, and requires `S` to restart. There is no degraded mode, and no attempt to extrapolate.
4. **Phase lock is not a real-time guarantee.** It guarantees plant and controller stay *consistent* with each other. It does not guarantee the pair runs at 1 kHz of wall-clock time. Under sustained host load the simulation runs slower than real time, correctly and consistently.

## The condition that makes it valid

This scheme works because **the plant is simulated on the same host thread that drives the controller**. Freezing the plant during a host stall is legitimate — there is no physical process continuing without its controller.

With a real plant, it would be wrong. A physical motor keeps spinning while the host is descheduled, so "stop simulating until the controller replies" is not available. A real HIL rig needs the controller to run on its own real-time clock and the host to timestamp and resynchronise, accepting a bounded phase error rather than eliminating it.

That distinction is the honest boundary of this design: it is a software-in-the-loop plant with a hardware controller, phase-locked by construction. It is not a real-time rig for a physical plant, and it should not be described as one.

## Related

- `03-hil-frame-protocol.md` — frame layout, checksums, and the current state of the MCU watchdog.
- `02-fixed-rate-timing-and-jitter.md` — how the host keeps its own 20 kHz/1 kHz cadence.
- `05-what-hil-validates.md` — what this arrangement does and does not prove.
