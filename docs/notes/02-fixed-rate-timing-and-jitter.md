# Fixed-rate loops on Linux: why `usleep(50)` drifts, and what the jitter alert actually measures

**Symptom.** A controller that is supposed to run at exactly 1 kHz occasionally prints `Timing Jitter Alert: Cycle took 1.3 ms`, and the average rate is a little below 1 kHz even when nothing looks slow.

This note covers the scheduling scheme used by every MDCS simulator, what it guarantees, and what it does not.

## The loop shape

Two rates, one thread:

| Rate | Period | Work |
|---|---|---|
| 20 kHz | 50 µs | RK4 plant step, encoder pulse generation, Z-index check |
| 1 kHz | 1 ms | Compensator call, telemetry write, bus reset |

The compensator is not scheduled from a timer. It fires once every 20 physics steps:

```c
i++;
if (i >= 20) {
    /* 1 kHz compensator */
    ...
    i = 0;
}
```

That is deliberate. The compensator consumes the encoder edges accumulated over exactly 20 plant steps, so the control interval is defined in simulation steps rather than wall-clock time. The loop is a fixed-step simulation that happens to be paced by a clock — not a real-time task that happens to simulate.

## Why relative sleep drifts

`usleep(50)` sleeps *at least* 50 µs, then the loop does its work, then sleeps again. The actual period is therefore

```
period = work + sleep
```

If the plant step costs 8 µs, the loop runs at 58 µs — 17.2 kHz, not 20 kHz — and the error accumulates against the wall clock without bound. Every timing diagram, every jitter measurement, and every telemetry timestamp slowly slides.

## Absolute deadlines

The fix is to schedule against an absolute timeline that is advanced independently of how long the work takes:

```c
const long NS_PER_STEP = 50000;   /* 50 us */

struct timespec next_step;
clock_gettime(CLOCK_MONOTONIC, &next_step);

while (1) {
    /* ... 20 kHz work ... */

    next_step.tv_nsec += NS_PER_STEP;
    if (next_step.tv_nsec >= 1000000000L) {
        next_step.tv_nsec -= 1000000000L;
        next_step.tv_sec++;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_step, NULL);
}
```

`TIMER_ABSTIME` means "sleep until this instant", not "sleep for this long". Work time is subtracted automatically: an 8 µs step sleeps 42 µs, a 30 µs step sleeps 20 µs, and the period stays 50 µs.

`CLOCK_MONOTONIC` is used rather than `CLOCK_REALTIME` because the latter can jump when NTP adjusts the system clock. A monotonic clock never goes backwards.

## What happens under overload

If a step takes longer than 50 µs, `next_step` is already in the past when the sleep is reached, so `clock_nanosleep` returns immediately. The loop then runs flat out until the backlog is cleared. Three consequences:

- **The step rate is not guaranteed.** The code asks for 20 kHz; the OS delivers whatever it can.
- **Per-step determinism is preserved.** `dt` is a compile-time constant (`0.00005f`), never measured from the clock. A late step is still a 50 µs step, so the step sequence never changes — only how fast it is delivered.
- **Simulation time and wall time diverge.** A long stall does not produce one giant integration step — it produces a burst of normal steps.

This is a soft real-time design. There is no `SCHED_FIFO`, no CPU isolation, and no `mlockall`. A desktop kernel can preempt the loop for milliseconds. The mitigation is architectural rather than scheduling: the freeze is deliberate, and `06-lockstep-host-owns-the-clock.md` explains what it buys.

## What the jitter alert measures

```c
clock_gettime(CLOCK_MONOTONIC, &current_time);
double elapsed_ms = (current_time.tv_sec - last_comp_time.tv_sec) * 1000.0
                  + (current_time.tv_nsec - last_comp_time.tv_nsec) / 1000000.0;

if (elapsed_ms > 1.05 || elapsed_ms < 0.95) { /* alert */ }
last_comp_time = current_time;
```

It measures the interval between compensator entries — 20 physics steps — against a 1 ms target with ±5% tolerance. It does **not** measure:

- per-step jitter inside the 20-step block,
- worst-case latency of a single iteration,
- scheduling delay distribution.

A loop can be perfectly regular at the 1 kHz boundary and still stutter at 20 kHz. If that matters, timestamp the top of the loop into a ring buffer and analyse the histogram offline:

```c
struct timespec t;
clock_gettime(CLOCK_MONOTONIC, &t);
uint64_t ns = (uint64_t)t.tv_sec * 1000000000ull + t.tv_nsec;
/* store ns - prev_ns in a preallocated array; dump on exit */
```

Preallocate the buffer. Calling `printf` or `malloc` from inside the loop measures the wrong thing and perturbs what it measures.

## Known quirk in the alert path

The first `printf` in the alert block is unconditional; the throttle counter only controls a second, flushed copy. The practical effect is that every violating cycle prints, and every 200th prints again with a flush. It is noisy but harmless, and it is deliberately left alone: the alert is diagnostic scaffolding, not production telemetry.

## Why not just use an RTOS or a timer ISR

Because this is a host program whose job is to make a desktop behave like a deterministic embedded target. An RTOS would add a scheduler to a system that already has one, and a timer ISR would move the control law out of the readable super-loop that students are meant to study. The absolute-deadline super-loop is the smallest mechanism that gives fixed-rate behaviour with fixed-step determinism, and it maps directly onto what an MCU does with a hardware timer.
