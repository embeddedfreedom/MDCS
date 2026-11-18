# A 12-byte HIL frame over USB CDC

**Problem.** A host-side plant simulation at 1 kHz needs to talk to a real STM32F411 running the control law, with a round-trip latency well below the 1 ms control period, and with a failure mode that fails safe instead of applying a stale or corrupted command.

This is the framing scheme that emerged. It is deliberately small: 12 bytes, fixed length, one checksum byte, no length field, no sequence number.

## Wire format

Both directions use the same 12-byte layout, only the field names differ:

| Offset | Size | Host → MCU | MCU → host |
|---|---|---|---|
| 0..1 | 2 | `0xAA 0x55` | `0x55 0xAA` |
| 2..5 | 4 | `float v1` | `float out1` |
| 6..9 | 4 | `float v2` | `float out2` |
| 10 | 1 | `uint8 cmd` | `uint8 status` |
| 11 | 1 | XOR of bytes 0..10 | XOR of bytes 0..10 |

Sync patterns are reversed between directions so a loopback or a mis-wired adapter cannot be mistaken for a valid reply.

The struct is packed, and the size is checked at compile time:

```c
#pragma pack(push, 1)
typedef struct __attribute__((packed)) {
    uint8_t sync[2];
    float   pos1;
    float   pos2;
    uint8_t host_cmd;
    uint8_t checksum;
} HostToMcuPacket;
#pragma pack(pop)

_Static_assert(sizeof(HostToMcuPacket) == 12, "HostToMcuPacket is not 12 bytes!");
```

The static assertion is the important part. Without it, a compiler that inserts padding between `pos2` and `host_cmd` silently changes the wire format, and the failure looks like a checksum bug.

## Why fixed length

A variable-length protocol needs either an escaping scheme or a length field plus a resynchronisation strategy. Both add code to the receive path, and the receive path here runs inside a USB callback on a Cortex-M4 with no RTOS.

At 12 bytes the parser is three states and no allocation:

```c
switch (parser_state) {
case STATE_HUNT_SYNC0:
    if (byte == 0xAA) { rx_packet[0] = byte; parser_state = STATE_HUNT_SYNC1; }
    break;

case STATE_HUNT_SYNC1:
    if (byte == 0x55)      { rx_packet[1] = byte; rx_idx = 2; parser_state = STATE_COLLECT_PAYLOAD; }
    else if (byte == 0xAA) { rx_packet[0] = 0xAA; }   /* re-align on AA AA 55 */
    else                   { parser_state = STATE_HUNT_SYNC0; }
    break;

case STATE_COLLECT_PAYLOAD:
    rx_packet[rx_idx++] = byte;
    if (rx_idx == HIL_PACKET_SIZE) {
        parser_state = STATE_HUNT_SYNC0;
        /* validate + dispatch */
    }
    break;
}
```

## Checksum: XOR, and its limits

XOR over bytes 0..10 is weak. It cannot detect two swapped bytes, it cannot detect an even number of flipped bits in the same position, and a random 11-byte payload passes with probability 1/256.

It is used here because the link is short, point-to-point, and travels over USB, which already has a CRC16 at the packet layer plus retry. The XOR byte is a framing sanity check, not an integrity guarantee. If this protocol were moved to a bare UART, a noisy cable, or a multi-drop bus, the checksum would need to become CRC-16/CCITT and the frame would need a sequence number so a duplicate reply cannot be mistaken for a fresh one.

The host side enforces the same rule in reverse and refuses the frame if either sync byte or the checksum is wrong:

```c
if (rx_pkt.sync[0] != 0x55 || rx_pkt.sync[1] != 0xAA || rx_pkt.checksum != rx_chk) {
    simulation_shutdown();
    *motor_dir = 0;
    return 0.0f;
}
```

## Timing budget

| Item | Value |
|---|---|
| Baud rate | 921600, 8N1 |
| One 12-byte frame on the wire | 12 × 10 bits ÷ 921600 ≈ 130 µs |
| Measured host → MCU → host | 0.32–0.50 ms (≈ 0.4 ms typical) |
| Control period | 1 ms |
| Host poll timeout | 3 ms |
| MCU watchdog (defined; not enabled in the current build) | 5 ms |

The measured 0.4 ms includes the wire time in both directions plus USB stack latency on the host and device. It leaves roughly 60% of the control period unused, which is the margin that makes a 1 kHz HIL loop viable over USB CDC at all.

The 3 ms poll timeout is three control periods: long enough to survive a scheduling hiccup, short enough that a dead board cannot stall the physics loop for a visible amount of time.

## Failure behaviour

The host never applies a stale or corrupted command. Any transport error collapses to zero control effort and stops the loop.

**Host.** Any short write, poll timeout, short read, bad sync, or bad checksum closes the serial descriptor, prints `CONNECTION ERROR`, sets the shared `runloop` flag to false, and returns zero output. The simulation keeps running with the plant unpowered. Pressing `S` reopens the port and resumes.

Closing the descriptor matters: it lets the next cycle call `open()` again, so a board that is unplugged and replugged recovers without restarting the simulator.

This is the real cutoff if the board hangs. The next exchange waits 3 ms and then fails; worst case is one control period plus the poll timeout, so control effort is zero within roughly 4 ms of the MCU going silent, and stays zero until `S` is pressed. The plant is simulated, so the consequence is a stalled model, not a runaway actuator.

**MCU.** The firmware is interrupt-driven. It idles in `wfi`; a frame arrival is the only thing that produces a control update. There is no free-running control task, so when the host stops sending, the MCU stops computing and returns to its idle loop. The timer compare register keeps its last value, so the last commanded duty remains on the pin until the next frame arrives.

A SysTick watchdog path exists in source — 5 ms, clears `compensator_enabled`, forces a re-sync so the controller does not resume with stale state — but `enable_systick()` is commented out in the USB build and the flags are not read by the compensator, so it does not run. Status is a constant `0x01`.

That is appropriate for the intended use. This is a student bench, the actuator is not driving the simulated plant, and a controller that simply stops when its host stops is the correct behaviour. The watchdog code is documented here so it is not mistaken for an active safety function.

Status bits are defined as:

| Bit | Meaning |
|---|---|
| 0 | `HIL_STATUS_RUNNING` — PA0 button state |
| 1 | `HIL_STATUS_TIMEOUT` — watchdog has tripped (defined, not currently produced) |

Neither is currently varied at runtime; the adapter writes `0x01` on every reply.

## Portability assumptions

- Both ends are little-endian. True for x86-64 and Cortex-M4, but not stated anywhere in the wire format.
- Both ends use IEEE-754 binary32 for `float`. True for GCC on both targets.
- `float` is 4 bytes and `uint8_t` is 1 byte, enforced by `_Static_assert`.

A portable version would serialise floats byte-by-byte in a defined order instead of relying on struct layout. For a single-vendor, single-toolchain bench link, the struct is the pragmatic choice and the assertion catches the realistic failure.

## What I would add for a production link

1. **CRC-16 instead of XOR**, if the transport is not USB.
2. **A sequence counter**, so a duplicated or reordered reply is detectable.
3. **A protocol version byte**, so host and firmware refuse to talk across a format change.
4. **An explicit length field**, if payloads ever become model-specific.

None of these are needed for a 12-byte frame on a 30 cm USB cable with a checksum at the transport layer. All of them would be needed the moment the link leaves the bench.
