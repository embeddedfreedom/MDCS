# MDCS v2.0 — HIL, flywheel, furnace

Modular Dynamics Control Suite is a real-time control-systems emulator written in C: 20 kHz physics, 1 kHz compensators, 3D visualisation, live telemetry, and a complete hardware-in-the-loop path to an STM32F411.

## Models

| Model | Control |
|---|---|
| Rotary inverted pendulum (Furuta) | LQR + energy swing-up |
| Pendulum with DC motor | LQR + integral, energy swing-up |
| Flywheel with DC motor — position | LQI + deadzone inverse |
| Flywheel with DC motor — speed | PI + feedforward |
| Furnace control | 2×2 coupled thermal LQI, dual PWM |

## HIL

- Complete HIL implementation for every model — the MCU runs the control law, the host runs the plant.
- STM32F411 over USB CDC at 921600 baud.
- Fixed 12-byte frames, XOR checksums, 3 ms poll timeout; ≈0.4 ms host↔MCU round trip.
- Bare-metal firmware: single-precision FPU only, no libm, no dynamic allocation.
- `make -f Makefile_HIL` builds the host binary and `MDCS_HIL.out` / `.bin`.

## Build

SIL:

```
cd models/rotary_inverted_pendulum
make
```

HIL:

```
make -f Makefile_HIL
```

Flywheel: `make position` / `make speed`, and the same targets with `-f Makefile_HIL`.

## Documentation

- `README.md` — architecture, installation, HIL frame format, bare-metal rules.
- Per-model `docs/LQI_DERIVATION.md` — derivations, gain selection, code mapping.
- Real furnace HIL packet trace in the README.

## Requirements

Linux, GCC, Python 3 with `pyserial numpy pyside6 pyqtgraph pygame PyOpenGL`; ARM GNU toolchain for HIL.

## License

GPLv3. TinyUSB is vendored under MIT in `tinyusb-master/`.

## Credits

Built by Diptopal Basu (embeddedfreedom). AI assisted with derivation and documentation under my direction; all embedded firmware, HIL transport, Makefile automation, hardware validation on the STM32F411, and final code review were done by me.
