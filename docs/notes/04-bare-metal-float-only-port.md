# Porting a compensator to Cortex-M4: the float-only rules, and how the linker tells you when you broke them

**Problem.** The same compensator source file is compiled twice: once for the host simulation (x86-64, glibc, libm) and once for the STM32F411 firmware (bare metal, `-nostartfiles`, no libm). Code that compiles and runs correctly on the host can silently fail to link — or link and then trap — on the target.

This note is the constraint set that makes one source file work in both places, and the three commands that verify it.

## The target

```
arm-none-eabi-gcc -O0 -mcpu=cortex-m4 -g -mthumb -nostartfiles \
    -mhard-float -mfpu=fpv4-sp-d16 \
    -fno-exceptions -fno-unwind-tables
```

- `-mfpu=fpv4-sp-d16` is the Cortex-M4F FPU. It is **single precision only**.
- `-mhard-float` passes float arguments in FPU registers, per the hard-float ABI.
- `-nostartfiles` and no `-lm` mean there is no C library to fall back on.

## What `double` costs there

A `double` on this target is not a slower instruction. It is a software library call:

```
__aeabi_dadd  __aeabi_dsub  __aeabi_dmul  __aeabi_ddiv  __aeabi_d2f  __aeabi_f2d
```

These live in libgcc, pull in additional code, and — worse — mixing them into a hard-float build can produce ABI errors at link time that read like nonsense (`VFP register arguments do not support this operation`). The failure mode is not "the control law is slightly slower". It is "the firmware does not build", or builds with a soft-float helper that corrupts the calling convention.

## The four ways a `double` sneaks in

**1. `M_PI` is a double.**

In glibc and newlib, `M_PI` is defined as a `double` constant. This is correct C and completely wrong here:

```c
float w = 2.0 * M_PI * f;      /* the whole expression is double */
```

Every trig or wrap operation in the firmware uses a local float constant instead:

```c
#define PI_F 3.14159265358979323846f
```

or, where the compensator needs a wrap:

```c
static float wrap_pi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}
```

**2. Unsuffixed decimal literals are `double`.**

```c
float k = 0.001;      /* double, then truncated — may also warn */
float k = 0.001f;     /* correct */
```

This matters most in gain tables. A table of 40 unsuffixed gains is 40 implicit `double` constants, and depending on optimisation level the compiler may materialise them through `__aeabi_d2f`.

**3. Implicit promotion in mixed expressions.**

```c
float y = x * 0.5;        /* x promoted to double, multiply in double */
float y = x * 0.5f;       /* stays single */
```

The same applies to `int` division assigned to a float, and to `pow()`, `exp()`, `sin()` and `cos()` — all of which are double-precision in the default libm.

**4. The math functions without the `f` suffix.**

| Host code | Bare-metal replacement |
|---|---|
| `sqrt(x)` | `sqrtf(x)`, or a Newton loop if libm is absent |
| `fabs(x)` | `fabsf(x)` |
| `round(x)` | `roundf(x)` |
| `fmod(x, y)` | `fmodf(x, y)` |
| `sin(x)`, `cos(x)` | `sinf(x)`, `cosf(x)` — or a lookup table |

The furnace compensator needs one square root, for the steady-state feedforward (`pwm = sqrt(P·R) / Vmax`). With no libm available, it uses a fixed-iteration Newton loop:

```c
static float sqrt_pos(float x) {
    if (x <= 0.0f) return 0.0f;
    float r = x;
    for (int i = 0; i < 12; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
}
```

Twelve iterations from a rough initial guess is more than enough for a feedforward term that is afterwards clamped to `[0, 1]`. It is not a general-purpose `sqrtf`; it is the smallest correct thing for this call site.

## Verification: three commands

**1. No unresolved symbols.**

```bash
arm-none-eabi-nm -u build/furnace_control/MDCS_HIL.out
```

Must print nothing. Any output means the link silently accepted an undefined reference.

**2. No double-precision helpers.**

```bash
arm-none-eabi-nm build/furnace_control/MDCS_HIL.out | grep __aeabi_d
```

Must print nothing. This is the single most useful check: it finds `double` arithmetic that survived the source review, including constants materialised by the compiler.

**3. The ABI is actually hard-float.**

```bash
arm-none-eabi-readelf -A build/furnace_control/MDCS_HIL.out | grep -E 'FP_arch|VFP_args'
```

Expected:

```
Tag_FP_arch: VFPv4-D16
Tag_ABI_VFP_args: VFP registers
```

If `VFP_args` says `Base AAPCS` or is missing, the build is using the soft-float calling convention somewhere, and float arguments are being passed through integer registers.

## The linker map is the real detector

`output.map` records which objects pulled in which library members. If `__aeabi_dmul` appears, the map shows what referenced it. That is faster than bisecting the source, especially when the `double` came from a header macro rather than from anything you wrote.

This is also why the firmware Makefile lists `.c` sources rather than `.h` dependencies: editing a header does not trigger a rebuild, so a change that introduces a `double` can sit unnoticed until a clean build. After any header edit, force it:

```bash
make -B -f Makefile_HIL
```

## Why the constraint is worth keeping

The compensator is shared between SIL and HIL on purpose. If the host build were allowed to use `double`, libm, and `printf("%f")`, the two builds would diverge: gains tuned in simulation would not be the gains that run on the MCU, and the numerical behaviour at the boundary would differ in ways that are hard to attribute.

Float-only keeps a single source of truth. The cost is discipline in four specific places — constants, literals, promotions, and library calls — and the benefit is that "it works in SIL" means something for the firmware.
