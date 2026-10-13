# Bridge SDI printf into the PlatformIO Serial Monitor

The task (TASK.md) wants the CH32V00x SDI printf stream in the PlatformIO
Serial Monitor. The ch32v platform already ships an `sdi_printf_monitor`
target for minichlink envs, but it launches `minichlink -T` as a raw terminal:
the output is visible, yet the Serial Monitor UI (pio device monitor) and its
VS Code button know nothing about it.

Alternatives considered:

- wlink `sdi-print enable` virtual serial port: not feasible (probe
  contention, tooling mismatch; see below).
- The WCH-Link CDC port (/dev/ttyACM0): carries the target UART1 passthrough,
  not SDI printf - a 921600 read of it is silent. Dead route.
- Bridge `minichlink -T` to a pty: scripts/sdi_monitor.py creates a pty pair,
  gives minichlink the master as its stdin/stdout, then runs
  `pio device monitor -p <slave>` in the foreground. The kernel pty pairing
  carries target logs to the monitor and keystrokes back to the target debug
  console; the wrapper tears both processes down when the monitor exits.

Why minichlink -T rather than wlink: the repo's ch32v003/ch32v006 envs already
use minichlink for upload and its `-T` terminal mode is proven to stream the
target boot logs; wlink's SDI virtual serial port would need a separate
daemon plus its own reset/attach handling, and the two tools contend for the
probe. The CDC route fails because the WCH-Link's built-in CDC is a UART1
passthrough, unrelated to SDI.

Consequences: the probe is held only while monitoring (a second tool session
fails with "Resource busy" until the monitor exits); the bridge is a wrapper
script plus one VS Code task, so it stays out of the build. The platform's
raw `sdi_printf_monitor` target remains available.
