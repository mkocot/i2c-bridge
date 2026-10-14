Build and firmware notes

Firmware for CH32V003/CH32V006 boards, plus host-side checks.

Build systems

    PlatformIO (platformio.ini): firmware, embedded and native tests
    platformio_native.ini: standalone ini with only the native-test env (src_dir=src)
    Makefile: host-only checks (quant, packet, fsm_test, fsm_fuzz)
    do.sh: one-off check: gcc -Ilib/fptc-lib/src -Ilib test.c

PlatformIO environments

    ch32v003: genericCH32V003F4P6, framework ch32v003fun, upload minichlink
        flags: -DSDI_PRINT=1 -DDRIVER_BMP280_WITH_INT -DFPT_WBITS=16 -Werror=format -Werror=incompatible-pointer-types
    ch32v006: genericCH32V006F8P6, framework ch32v003fun, upload minichlink
        flags: -DCH32V00x -DFR_CORE_ONLY=1 plus strict -Werror set (format, overflow, undef, incompatible-pointer-types)
    ch32v006x: genericCH32V006F8P6, framework noneos-sdk
        upload: custom command wlink flash $UPLOAD_FLAGS $SOURCE
    LGT8F328P: AVR board, framework arduino, 16MHz
        toolchain: ../../avr-gcc-13.2.0-x64-linux (local file package)
    native-test: host build of src/, test_ignore test_embedded

Common flags: -Wno-unused-function, -DSDI_PRINT=1, -DFPT_WBITS=16.
Defines select code paths via #if: -DCH32V00x, -DCH32V003, -DFPT_WBITS, -DFR_CORE_ONLY, -DDRIVER_BMP280_WITH_INT.

Dependencies

    lib_deps: file:///home/nfinity/git/arduino-crc8, github.com/mkocot/aht20, aht30, bmp280 (branch feat/configurable-debug)
    git submodules in lib/: sensor drivers (aht20, aht30, bme280, bmp280, dht11, hdc1080, htu21d, htu31d, max31865, mcp9808, pca9548a, sht35, sht4x, shtc3, si7021),
        plus CH32V00x_lib_i2c, fptc-lib, fr_math, fsm

Framework

    ch32v003fun is a PlatformIO package, not vendored in the repo:
    /home/nfinity/.platformio/packages/framework-ch32v003fun/
        ch32fun.h - main header, included by src/main.c
        extralibs/ch32v003_GPIO_branchless.h - GPIO_pinMode, GPIO_port_enable, GPIO_digitalWrite (branchless macros, pin encodes port<<4|pin)
    Building outside PlatformIO requires this package on the include path.

Tests

    pio test -e native-test - host build of src/
    test/test_arena - arena allocator tests
    test/test_packet - packet/CRC8 tests
    make quant | make packet | make fsm_test | make fsm_fuzz - host-only

Flashing

    ch32v003, ch32v006: minichlink (pio run -t upload)
    ch32v006x: wlink flash (custom upload_command)

Monitoring SDI printf

    ch32v003/ch32v006 printf goes over the SWIO debug pin (SDI), not a UART:
    the debug probe decodes it and the host side is minichlink -T.
        Raw terminal: pio run -t sdi_printf_monitor -e ch32v006
        Serial Monitor UI: VS Code task "PlatformIO: Monitor SDI Printf (ch32fun)"
            (asks for an environment), or directly: python3 scripts/sdi_monitor.py ch32v006
        The wrapper script bridges minichlink -T to a pty, so the PlatformIO
            Serial Monitor (pio device monitor) shows the SDI printf stream,
            with keystrokes forwarded to the target debug console.
    The probe is exclusive: only one minichlink/wlink session at a time. A second
    session fails with "Resource busy"; stop the monitor before re-flashing.

Local state and gotchas

    pio is not on PATH on this machine; previous build artifacts live in .pio/build/{ch32v003,ch32v006,ch32v006x,native-test}
    -Werror=format turns printf format mismatches into build errors
    src/ holds stale backup copies (main copy, main.c.x, main.c.working-spi); src/main.c is the firmware

Hardware notes

    HC-12 radio: USART1 TX on PD5 (SIMPLEX mode), SET pin on PD6, open-drain
    HC-12 SET: low = AT command mode, high = passthrough; the module has an internal 10k pull-up on SET,
        so open-drain write 1 releases the pin and the module pull-up pulls it high
    TCA9548A I2C mux reset: PC4, open-drain, default high
    HC-12 wiring (CABLES.md): GND TX SET V+ (from back)
