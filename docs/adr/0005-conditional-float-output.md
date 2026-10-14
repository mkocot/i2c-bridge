# Sensor drivers ship raw-only; float conversion compiles out

LibDriver sensor drivers expose both raw register counts and float-converted outputs
(temperature_s, humidity_s, ...). The firmware only ever reads the raw counts and converts
them itself in the wrappers under include/ (inline fpt helpers or float math), so every
driver's float conversion is wrapped in `#ifdef DRIVER_*_WITH_FLOAT_OUTPUT` and compiled out.
The macro is defined nowhere in the build: a port that wants float outputs defines it per
driver. The guard also covers the secondary read helpers (read_temperature, read_humidity,
continuous_read, read_low_power, *_convert_to_data), not just the combined reads.

Consequences: with the macro undefined the guarded output parameters are left unwritten, so
the wrappers always pass NULL for them; each guarded write is paired with an `#else (void)param;`
cast. The guard style is uniformly `#ifdef` (no value check). Two drivers are deliberate
exceptions: bmp280 keeps its own `DRIVER_BMP280_WITH_INT` output-type switch because
include/bmp280.h consumes its temperature/pressure outputs via fl2fpt, and bme280 is untouched
(it is not wired into the build). dht11's humidity output is an integer copy with no float
math, so it stays unguarded.
