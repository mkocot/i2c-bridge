# HTU21D measurements use no-hold commands in single transactions

The firmware originally triggered HTU21D measurements with hold-master commands (0xE3/0xE5)
through htu_read_with_scl, which waited only 20 ms between command and read while the chip
can hold SCL low for up to 50 ms (14-bit temp): reads failed intermittently ("htu21d:
read failed") and the sensor appeared and disappeared across loops. The printed
-46.850 C / -6.0 %RH came from a separate bug: sensor_htu21d_obtain converted the
never-written globals tmp_temperature / tmp_humidity_f instead of the raw values it had
just read.

The glue (include/htuxx.h) now keeps every HTU21D exchange in a single transaction:

- the serial-number read uses the 16-bit register read (FA 0F + repeated start + 8
  bytes). A separate read transaction after the command write is NACKed by the sensor
  (observed as I2C_ERR_NACK), so the two-transaction no-hold shape is unusable on this
  MCU;
- the measurement read uses the delayed single-transaction read with the no-hold
  commands translated in (0xE3/0xE5 are swapped for 0xF3/0xF5) and waits of 60 ms
  (temp) / 25 ms (RH), the datasheet maximum conversion time (50 ms / 16 ms) plus
  margin. The sensor never stretches SCL, so the transaction cannot race its hold.

The driver still runs with the hold-master mode flag, but only as a routing choice: the
wire traffic is no-hold. obtain converts tmp_raw_temperature16 / tmp_raw_humidity16.
