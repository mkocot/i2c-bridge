# SHT3x and SHTC3 single-shot reads use no-clock-stretch commands

The firmware previously triggered SHT35 and SHTC3 single-shot measurements with the
clock-stretching command variants (SHT35 0x2C06, SHTC3 0x7CA2 / 0x5C24), leaving SCL
stretched for the whole conversion while the wire layer read with a 12 ms internal
delay. The CH32V00x I2C glue cannot safely race a stretched line (same constraint
behind ADR-0001), so the no-clock-stretch variants (SHT35 0x2400, SHTC3 0x7866 /
0x58E0) are used instead; the wire layer keeps the single-transaction delayed read.

The datasheet maximum conversion time is 15.5 ms (SHT35 high repeatability) and
12.1 ms (SHTC3 normal), both longer than the 12 ms wire delay. The delay was kept
unchanged deliberately and the decision verified empirically: 51 consecutive loop
cycles with the new commands produced zero read errors and a stable sensor set.
If intermittent read failures appear later, raise the wire delay above the datasheet
maximum; do not revert to clock stretching.

The dead 0x44/0x70 special cases in libdriver_iic_addr16_read were removed at the
same time; SHT35/SHTC3 delay handling now lives only in the sensor glue wrappers.
