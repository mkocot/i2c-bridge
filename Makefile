CXX := clang++

quant: quant.c
	$(CC) -O2 -Ilib/fptc-lib/src -Iinclude -DFPT_WBITS=16 -lm quant.c -o quant
div_test: div_test.c
	$(CC) -O2 -Ilib/fptc-lib/src -Iinclude -DFPT_WBITS=16 -lm div_test.c -o div_test
conv_test: conv_test.c
	$(CC) -O2 -Ilib/fptc-lib/src -Iinclude -DFPT_WBITS=16 -lm conv_test.c -o conv_test
bmp280_bench: bmp280_bench.c lib/bmp280/src/driver_bmp280.c include/sensor.h
	$(CC) -O2 -I. -Ilib/fptc-lib/src -Iinclude -DFPT_WBITS=16 -lm bmp280_bench.c -o bmp280_bench
packet: test.c include/packet.h
	$(CC) -Iinclude -Ilib/fptc-lib/src -DFPT_WBITS=16 test.c -o packet
fsm_test: fsm_test.cpp include/SensorProtocol.hpp include/ProtocolParser.hpp
	$(CXX) -g -O0 -I ../arduino-crc8/include -Iinclude -Ilib/fsm -o fsm_test fsm_test.cpp
wtocol_test: test/wtocol_test.c lib/wtocol/wtocol.c lib/wtocol/wtocol.h
	$(CC) -std=c11 -Wall -Wextra -Wshadow -Wundef -Werror=sign-compare -Ilib/wtocol test/wtocol_test.c lib/wtocol/wtocol.c -lm -o wtocol_test
fsm_fuzz: fsm_test.cpp
	$(CXX) -DFUZZ_ME -g -O1 -fsanitize=fuzzer,address -Ilib/fsm -o fsm_test fsm_test.cpp

# 64-bit division is 1.2 kB of libgcc; it must stay out of the image (ADR-0015)
NM ?= /home/nfinity/.platformio/packages/toolchain-riscv/bin/riscv-wch-elf-nm
ELFS := .pio/build/ch32v006/firmware.elf .pio/build/ch32v003/firmware.elf
helpers:
	@for e in $(ELFS); do \
	  test -f $$e || { echo "$$e missing: pio run -e ... first"; exit 1; }; \
	  bad=$$($(NM) $$e | awk '$$3 ~ /^__(u?div|u?mod)di3$$/ {print $$3}'); \
	  printf '%-36s %s\n' "$$e" "$${bad:-no 64-bit division}"; \
	  test -z "$$bad" || exit 1; \
	done
