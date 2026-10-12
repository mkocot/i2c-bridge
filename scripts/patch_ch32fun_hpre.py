#!/usr/bin/env python3
# Patch ch32v003fun SystemInit so a CH32V00x target below 24MHz (PLL off) keeps
# the HPRE divisor instead of running at full HSI.
#
# Why: CH32V00x HSI is 24MHz. The upstream no-PLL/HSI branch writes
#     RCC->CFGR0 = RCC_HPRE_DIV1;  // 0x0
# On CH32V00x HPRE[3:0]=0000 means "prescaler off" (CH32V00XRM RCC_CFGR0), so
# the core immediately runs at 24MHz while FLASH_ACTLR latency was already set
# (earlier in SystemInit) for FUNCONF_SYSTEM_CORE_CLOCK (<=15MHz -> 0 wait).
# 24MHz @ 0 wait-state fetches corrupt, the QingKe core resets, main() loops.
#
# Fix: set HPRE to divide HSI down to FUNCONF_SYSTEM_CORE_CLOCK using the
# CH32V00x HPRE encoding (0x0..0x7 = /1../8). Valid for 24MHz..3MHz.
# Idempotent: re-running on an already-patched file is a no-op.
from pathlib import Path

MARKER = "pio-hpre-patch"
ANCHOR = "RCC->CFGR0 = RCC_HPRE_DIV1;"
NL = chr(10)
T = chr(9)

def patch(path):
    text = path.read_text()
    if MARKER in text:
        return "already patched"
    lines = text.splitlines(keepends=True)
    hits = [i for i, l in enumerate(lines) if l.lstrip().startswith(ANCHOR)]
    if len(hits) != 1:
        raise SystemExit(
            "ch32fun_hpre_patch: expected exactly one '" + ANCHOR +
            "' in " + str(path) + ", found " + str(len(hits)) +
            " - upstream ch32fun.c changed; re-derive the patch"
        )
    i = hits[0]
    indent = lines[i][: len(lines[i]) - len(lines[i].lstrip())]
    body = NL.join([
        indent + "#if defined(CH32V00x)",
        indent + T + "RCC->CFGR0 = ((HSI_VALUE / FUNCONF_SYSTEM_CORE_CLOCK) - 1) << 4; // " + MARKER + ": HPRE[3:0] 0x0..0x7 = /1../8",
        indent + "#else",
        indent + T + "RCC->CFGR0 = RCC_HPRE_DIV1;",
        indent + "#endif",
    ]) + NL
    lines[i] = body
    path.write_text("".join(lines))
    return "patched"

# SCons entry (PlatformIO calls this module with Import() in scope).
try:
    Import("env")  # noqa: F821 - SCons-injected builtin
    target = Path(env.PioPlatform().get_package_dir("framework-ch32v003fun")) / "ch32fun" / "ch32fun.c"
except NameError:
    import sys
    if len(sys.argv) != 2:
        raise SystemExit("usage: patch_ch32fun_hpre.py <path-to-ch32fun.c>")
    target = Path(sys.argv[1])

print("ch32fun_hpre_patch: " + patch(target) + " -> " + str(target))
