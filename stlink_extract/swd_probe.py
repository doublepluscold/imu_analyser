#!/usr/bin/env python3
"""
swd_probe.py - non-destructive SWD probe for the unknown STM32H743 IMU module.

READ-ONLY. Never writes flash, never touches option bytes / RDP.

Subcommands:
  rdp                         read protection level (gates everything)
  pins                        dump GPIO config -> live pinout (which pin = which peripheral)
  locate [--addr A --size N]  find RAM words that change when you move the board (= IMU data)
  watch  ADDR [ADDR ...]      print given RAM addresses live as float32 / int32

Connect mode is "attach": it hooks the running core WITHOUT reset or halt, so the
firmware keeps producing IMU data while you read it. If attach fails, the firmware
has disabled SWD (see the plan .md) and this whole route is out.

Deps:  pip install pyocd    (target stm32h743xx is built in; if not:
       pyocd pack install stm32h743vitx )
Run:   python3 swd_probe.py rdp
"""

import sys
import time
import struct
import argparse

try:
    from pyocd.core.helpers import ConnectHelper
except ImportError:
    sys.exit("pyocd not found:  pip install pyocd")

TARGET = "stm32h743xx"

# --- H743 register map ---
FLASH_OPTSR_CUR = 0x5200201C          # RDP in bits [15:8]
RCC_AHB4ENR     = 0x580244E0          # GPIOA..K clock enables in bits [10:0]
GPIO_BASE       = 0x58020000          # GPIOA; each port +0x400
GPIO_STRIDE     = 0x400
PORTS           = "ABCDEFGHIJK"
REG_MODER, REG_OTYPER, REG_PUPDR, REG_AFRL, REG_AFRH = 0x00, 0x04, 0x0C, 0x20, 0x24

# AF-number -> likely peripheral family on H743 (first-pass hint; confirm exact
# signal in the datasheet "Alternate function mapping" table).
AF_HINT = {
    0:  "SYS (SWD/MCO/RTC/TRACE)",
    1:  "TIM1/2, LPTIM1, HRTIM",
    2:  "TIM3/4/5/12, SAI1",
    3:  "TIM8, LPTIMx, LPUART",
    4:  "I2C1-4, USART1, TIM15",
    5:  "SPI1-6",
    6:  "SPI2/3, SAI1, UART4, I2C4",
    7:  "USART1-3, SPI2/3/6, UART7",
    8:  "UART4/5/7/8, LPUART1, SPI6, SAI2",
    9:  "FDCAN1/2, QUADSPI, TIM13/14, LTDC, FMC",
    10: "SAI2/4, QUADSPI, SDMMC2, USB, LTDC",
}


def connect(freq=100000, mode="attach"):
    session = ConnectHelper.session_with_chosen_probe(
        target_override=TARGET, connect_mode=mode,
        options={"frequency": freq, "resume_on_disconnect": False})
    session.open()
    return session


def cmd_rdp(target):
    v = target.read32(FLASH_OPTSR_CUR)
    rdp = (v >> 8) & 0xFF
    level = {0xAA: "0 (unprotected - flash readable)",
             0xCC: "2 (locked - SWD dead, irreversible)"}.get(rdp, "1 (flash locked; RAM/regs still readable)")
    print(f"FLASH_OPTSR_CUR = 0x{v:08X}   RDP byte = 0x{rdp:02X}")
    print(f"RDP level: {level}")
    if rdp == 0xAA:
        print("  -> you can dump firmware with STM32CubeProgrammer (-r 0x08000000 0x200000 fw.bin)")
    elif rdp == 0xCC:
        print("  -> SWD extraction not possible; fall back to logic-analyzer SPI sniff")
    else:
        print("  -> firmware dump blocked, but 'pins' and 'locate' below still work. Do NOT lower RDP (mass-erase).")


def decode_port(target, pi):
    base = GPIO_BASE + pi * GPIO_STRIDE
    moder = target.read32(base + REG_MODER)
    afrl  = target.read32(base + REG_AFRL)
    afrh  = target.read32(base + REG_AFRH)
    rows = []
    for pin in range(16):
        mode = (moder >> (2 * pin)) & 3
        if mode == 2:  # alternate function
            af = ((afrl if pin < 8 else afrh) >> (4 * (pin % 8))) & 0xF
            hint = AF_HINT.get(af, "see datasheet AF table")
            rows.append((pin, f"AF{af}", hint))
        elif mode == 1:
            rows.append((pin, "OUT", "GPIO output (LED / EN / CS-bitbang?)"))
    return rows


def cmd_pins(target):
    clk = target.read32(RCC_AHB4ENR)
    print("Ports with clock enabled:",
          " ".join(PORTS[i] for i in range(len(PORTS)) if clk & (1 << i)) or "(none)")
    print("\nPins in AF / output mode (unused inputs & analog hidden):\n")
    print(f"{'PIN':<6}{'MODE':<6}LIKELY FUNCTION")
    for pi, port in enumerate(PORTS):
        if not (clk & (1 << pi)):
            continue
        for pin, mode, hint in decode_port(target, pi):
            print(f"P{port}{pin:<5}{mode:<6}{hint}")
    print("\nLook for: a block of AF5 pins (SPI to the SCH1633), AF7/AF8 pins (USART to")
    print("ADM3232E and to GPS), and OUT pins near them (chip-selects). Confirm exact")
    print("signals against the H743 datasheet alternate-function table.")


def read_block(target, addr, nwords):
    return target.read_memory_block32(addr, nwords)


def cmd_locate(target, addr, size, secs):
    nwords = size // 4
    def capture(label, duration):
        print(f"  {label}: capturing {duration:.0f}s ...", flush=True)
        snaps = []
        t_end = time.time() + duration
        while time.time() < t_end:
            snaps.append(read_block(target, addr, nwords))
            time.sleep(0.02)
        return snaps

    def variance(snaps):
        n = len(snaps)
        out = [0.0] * nwords
        for w in range(nwords):
            col = [s[w] for s in snaps]
            m = sum(col) / n
            out[w] = sum((x - m) ** 2 for x in col) / n
        return out

    print(f"Scanning 0x{addr:08X} .. 0x{addr+size:08X} ({nwords} words)")
    input("  Hold the board STILL, then press Enter...")
    still = variance(capture("still", secs))
    input("  Now ROTATE / SHAKE the board continuously, then press Enter...")
    move = variance(capture("move", secs))

    hits = []
    for w in range(nwords):
        if move[w] > 0 and move[w] > 20 * (still[w] + 1):
            hits.append((move[w], addr + 4 * w))
    hits.sort(reverse=True)
    print(f"\nMotion-correlated words (candidate IMU data): {len(hits)}")
    for var, a in hits[:24]:
        raw = read_block(target, a, 1)[0]
        f = struct.unpack("<f", struct.pack("<I", raw))[0]
        print(f"  0x{a:08X}  move_var={var:>12.0f}  raw=0x{raw:08X}  float={f: .4f}")
    if hits:
        addrs = " ".join(f"0x{a:08X}" for _, a in hits[:6])
        print(f"\nWatch the top ones live:\n  python3 {sys.argv[0]} watch {addrs}")
    else:
        print("  Nothing correlated. Try another region (--addr 0x24000000 for AXI SRAM),")
        print("  a bigger --size, or confirm the core is running (attach succeeded).")


def cmd_watch(target, addrs):
    print("addr        int32         float32     (Ctrl+C to stop)")
    try:
        while True:
            line = []
            for a in addrs:
                raw = read_block(target, a, 1)[0]
                f = struct.unpack("<f", struct.pack("<I", raw))[0]
                i = struct.unpack("<i", struct.pack("<I", raw))[0]
                line.append(f"0x{a:08X} {i:>11} {f: 10.3f}")
            print("  |  ".join(line), flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped.")


def parse_freq(s):
    s = s.strip().lower()
    mult = 1
    if s.endswith("k"):
        mult, s = 1000, s[:-1]
    elif s.endswith("m"):
        mult, s = 1_000_000, s[:-1]
    return int(float(s) * mult)


def main():
    # --freq / --connect-mode are accepted both before and after the subcommand.
    # SUPPRESS defaults keep a value given in one position from being reset by the other.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--freq", type=parse_freq, default=argparse.SUPPRESS,
                        help="SWD clock, e.g. 100k / 500k / 1M (default 100k; SWDIO has a 10k series R)")
    common.add_argument("--connect-mode", default=argparse.SUPPRESS,
                        choices=["attach", "under-reset", "halt"],
                        help="attach = hook running core (needed for pins/locate/watch)")

    p = argparse.ArgumentParser(description="Non-destructive SWD probe (read-only).",
                                parents=[common])
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("rdp", parents=[common])
    sub.add_parser("pins", parents=[common])
    lp = sub.add_parser("locate", parents=[common])
    lp.add_argument("--addr", type=lambda x: int(x, 0), default=0x20000000)  # DTCM
    lp.add_argument("--size", type=lambda x: int(x, 0), default=0x4000)      # 16 KB window
    lp.add_argument("--secs", type=float, default=4.0)
    wp = sub.add_parser("watch", parents=[common])
    wp.add_argument("addrs", nargs="+", type=lambda x: int(x, 0))
    args = p.parse_args()

    freq = getattr(args, "freq", 100000)
    mode = getattr(args, "connect_mode", "attach")

    if args.cmd in ("pins", "locate", "watch") and mode != "attach":
        print("note: pins/locate/watch need --connect-mode attach to see the running "
              "firmware. under-reset halts before the firmware configures anything.\n")

    session = connect(freq, mode)
    try:
        target = session.target
        if args.cmd == "rdp":
            cmd_rdp(target)
        elif args.cmd == "pins":
            cmd_pins(target)
        elif args.cmd == "locate":
            cmd_locate(target, args.addr, args.size, args.secs)
        elif args.cmd == "watch":
            cmd_watch(target, args.addrs)
    finally:
        session.close()


if __name__ == "__main__":
    main()
