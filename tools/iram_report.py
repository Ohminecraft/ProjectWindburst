#!/usr/bin/env python3
"""What WB_FAST (src/config/fast_code.h) really did to a firmware build.

usage: python3 tools/iram_report.py <firmware.elf> [objdump]
       (PlatformIO: .pio/build/<env>/firmware.elf; objdump defaults to the xtensa-esp32s3-elf-objdump of ~/.platformio)

Prints  1. the functions that sit in internal RAM, with their size, and the total
        2. for every one of them, the functions it CALLS that are still in flash - each of those is fetched through the
           instruction cache in every control cycle (library code cannot be moved from here; our own functions can: mark them)
Needs: pip install pyelftools"""
import sys, os, re, glob, subprocess, struct
from elftools.elf.elffile import ELFFile

elf_path = sys.argv[1]
objdump = sys.argv[2] if len(sys.argv) > 2 else (glob.glob(os.path.expanduser('~/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump')) or ['xtensa-esp32s3-elf-objdump'])[0]
IRAM = (0x40370000, 0x403E0000)                    # ESP32-S3 internal RAM as code
FLASH = (0x42000000, 0x44000000)                   # code in flash, seen through the instruction cache (0x4000_0000: the chip's ROM - always there)
def in_iram(a): return IRAM[0] <= a < IRAM[1]
def in_flash(a): return FLASH[0] <= a < FLASH[1]

f = open(elf_path, 'rb'); elf = ELFFile(f)
secs = [(s['sh_addr'], s['sh_addr'] + s['sh_size'], s) for s in elf.iter_sections() if s['sh_addr'] and s['sh_type'] == 'SHT_PROGBITS']
def word(addr):
    for a, b, s in secs:
        if a <= addr < b - 3: return struct.unpack_from('<I', s.data(), addr - a)[0]
    return None
funcs = {}
for s in elf.get_section_by_name('.symtab').iter_symbols():
    if s['st_info']['type'] == 'STT_FUNC' and s['st_size']: funcs.setdefault(s['st_value'], (s.name, s['st_size']))
def demangle(names):
    try: out = subprocess.run([objdump.replace('objdump', 'c++filt')], input='\n'.join(names) + '\n', capture_output=True, text=True).stdout.splitlines()     # (stdin: thousands of names do not fit on a Windows command line)
    except OSError: return names
    return out if len(out) == len(names) else names
starts = sorted(funcs)
def owner(addr):
    import bisect
    i = bisect.bisect_right(starts, addr) - 1
    if i >= 0 and addr < starts[i] + funcs[starts[i]][1]: return starts[i]
    return None

ours = re.compile(r'FlightApp|FlightCore|FlightTask|\bfc::|\bmotor::|DshotHw|RmtDirectPort|IMUAccGyro|AccGyro|SPIBus|AgSpiRegBus|\bbb::|\bapp::|RtosSync')
names = demangle([funcs[a][0] for a in starts]); nice = dict(zip(starts, names))
ram = [a for a in starts if in_iram(a) and ours.search(nice[a])]
total = sum(funcs[a][1] for a in ram)
print('== %d of the firmware\'s own functions in internal RAM, %d bytes (all code in RAM: %d bytes)' % (len(ram), total, elf.get_section_by_name('.iram0.text')['sh_size']))
for a in sorted(ram, key=lambda a: -funcs[a][1]): print('  %5d  %s' % (funcs[a][1], nice[a]))

# ---- calls that leave RAM
# Read from the machine code itself (objdump prints some functions - GCC's ".isra" clones - as data, and their calls would be
# missing). Xtensa: an instruction is 2 bytes when the low nibble of its first byte is 8..13, otherwise 3. Three kinds matter:
#   l32r  aT, literal     xxxxT1   register <- 32-bit constant stored before the function (the address of a function, usually)
#   callN target          op0 = 5  direct call (the linker uses it when the target is near: RAM to RAM)
#   callxN aS             0000S0 | 0xC0 | N << 4    call the address in a register (RAM to flash and to ROM are always this)
iram_sec = elf.get_section_by_name('.iram0.text'); iram_base = iram_sec['sh_addr']; iram_data = iram_sec.data()
calls = {}; indirect = {}; sites = {}
for fn in ram:
    pc = fn; end = fn + funcs[fn][1]; lit = {}          # register -> (constant, instructions ago)
    while pc < end:
        b0 = iram_data[pc - iram_base]; op0 = b0 & 0x0F; n = 2 if 8 <= op0 <= 13 else 3
        if pc + n > end: break
        for r in list(lit):
            lit[r] = (lit[r][0], lit[r][1] + 1)
            if lit[r][1] > 12: del lit[r]                # (the compiler loads the address right before it sets up the arguments)
        if n == 3:
            w = b0 | iram_data[pc - iram_base + 1] << 8 | iram_data[pc - iram_base + 2] << 16
            if op0 == 1:                                 # l32r
                addr = ((pc + 3) & ~3) - ((0x10000 - (w >> 8)) << 2)
                v = word(addr)
                if v is not None: lit[b0 >> 4] = (v, 0)
            elif op0 == 5:                               # call0/4/8/12
                off = w >> 6
                if off & 0x20000: off -= 0x40000
                t = (pc & ~3) + 4 + (off << 2)
                if t in funcs: calls.setdefault(fn, set()).add(t); sites[t] = sites.get(t, 0) + 1      # (a direct call reaches other RAM code only; anything else was no call)
                lit = {}
            elif (w & 0xFFF0CF) == 0x0000C0:             # callx0/4/8/12
                reg = (w >> 8) & 0xF
                if reg in lit and (in_iram(lit[reg][0]) or in_flash(lit[reg][0]) or 0x40000000 <= lit[reg][0] < 0x40060000):     # (a constant that is no code address was not this call's target)
                    t = lit[reg][0]; calls.setdefault(fn, set()).add(t); sites[t] = sites.get(t, 0) + 1
                else: indirect[fn] = indirect.get(fn, 0) + 1
                lit = {}
        pc += n
out = {}; rom = {}
for fn, ts in calls.items():
    for t in ts:
        o = owner(t)
        if o is not None and in_flash(o): out.setdefault(o, set()).add(fn)
        elif not in_iram(t) and not in_flash(t): rom[t] = rom.get(t, 0) + 1
print('\n== called from those, but still in flash: %d functions, %d bytes' % (len(out), sum(funcs[o][1] for o in out)))
for o in sorted(out, key=lambda o: (not ours.search(nice[o]), -funcs[o][1])):
    who = sorted(nice[c].split('(')[0] for c in out[o])
    print('  %5d  %-58s x%-3d <- %s%s' % (funcs[o][1], nice[o][:58], sites.get(o, 0), ', '.join(who[:3]), ' ...' if len(who) > 3 else ''))
romsym = {}
for sym in elf.get_section_by_name('.symtab').iter_symbols():
    if sym['st_value'] in rom and sym.name: romsym.setdefault(sym['st_value'], sym.name)
if rom: print('\n== ROM routines called (no cache involved; the number of places they are called from): ' + ', '.join('%s x%d' % (romsym.get(t, hex(t)), sites.get(t, 0)) for t in sorted(rom, key=lambda t: -sites.get(t, 0))))
# double precision is done in software on this chip (its FPU is single precision): every such call in the cycle is worth a look
dbl = {}
for fn, ts in calls.items():
    for t in ts:
        nm = romsym.get(t, '')
        if re.search(r'df[0-9]?$|^__fix(uns)?df|^__float(un)?(si|di)df|^__extendsfdf2|^__truncdfsf2', nm): dbl.setdefault(fn, set()).add(nm)
if dbl:
    print('\n== double-precision arithmetic (software, the FPU is single precision) is used by:')
    for fn in sorted(dbl, key=lambda a: nice[a]): print('    %-70s %s' % (nice[fn][:70], ' '.join(sorted(dbl[fn]))))
if indirect: print('\n(%d calls through a pointer - virtual functions, callbacks - are not followed)' % sum(indirect.values()))
