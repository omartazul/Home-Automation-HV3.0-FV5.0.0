#!/usr/bin/env python3
"""
Compute uniform-power FAN_DELAY_US for Home-Automation HV3.0 firmware (FV5.0.0).

Uses the power equation P(alpha) = 1 - alpha/pi + sin(2*alpha)/(2*pi) where alpha in [0, pi]
to generate phase-angle delay values that produce linearly-spaced power levels.

Half-cycle duration = 10000 us at 50Hz mains frequency by default.
Use --freq 60 to generate values for 60Hz (half-cycle = 8333.33 us), or
provide an explicit --half-cycle-us value to override.

Note: There is a convenient batch wrapper `generate_fan_delays.bat` in this
folder which runs the script and writes output files into the current working
directory when double-clicked. Use this to easily create `50Hz` and `60Hz` outputs.

SPDX-License-Identifier: CC-BY-NC-4.0
Copyright (c) 2025 Md. Omar Faruk Tazul Islam
"""
import math
import argparse
import os

HALF_CYCLE_US = 10000.0
MIN_DELAY_US = 50
MAX_DELAY_US = 9500

def parse_args():
    p = argparse.ArgumentParser(description='Compute uniform-power FAN_DELAY_US for Home-Automation HV3.0 firmware (FV5.0.0)')
    p.add_argument('--freq', '-f', type=float, default=50.0, help='mains frequency in Hz (default: 50)')
    p.add_argument('--half-cycle-us', type=float, default=None, dest='half_cycle_us', help='explicit half-cycle duration in microseconds (overrides --freq)')
    p.add_argument('--include-off', action='store_true', help='prepend a 0 for index 0 (OFF) in the FAN_DELAY_US array')
    p.add_argument('--out', '-o', type=str, default=None, help='write output file name (placed in current folder; default name generated from frequency)')
    p.add_argument('--generate-all', action='store_true', help='generate output for 50Hz and 60Hz and write to separate files in the current directory')
    return p.parse_args()

N = 9
MIN_P = 0.05 # minimum conduction power for level 1 (5%) to avoid non-latching triac

def default_filename(freq, include_off: bool):
    suffix = '_off' if include_off else ''
    return f"fan_delay_output_{int(freq)}hz{suffix}.txt"


def main(args):
    # compute effective half-cycle in microseconds and human readable frequency
    if args.half_cycle_us is not None:
        half_cycle_us = float(args.half_cycle_us)
        freq_for_name = 1.0 / (2.0 * half_cycle_us * 1e-6)
    else:
        half_cycle_us = 1e6 / (2.0 * float(args.freq))
        freq_for_name = float(args.freq)
    include_off = bool(args.include_off)

    delays = compute_delays(N, MIN_P, half_cycle_us)
    output_text = format_output(delays, half_cycle_us, freq_for_name, include_off)

    # write output to file in current directory
    if args.out:
        out_name = os.path.basename(args.out)
    else:
        out_name = default_filename(freq_for_name, include_off)
    out_path = os.path.join(os.getcwd(), out_name)
    write_output(output_text, out_path)
    # small message to user
    print(f"Wrote: {out_path}")


def compute_delays(levels, min_power, half_cycle_us):
    """Return a list of integer microsecond delays, clamped to MIN_DELAY_US/MAX_DELAY_US.

    - levels: number of discrete fan levels (int)
    - min_power: minimum conduction power for level 1 (float in 0..1)
    - half_cycle_us: half cycle duration in microseconds (float)
    """
    delays = []
    for level in range(1, levels + 1):
        # linear steps between min_power and 1.0
        P = min_power + (level - 1) * ((1.0 - min_power) / (levels - 1))
        alpha = alpha_from_P(P)
        delay_us = alpha * half_cycle_us / math.pi
        # clamp to min/max
        if delay_us < MIN_DELAY_US:
            delay_us = MIN_DELAY_US
        if delay_us > MAX_DELAY_US:
            delay_us = MAX_DELAY_US
        delays.append(round(delay_us))
    return delays


def format_output(delays, half_cycle_us, freq_for_name, include_off=False):
    """Return the text output produced by the original main() logic as a single string.
    This includes: header, indices, C arrays, and the normalized conduction power table.
    """
    content = []
    content.append(format_header(freq_for_name, half_cycle_us, len(delays)))
    content.append(str(delays))
    content.append('')
    content.extend(format_indices(delays))
    content.append('')
    content.append(format_c_array(delays, include_off=include_off))
    content.append('')
    content.extend(format_power_table(delays, half_cycle_us))
    content.append('')
    content.append(format_delay_from_percent_array(half_cycle_us))
    return '\n'.join(content)


def format_header(freq_for_name, half_cycle_us, num_levels):
    return f"Computed FAN_DELAY_US for {int(freq_for_name)}Hz (half-cycle {half_cycle_us} us), levels 1..{num_levels}"


def format_indices(delays):
    lines = []
    for i, d in enumerate(delays):
        lines.append(f"index {i}: {d} us")
    return lines


def format_c_array(delays, include_off=False, name='FAN_DELAY_US'):
    lines = []
    lines.append("\nC array to paste into code:")
    if include_off:
        arr = [0] + delays
    else:
        arr = delays
    arr_len = len(arr)
    arr_items = ", ".join(str(d) for d in arr)
    lines.append(f"const unsigned int {name}[{arr_len}] = {{{arr_items}}};")
    return '\n'.join(lines)


def format_power_table(delays, half_cycle_us):
    lines = []
    lines.append("Normalized conduction power for each delay:")
    for i, d in enumerate(delays):
        alpha = d * math.pi / half_cycle_us
        P = P_of_alpha(alpha)
        lines.append(f"level {i+1} | delay {d} us | P={P:.4f} -> {P*100:.1f}%")
    return lines


def format_delay_from_percent_array(half_cycle_us):
    lines = []
    lines.append("\nC array to paste into code:")
    vals = []
    for P in range(101):
        alpha = alpha_from_P(P / 100.0)
        delay_us = round(alpha * half_cycle_us / math.pi)
        if delay_us < MIN_DELAY_US:
            delay_us = int(MIN_DELAY_US)
        if delay_us > MAX_DELAY_US:
            delay_us = int(MAX_DELAY_US)
        vals.append(delay_us)
    lines.append("const unsigned int DELAY_FROM_PERCENT[101] = {" + ", ".join(str(v) for v in vals) + "};")
    return '\n'.join(lines)


def write_output(text, out_path):
    with open(out_path, 'w', encoding='utf-8') as fh:
        fh.write(text)

def generate_all():
    # Generate 50Hz and 60Hz versions and write to files
    outdir = '.'
    # 50Hz
    a50 = argparse.Namespace(freq=50.0, half_cycle_us=None, include_off=False, out=f"{outdir}/fan_delay_output_50hz.txt", generate_all=False)
    main(a50)
    # 50Hz include-zero
    a50z = argparse.Namespace(freq=50.0, half_cycle_us=None, include_off=True, out=f"{outdir}/fan_delay_output_50hz_off.txt", generate_all=False)
    main(a50z)
    # 60Hz
    a60 = argparse.Namespace(freq=60.0, half_cycle_us=None, include_off=False, out=f"{outdir}/fan_delay_output_60hz.txt", generate_all=False)
    main(a60)
    # 60Hz include-zero
    a60z = argparse.Namespace(freq=60.0, half_cycle_us=None, include_off=True, out=f"{outdir}/fan_delay_output_60hz_off.txt", generate_all=False)
    main(a60z)

def P_of_alpha(alpha):
    return 1.0 - alpha/math.pi + math.sin(2*alpha)/(2.0*math.pi)

# invert using bisection

def alpha_from_P(P):
    lo = 0.0
    hi = math.pi
    for _ in range(50):
        mid = (lo + hi) / 2.0
        p_mid = P_of_alpha(mid)
        if p_mid > P:
            lo = mid
        else:
            hi = mid
    return (lo+hi)/2.0
if __name__ == '__main__':
    args = parse_args()
    if args.generate_all:
        generate_all()
    else:
        main(args)
