#!/usr/bin/env python3
"""
Smoke test for the new -continuous mode.

It runs two rounds against the chibicc_main_only_block benchmark:
1. An add-only diff -> should fall back to full Andersen (TotalNode = 47374).
2. A delete-only diff on the same bitcode -> should run Andersen_INC (TotalNode = 47368).
"""

import subprocess
import os
import tempfile

WPA = "/Users/jiayi/SILVA-llvm15/SILVA-llvm15/build/bin/wpa"
ROOT = "/Users/jiayi/SILVA-llvm15"

def extract_totalnode(output):
    """Extract the SVFG TotalNode from wpa output."""
    in_svfg = False
    for line in output.splitlines():
        if "****SVFG Statistics****" in line:
            in_svfg = True
        elif "#######################################################" in line and in_svfg:
            in_svfg = False
        elif in_svfg and line.startswith("TotalNode"):
            return int(line.split()[1])
    return None

def main():
    os.chdir(ROOT)

    # Round 1: add-only diff (after -> before of the block-only variant).
    # Intentionally omit -sourcediff to verify auto-generation.
    round1 = (
        "-beforecpp ./chibicc_main_only_block_work/after "
        "-aftercpp ./chibicc_main_only_block_work/before "
        "./chibicc_main_only_block_work/chibicc_before.bc"
    )
    # Round 2: delete-only diff (before -> after of the block-only variant).
    # Intentionally omit -sourcediff to verify auto-generation.
    round2 = (
        "-beforecpp ./chibicc_main_only_block_work/before "
        "-aftercpp ./chibicc_main_only_block_work/after "
        "./chibicc_main_only_block_work/chibicc_before.bc"
    )

    input_text = f"{round1}\n{round2}\nquit\n"

    proc = subprocess.run(
        [WPA, "-continuous", "-svfg", "-relapath"],
        input=input_text,
        capture_output=True,
        text=True,
        timeout=300,
    )

    combined = proc.stdout + proc.stderr
    if proc.returncode != 0:
        print("wpa exited with code", proc.returncode)
        print(combined[-2000:])
        return 1

    # Collect TotalNode values from each round's SVFG stats.
    totalnodes = []
    in_svfg = False
    for line in combined.splitlines():
        if "****SVFG Statistics****" in line:
            in_svfg = True
        elif "#######################################################" in line and in_svfg:
            in_svfg = False
        elif in_svfg and line.startswith("TotalNode"):
            totalnodes.append(int(line.split()[1]))

    print("Round 1 (add-only -> full Andersen) TotalNode:", totalnodes[0] if len(totalnodes) > 0 else "N/A")
    print("Round 2 (delete-only -> Andersen_INC) TotalNode:", totalnodes[1] if len(totalnodes) > 1 else "N/A")

    ok = True
    if len(totalnodes) < 2:
        print("ERROR: did not find two SVFG TotalNode values")
        ok = False
    else:
        if totalnodes[0] != 47374:
            print(f"ERROR: round 1 expected 47374, got {totalnodes[0]}")
            ok = False
        if totalnodes[1] != 47368:
            print(f"ERROR: round 2 expected 47368, got {totalnodes[1]}")
            ok = False

    if ok:
        print("PASS: -continuous mode correctly switches between full and incremental analysis.")
        return 0
    else:
        print("FAIL")
        return 1

if __name__ == "__main__":
    exit(main())
