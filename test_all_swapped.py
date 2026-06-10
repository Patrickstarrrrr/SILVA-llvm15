#!/usr/bin/env python3
import subprocess
import os
import sys

WPA = "/Users/jiayi/SILVA-llvm15/SILVA-llvm15/build/bin/wpa"
INC_TEST_DIR = "/Users/jiayi/SILVA-llvm15/SILVA-llvm15/inc-test"

def extract_stats(output):
    stats = {}
    for line in output.splitlines():
        line = line.strip()
        for key in ["MemRegions", "TotalNode", "TotalEdge", "FormalIn", "FormalOut", "ActualIn", "ActualOut"]:
            if line.startswith(key):
                parts = line.split()
                if len(parts) >= 2:
                    stats[key] = parts[1]
    return stats

def run_wpa(args):
    cmd = [WPA] + args
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    return extract_stats(result.stdout + result.stderr)

def run_test(test_dir):
    print(f"\n{'='*50}")
    print(f"Test: {test_dir}")
    print(f"{'='*50}")
    
    os.chdir(test_dir)
    
    # Generate swapped diff
    subprocess.run(["diff", "-N", "-r", "-X", "DiffPattern.txt", "after", "before"], 
                   stdout=open("sourceDiff_swapped.txt", "w"), stderr=subprocess.DEVNULL)
    
    full_old = run_wpa(["-ander", "-svfg", "old.bc"])
    full_new = run_wpa(["-ander", "-svfg", "new.bc"])
    inc_std = run_wpa(["-iander", "-svfg", "-irdiff", "-is-new", "-relapath",
                       "-sourcediff", "./sourcediff.txt",
                       "-beforecpp", "./before", "-aftercpp", "./after", "new.bc"])
    inc_swap = run_wpa(["-iander", "-svfg", "-irdiff", "-is-new", "-relapath",
                        "-sourcediff", "./sourceDiff_swapped.txt",
                        "-beforecpp", "./after", "-aftercpp", "./before", "old.bc"])
    
    print(f"Full old: {full_old}")
    print(f"Full new: {full_new}")
    print(f"Inc std:  {inc_std}")
    print(f"Inc swap: {inc_swap}")
    
    std_ok = (inc_std == full_new)
    swap_ok = (inc_swap == full_old)
    
    print(f"Std match new: {std_ok}")
    print(f"Swap match old: {swap_ok}")
    
    os.chdir("../..")
    return std_ok, swap_ok

os.chdir("/Users/jiayi/SILVA-llvm15/SILVA-llvm15")
all_std_ok = True
all_swap_ok = True
for d in sorted(os.listdir(INC_TEST_DIR)):
    if d.startswith("test"):
        std_ok, swap_ok = run_test(os.path.join(INC_TEST_DIR, d))
        all_std_ok &= std_ok
        all_swap_ok &= swap_ok

print(f"\n{'='*50}")
print(f"Overall: std_ok={all_std_ok}, swap_ok={all_swap_ok}")
