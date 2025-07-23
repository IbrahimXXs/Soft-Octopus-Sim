#!/usr/bin/env python3
"""
generate_spatials.py

Prompt for B and R (e.g. B=2, R=2) and print out 9 <spatial> XML blocks
with the correct site‑pairs and range minima.
"""

def generate_spatials(B: int, R: int):
    # mapping from spatial index → minimum range
    range_mins = {
        1: 0.003,
        2: 0.009,
        3: 0.009,
        4: 0.009,
        5: 0.003,
        6: 0.009,
        7: 0.009,
        8: 0.009,
        9: 0.003,
    }

    for i in range(1, 10):
        b_idx = (i - 1) // 3 + 1   # 1,1,1,2,2,2,3,3,3
        s_idx = (i - 1) % 3 + 1    # 1,2,3,1,2,3,1,2,3

        name   = f"T_B{B}S1_{i}_R{R}"
        rmin   = range_mins[i]
        site_b = f"site{b_idx}_B{B}_R{R}"
        site_s = f"site{s_idx}_S1_R{R}"

        print(f'<spatial name="{name}" width="0.0004" frictionloss="0.1" '
              f'rgba=".95 .3 .3 1" limited="true" range="{rmin:.3f} 0.1">')
        print(f'    <site site="{site_b}"/>')
        print(f'    <site site="{site_s}"/>')
        print('</spatial>\n')

if __name__ == "__main__":
    try:
        B = int(input("Enter B number (e.g. 2): ").strip())
        R = int(input("Enter R number (e.g. 2): ").strip())
    except ValueError:
        print("❌ Please enter integer values for B and R.")
        exit(1)

    print(f"\n<!-- Generating spatials for B={B}, R={R} -->\n")
    generate_spatials(B, R)
