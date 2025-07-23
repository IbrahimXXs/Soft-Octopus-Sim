#!/usr/bin/env python3
"""
generate_muscles.py

Prompts for B and R (e.g. B=2, R=2) and then prints out 9 <muscle> XML lines
with the exact lengthrange mapping you specified.
"""

def generate_muscles(B: int, R: int):
    # mapping from muscle index → minimum length
    length_mins = {
        1: 0.001,
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
        name   = f"A_B{B}S1_{i}_R{R}"
        tendon = f"T_B{B}S1_{i}_R{R}"
        length = length_mins[i]
        print(
            f'\t<muscle name="{name}" '
            + 'ctrllimited="true" '
            + f'lengthrange="{length:.3f} 0.1" '
            + 'ctrlrange="0 1" '
            + 'force="50" '
            + f'tendon="{tendon}"/>'
        )

if __name__ == "__main__":
    try:
        B = int(input("Enter B number (e.g. 2): ").strip())
        R = int(input("Enter R number (e.g. 2): ").strip())
    except ValueError:
        print("Please enter integer values for B and R.")
        exit(1)

    print(f"\n<!-- Generating muscles for B={B}, R={R} -->")
    generate_muscles(B, R)
