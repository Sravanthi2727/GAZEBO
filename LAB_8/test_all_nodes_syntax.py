#!/usr/bin/env python3
"""
Syntax and import validation test for all Lab 8 Python nodes.
"""

import ast
import os
import sys

LAB8_DIR = os.path.dirname(os.path.realpath(__file__))

py_files = [
    "problem1_battery_aware_robot_node.py",
    "problem2_decentralized_exploration_node.py",
    "problem3_distributed_mission_planning_node.py",
    "problem4_self_driving_car_node.py",
    "problem5_agricultural_monitoring_node.py",
]

print("==================================================")
print(" Validating Syntax for All Lab 8 Python Nodes")
print("==================================================")

all_passed = True
for f in py_files:
    path = os.path.join(LAB8_DIR, f)
    if not os.path.exists(path):
        print(f"[-] MISSING: {f}")
        all_passed = False
        continue
    try:
        with open(path, "r", encoding="utf-8") as src:
            ast.parse(src.read(), filename=f)
        print(f"[+] SYNTAX OK: {f}")
    except SyntaxError as e:
        print(f"[-] SYNTAX ERROR in {f}: {e}")
        all_passed = False

if all_passed:
    print("==================================================")
    print(" SUCCESS: All 5 Python nodes have valid syntax!")
    print("==================================================")
    sys.exit(0)
else:
    print("==================================================")
    print(" FAILURE: One or more scripts had syntax errors.")
    print("==================================================")
    sys.exit(1)
