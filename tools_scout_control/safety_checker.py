#!/usr/bin/env python3
"""
MOVED — this file has been relocated to tools_safety_nav/safety_checker.py

Run:
    python3 tools_safety_nav/safety_checker.py

Or via the tmux launcher:
    bash run_all.sh
"""
import sys

print(
    "\n[ERROR] safety_checker.py has moved.\n"
    "  New location: tools_safety_nav/safety_checker.py\n"
    "  Run from repo root: python3 tools_safety_nav/safety_checker.py\n",
    file=sys.stderr,
)
sys.exit(1)
