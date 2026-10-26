#!/usr/bin/env python3
"""Standalone entry for piper_control.cli."""
from _bootstrap import ROOT
import sys
from piper_control.cli import main

if __name__ == "__main__":
    raise SystemExit(main(['run'] + sys.argv[1:]))
