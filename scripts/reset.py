#!/usr/bin/env python3
"""Standalone entry for piper_control.hardware.operations."""
from _bootstrap import ROOT
import sys
from piper_control.hardware.operations import main

if __name__ == "__main__":
    raise SystemExit(main(['home'] + sys.argv[1:]))
