#!/usr/bin/env python3
"""Derive current-only state from the existing right-arm LeRobot dataset."""
from _bootstrap import ROOT
from piper_control.recording.select_state import main

if __name__ == "__main__":
    raise SystemExit(main())
