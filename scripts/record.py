#!/usr/bin/env python3
"""Standalone manual ROS bag recorder; never connects to robot hardware."""
from _bootstrap import ROOT
from piper_control.recording.manual_bag import main

if __name__ == "__main__":
    raise SystemExit(main())
