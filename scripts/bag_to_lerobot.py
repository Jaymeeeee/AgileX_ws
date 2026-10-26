#!/usr/bin/env python3
"""Convert the insert_plug_v4 ROS bags offline; never connects to hardware."""
from _bootstrap import ROOT
from piper_control.recording.bag_to_lerobot import main

if __name__ == "__main__":
    raise SystemExit(main())
