#!/usr/bin/env python3
"""Measure real ACT update throughput; run only while the main job is stopped."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batches", nargs="+", type=int, default=[16, 32])
    args = parser.parse_args()
    ws = Path(__file__).resolve().parents[1]
    results = []
    for batch in args.batches:
        log = ws / "logs" / f"act_benchmark_bs{batch}.log"
        cmd = [str(ws / "training/train_act.sh"), f"--batch_size={batch}", "--steps=30",
               "--log_freq=5", "--save_checkpoint=false", "--eval_steps=0",
               f"--output_dir={ws / 'outputs' / f'act_benchmark_bs{batch}'}"]
        print(f"Benchmark batch={batch}, log={log}", flush=True)
        started = time.monotonic()
        with log.open("xb") as stream:
            process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=240)
            except BaseException:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=20)
                raise
        text = log.read_text(errors="replace")
        rows = re.findall(r"step_s:([\d.]+) smp/s:([\d.]+) mem_gb:([\d.]+)", text)
        result = {"batch_size": batch, "exit_code": code, "log": str(log),
                  "elapsed_sec": round(time.monotonic() - started, 2)}
        if code == 0 and len(rows) >= 4:
            stable = rows[-3:]  # Discard initialization and first updates.
            step_s = sum(float(row[0]) for row in stable) / len(stable)
            result.update(step_s=step_s, samples_per_sec=batch/step_s,
                          peak_allocated_gib=max(float(row[2]) for row in rows))
        else:
            result["error_tail"] = text[-2500:]
        results.append(result)
        (ws / "logs/act_batch_benchmark.json").write_text(json.dumps(results, indent=2)+"\n")
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
