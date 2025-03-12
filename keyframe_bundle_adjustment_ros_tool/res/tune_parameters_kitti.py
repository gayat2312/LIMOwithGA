#!/usr/bin/env python3
import subprocess

def main():
    depth_thres = [0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19]
    repr_thres = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.8, 2.0]
    # weights = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    weights = [0.9]

    for d in depth_thres:
        for r in repr_thres:
            for w in weights:
                print(f"Running evaluation with depth_thres={d}, repr_thres={r}, weight={w}")
                subprocess.run(["bash", "kitti_eval_script.sh", f"{d}", f"{r}", f"{w}"], check=True)

if __name__ == "__main__":
    main()
