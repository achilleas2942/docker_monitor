import matplotlib.pyplot as plt
from collections import defaultdict
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import numpy as np
import scienceplots
import rosbag
import re
import os

def compute_cpu_percentage(timestamps, usages):
    timestamps = np.array(timestamps)
    usages = np.array(usages)
    dt = np.diff(timestamps)
    du = np.diff(usages)
    with np.errstate(divide='ignore', invalid='ignore'):
        cpu_pct = (du / dt) * 100
        cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
        cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
    return cpu_pct

def extract_master_cpu_from_bag(bag_path):
    times = []
    usages = []
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["docker_stats"]):
                if re.match(r"master_cbf", msg.name):
                    times.append(msg.timestamp.to_sec())
                    usages.append(msg.cpu_usage)
    except Exception as e:
        print(f"Failed to read {bag_path}: {e}")
    return times, usages

def main():
    with plt.style.context(["science", "ieee"]):
        bag_dir = "/home/oem/Downloads/bags/"  # Update this path
        bag_files = sorted([os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.bag')])

        plt.figure(figsize=(5.0, 2.0))
        
        for i, bag_file in enumerate(bag_files):
            times, usages = extract_master_cpu_from_bag(bag_file)
            # print(f"Processing {bag_file}: {len(times)} entries found.")
            # print(f"Times: {times[:5]}... Usages: {usages[:5]}...")  # Print first 5 for debugging
            if not times or not usages:
                continue
            cpu_pct = compute_cpu_percentage(times, usages) / 10.0  # Normalize as in your previous logic
            t = np.array(times) - times[0]
            t = np.insert(t, 0, 0)  # Ensure the first time is zero
            custom_labels = ["5 Pairs", "10 Pairs", "20 Pairs"]
            label = custom_labels[i]
            plt.plot(t, cpu_pct, label=label)

        plt.xlabel(r"time $(s)$", fontsize=10)
        plt.ylabel(r"CPU $(\%)$", fontsize=10)
        plt.title("Master Containers")
        # plt.grid(True)
        plt.legend(fontsize=6, loc='upper right')
        plt.tight_layout()
        plt.ylim(0, 35)
        plt.xlim(0, 80)
        plt.subplots_adjust(right=0.75)
        plt.savefig("/home/oem/Downloads/master_cpu_comparison.pdf", bbox_inches="tight")
        plt.savefig("/home/oem/Downloads/master_cpu_comparison.png", bbox_inches="tight")
        # plt.show()

if __name__ == "__main__":
    main()
