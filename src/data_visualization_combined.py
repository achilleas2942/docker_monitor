import matplotlib.pyplot as plt
from collections import defaultdict
import matplotlib.cm as cm
import numpy as np
import scienceplots
import rosbag
import re
import os


def extract_cpu_data(bag_path):
    uav_data = defaultdict(lambda: {"time": [], "cpu": []})
    ground_data = defaultdict(lambda: {"time": [], "cpu": []})
    master_data = defaultdict(lambda: {"time": [], "cpu": []})

    print(f"Reading from {bag_path}...")
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=["docker_stats"]):
            name = msg.name
            cpu = msg.cpu_usage
            time = msg.timestamp.to_sec()

            if re.match(r"rotor_cbf\d+", name):
                uav_data[name]["time"].append(time)
                uav_data[name]["cpu"].append(cpu)
            elif re.match(r"tb_cbf\d+", name):
                ground_data[name]["time"].append(time)
                ground_data[name]["cpu"].append(cpu)
            elif re.match(r"master_cbf", name):
                master_data[name]["time"].append(time)
                master_data[name]["cpu"].append(cpu)

    return uav_data, ground_data, master_data


def compute_cpu_percentage(timestamps, usages):
    timestamps = np.array(timestamps)
    usages = np.array(usages)
    dt = np.diff(timestamps)
    du = np.diff(usages)
    with np.errstate(divide='ignore', invalid='ignore'):
        percentage = (du / dt) * 100
        percentage = np.insert(percentage, 0, np.nan)
    return percentage


def get_n_colors(n, cmap_name='nipy_spectral'):
    cmap = cm.get_cmap(cmap_name, n)
    return [cmap(i) for i in range(n)]


def natural_keys(text):
    return [int(c) if c.isdigit() else c for c in re.split(r'(\d+)', text)]


def extract_master_cpu_from_bag(bag_path):
    times, usages = [], []
    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=["docker_stats"]):
                if re.match(r"master_cbf", msg.name):
                    times.append(msg.timestamp.to_sec())
                    usages.append(msg.cpu_usage)
    except Exception as e:
        print(f"Failed to read {bag_path}: {e}")
    return times, usages


def plot_combined(uav_data, ground_data, bag_dir):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(4.0, 5.5), sharex=False)

    custom_labels = {f"rotor_cbf{i}": f"Pair {i}" for i in range(1, 21)}
    colors = get_n_colors(len(uav_data))

    lines_to_plot = sorted(
        [(name, custom_labels[name], colors[i]) for i, name in enumerate(sorted(uav_data.keys()))],
        key=lambda x: natural_keys(x[1])
    )

    # (a) UAV subplot
    for name, label, color in lines_to_plot:
        data = uav_data[name]
        cpu_pct = compute_cpu_percentage(data["time"], data["cpu"]) / 11
        cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
        if data["time"]:
            t = np.array(data["time"]) - data["time"][0]
            t = np.insert(t, 0, 0)
            ax1.plot(t, cpu_pct, label=label, color=color)
            ax1.axhline(y=100, linestyle='--')
            ax1.axhline(y=65, linestyle='--')
    ax1.set_xlim([0, 150])
    ax1.set_ylim([60, 105])
    ax1.set_yticks(np.arange(60, 105, 10))
    ax1.set_xticklabels([])  # remove x-tick labels, shared with ax2
    ax1.set_ylabel(r"CPU $(\%)$", fontsize=10)
    ax1.set_title("(a) UAV Containers")

    # (b) UGV subplot
    for i, (name, data) in enumerate(sorted(ground_data.items())):
        cpu_pct = compute_cpu_percentage(data["time"], data["cpu"])
        cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
        if data["time"]:
            t = np.array(data["time"]) - data["time"][0]
            t = np.insert(t, 0, 0)
            ax2.plot(t, cpu_pct, label=None, color=colors[i])
            ax2.axhline(y=23, linestyle='--')
            ax2.axhline(y=15, linestyle='--')
    ax2.set_xlim([0, 150])
    ax2.set_ylim([13, 25])
    ax2.set_yticks(np.arange(13, 26, 3))
    ax2.set_ylabel(r"CPU $(\%)$", fontsize=10)
    ax2.set_title("(b) UGV Containers")

    # (c) Master containers — merged: multi-bag comparison
    bag_files = sorted([os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.bag')])
    custom_labels_master = ["5 Pairs", "10 Pairs", "20 Pairs"]
    for i, bag_file in enumerate(bag_files):
        times, usages = extract_master_cpu_from_bag(bag_file)
        if not times or not usages:
            continue
        cpu_pct = compute_cpu_percentage(times, usages) / 10.0
        cpu_pct[0] = 0  # replace leading NaN so line starts visible at t=0
        t_arr = np.array(times) - times[0]
        label = custom_labels_master[i] if i < len(custom_labels_master) else os.path.basename(bag_file)
        ax3.plot(t_arr, cpu_pct, label=label)
    ax3.set_xlim([0, 80])
    ax3.set_ylim([0, 45])
    ax3.set_yticks(np.arange(0, 50, 10))
    ax3.set_xlabel(r"time $(s)$", fontsize=10)
    ax3.set_ylabel(r"CPU $(\%)$", fontsize=10)
    ax3.set_title("(c) Master Containers")
    ax3.legend(fontsize=7, loc='upper right')

    # Pairs legend between ax2 and ax3
    h1, l1 = ax1.get_legend_handles_labels()
    fig.legend(h1, l1, loc='upper center', bbox_to_anchor=(0.5, 0.38),
               ncol=5, fontsize=6, frameon=False)

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.9)
    plt.savefig("/monitor/docker_cpu_combined.pdf", bbox_inches="tight")
    plt.savefig("/monitor/docker_cpu_combined.png", bbox_inches="tight")


def main():
    bag_path = "/monitor/bag_250507_1448.bag"
    bag_dir = "/monitor/"

    uav_data, ground_data, _ = extract_cpu_data(bag_path)
    plot_combined(uav_data, ground_data, bag_dir)


if __name__ == "__main__":
    main()
