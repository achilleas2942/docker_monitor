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


def plot_combined(uav_data, ground_data, master_data, bag_dir):
    with plt.style.context(["science", "ieee"]):
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(4.0, 5.0), sharex=False)

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

        # (c) Master (single bag)
        for name, data in sorted(master_data.items()):
            cpu_pct = compute_cpu_percentage(data["time"], data["cpu"]) / 10
            cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
            cpu_pct = np.insert(cpu_pct, len(cpu_pct) - 1, cpu_pct[len(cpu_pct) - 1])
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]
                t = np.insert(t, 0, 0)
                t = np.append(t, 150)
                ax3.plot(t, cpu_pct, label="Watcher")
                ax3.axhline(y=30, linestyle='--')
                ax3.axhline(y=18, linestyle='--')
        ax3.set_xlim([0, 150])
        ax3.set_ylim([14, 34])
        ax3.set_yticks(np.arange(14, 35, 4))
        ax3.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax3.set_title("(c) Master Container")
        ax3.legend(fontsize=6, loc='center left', bbox_to_anchor=(1.02, 0.5), frameon=False)

        # (d) Master comparison across bags
        bag_files = sorted([os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.bag')])
        custom_labels_master = ["5 Pairs", "10 Pairs", "20 Pairs"]
        for i, bag_file in enumerate(bag_files):
            times, usages = extract_master_cpu_from_bag(bag_file)
            if not times or not usages:
                continue
            cpu_pct = compute_cpu_percentage(times, usages) / 10.0
            cpu_pct = np.insert(cpu_pct, 0, cpu_pct[0])
            t_arr = np.array(times) - times[0]
            t_arr = np.insert(t_arr, 0, 0)
            label = custom_labels_master[i] if i < len(custom_labels_master) else os.path.basename(bag_file)
            ax4.plot(t_arr, cpu_pct, label=label)
        ax4.set_xlim([0, 80])
        ax4.set_ylim([0, 35])
        ax4.set_xlabel(r"time $(s)$", fontsize=10)
        ax4.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax4.set_title("(d) Master Containers Comparison")
        ax4.legend(fontsize=6, loc='upper right')

        # Shared legend for ax1/ax2 pairs
        handles, labels = ax1.get_legend_handles_labels()
        fig.legend(
            handles, labels,
            loc='center left',
            bbox_to_anchor=(0.79, 0.72),
            fontsize=6,
            frameon=False
        )

        plt.tight_layout()
        plt.subplots_adjust(right=0.78)
        plt.savefig("/home/oem/Downloads/docker_cpu_combined.pdf", bbox_inches="tight")
        plt.savefig("/home/oem/Downloads/docker_cpu_combined.png", bbox_inches="tight")


def main():
    bag_path = "/home/oem/Downloads/bags/bag_250507_1448.bag"
    bag_dir = "/home/oem/Downloads/bags/"

    uav_data, ground_data, master_data = extract_cpu_data(bag_path)
    plot_combined(uav_data, ground_data, master_data, bag_dir)


if __name__ == "__main__":
    main()
