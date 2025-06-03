import matplotlib.pyplot as plt
from collections import defaultdict
import matplotlib.colors as mcolors
import matplotlib.cm as cm
import numpy as np
import scienceplots
import rosbag
import re


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
    """Compute approximate CPU percentage from cumulative usage."""
    timestamps = np.array(timestamps)
    usages = np.array(usages)
    
    dt = np.diff(timestamps)
    du = np.diff(usages)
    
    with np.errstate(divide='ignore', invalid='ignore'):
        percentage = (du / dt) * 100  # Approximate usage in %
        percentage = np.insert(percentage, 0, np.nan)  # align length
    
    return percentage

def get_n_colors(n, cmap_name='nipy_spectral'):
    cmap = cm.get_cmap(cmap_name, n)
    return [cmap(i) for i in range(n)]

def natural_keys(text):
    # Split text into parts of digits and non-digits
    return [int(c) if c.isdigit() else c for c in re.split(r'(\d+)', text)]

def plot_cpu_timeseries(uav_data, ground_data, master_data):
    with plt.style.context(["science", "ieee"]):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(4.0, 3.8), sharex=True)
        custom_labels = {
            "rotor_cbf1": "Pair 1",
            "rotor_cbf2": "Pair 2",
            "rotor_cbf3": "Pair 3",
            "rotor_cbf4": "Pair 4",
            "rotor_cbf5": "Pair 5",
            "rotor_cbf6": "Pair 6",
            "rotor_cbf7": "Pair 7",
            "rotor_cbf8": "Pair 8",
            "rotor_cbf9": "Pair 9",
            "rotor_cbf10": "Pair 10",
            "rotor_cbf11": "Pair 11",
            "rotor_cbf12": "Pair 12",
            "rotor_cbf13": "Pair 13",
            "rotor_cbf14": "Pair 14",
            "rotor_cbf15": "Pair 15",
            "rotor_cbf16": "Pair 16",
            "rotor_cbf17": "Pair 17",
            "rotor_cbf18": "Pair 18",
            "rotor_cbf19": "Pair 19",
            "rotor_cbf20": "Pair 20",
        }
        
        colors = get_n_colors(len(uav_data))
        
        lines_to_plot = sorted(
            [(name, custom_labels[name], colors[i]) for i, name in enumerate(sorted(uav_data.keys()))],
            key=lambda x: natural_keys(x[1])  # Sort by label (e.g., "Pair 10" > "Pair 2")
        )

        # UAV subplot
        for name, label, color in lines_to_plot:
            data = uav_data[name]
            cpu_pct = compute_cpu_percentage(data["time"], data["cpu"]) / 10    # Normalize time for uav containers
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]  # normalize
                # label = custom_labels.get(name, name)
                ax1.plot(t, cpu_pct, label=label, color=color)
        ax1.set_xlim([0, 28.5])
        ax1.set_ylim([65, 110])
        ax1.set_yticks(np.arange(65, 115, 15))
        ax1.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax1.set_title("(a) UAV Containers")
        # ax1.grid(True)
        # ax1.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=6, frameon=False)

        # Ground robot subplot
        for i, (name, data) in enumerate(sorted(ground_data.items())):
            cpu_pct = compute_cpu_percentage(data["time"], data["cpu"])
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]  # normalize
                ax2.plot(t, cpu_pct, label=None, color=colors[i])
        ax2.set_xlim([0, 28.5])
        ax2.set_ylim([16, 22])
        ax2.set_yticks(np.arange(16, 24, 2))
        ax2.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax2.set_title("(b) UGV Containers")
        # ax2.grid(True)
        
        for name, data in sorted(master_data.items()):
            cpu_pct = compute_cpu_percentage(data["time"], data["cpu"]) / 10  # Normalize time for master container
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]  # normalize
                ax3.plot(t, cpu_pct, label="Watcher")
        ax3.set_xlim([0, 28.5])
        ax3.set_ylim([10, 30])
        ax3.set_yticks(np.arange(10, 35, 10))
        ax3.set_xlabel(r"time $(s)$", fontsize=10)
        ax3.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax3.set_title("(c) Master Container")
        # ax3.grid(True)
        ax3.legend(fontsize=6, loc='center left', bbox_to_anchor=(1.02, 0.5), frameon=False)
        
        handles, labels = ax1.get_legend_handles_labels()
        fig.legend(
            handles, labels,
            loc='center left',
            bbox_to_anchor=(0.79, 0.65),  # Adjust vertical placement to span ax1 + ax2
            fontsize=6,
            frameon=False
        )

        plt.tight_layout()
        plt.subplots_adjust(right=0.78)  
        plt.savefig("/home/oem/Downloads/docker_cpu_timeseries.pdf", bbox_inches="tight")
        plt.savefig("/home/oem/Downloads/docker_cpu_timeseries.png", bbox_inches="tight")
        plt.show()


def main():
    bag_path = "/home/oem/Downloads/bag_250507_1419.bag"
    uav_data, ground_data, master_data = extract_cpu_data(bag_path)
    plot_cpu_timeseries(uav_data, ground_data, master_data)


if __name__ == "__main__":
    main()
