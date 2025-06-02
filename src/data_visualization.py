import rosbag
import re
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
import scienceplots

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


def plot_cpu_timeseries(uav_data, ground_data, master_data):
    with plt.style.context(["science", "ieee"]):
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 4.5), sharex=True)

        # UAV subplot
        for name, data in sorted(uav_data.items()):
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]  # normalize
                ax1.plot(t, data["cpu"], label=name)
        ax1.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax1.set_title("UAV Containers")
        ax1.grid(True)
        # ax1.legend(fontsize=6)

        # Ground robot subplot
        for name, data in sorted(ground_data.items()):
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]
                ax2.plot(t, data["cpu"], label=name)
        ax2.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax2.set_title("UGV Containers")
        ax2.grid(True)
        # ax2.legend(fontsize=6)
        
        for name, data in sorted(master_data.items()):
            if data["time"]:
                t = np.array(data["time"]) - data["time"][0]  # normalize
                ax3.plot(t, data["cpu"], label=name)
        ax3.set_xlabel(r"Time $(s)$", fontsize=10)
        ax3.set_ylabel(r"CPU $(\%)$", fontsize=10)
        ax3.set_title("Master Container")
        ax3.grid(True)

        plt.tight_layout()
        plt.savefig("/home/oem/Downloads/docker_cpu_timeseries.pdf", bbox_inches="tight")
        plt.savefig("/home/oem/Downloads/docker_cpu_timeseries.png", bbox_inches="tight")
        plt.show()


def main():
    bag_path = "/home/oem/Downloads/bag_250507_1419.bag"
    uav_data, ground_data, master_data = extract_cpu_data(bag_path)
    plot_cpu_timeseries(uav_data, ground_data, master_data)


if __name__ == "__main__":
    main()
