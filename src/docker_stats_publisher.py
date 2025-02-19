#!/usr/bin/env python3

import rospy
import docker
from docker_monitor.msg import DockerStats


def get_docker_stats():
    client = docker.from_env()
    containers = client.containers.list()

    rospy.init_node("docker_stats_publisher", anonymous=True)
    pub = rospy.Publisher(f"docker_stats/{container.name}", DockerStats, queue_size=10)
    rate = rospy.Rate(1)  # Publish every second

    while not rospy.is_shutdown():
        for container in containers:
            stats = container.stats(stream=False)  # Get non-streaming stats

            msg = DockerStats()
            msg.container_id = container.id[:12]
            msg.name = container.name
            msg.cpu_usage = (
                stats["cpu_stats"]["cpu_usage"]["total_usage"] / 1e9
            )  # Convert to percentage
            msg.memory_usage = stats["memory_stats"]["usage"] / (
                1024 * 1024
            )  # Convert to MB
            msg.memory_limit = stats["memory_stats"]["limit"] / (
                1024 * 1024
            )  # Convert to MB
            msg.memory_percentage = (msg.memory_usage / msg.memory_limit) * 100
            msg.net_io = f"{stats['networks']['eth0']['rx_bytes']} / {stats['networks']['eth0']['tx_bytes']}"
            msg.block_io = f"{stats['blkio_stats']['io_service_bytes_recursive']}"
            msg.pids = stats["pids_stats"]["current"]

            pub.publish(msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        get_docker_stats()
    except rospy.ROSInterruptException:
        pass
