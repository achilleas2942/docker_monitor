#!/usr/bin/env python3

import rospy
import docker
import concurrent.futures
from docker_monitor.msg import DockerStats

# Initialize ROS node
rospy.init_node('docker_stats_publisher', anonymous=True)
pub = rospy.Publisher('docker_stats', DockerStats, queue_size=10)
rate = rospy.Rate(1)  # Publish every second

# Docker Client
client = docker.from_env()

def fetch_and_publish_stats(container):
    """Fetches stats for a single container and publishes it."""
    try:
        stats = container.stats(stream=False)  # Get non-streaming stats

        msg = DockerStats()
        msg.container_id = container.id[:12]
        msg.name = container.name
        msg.cpu_usage = stats['cpu_stats']['cpu_usage']['total_usage'] / 1e9  # Convert to percentage
        msg.memory_usage = stats['memory_stats']['usage'] / (1024 * 1024)  # Convert to MB
        msg.memory_limit = stats['memory_stats']['limit'] / (1024 * 1024)  # Convert to MB
        msg.memory_percentage = (msg.memory_usage / msg.memory_limit) * 100
        msg.net_io = f"{stats['networks']['eth0']['rx_bytes']} / {stats['networks']['eth0']['tx_bytes']}"
        msg.block_io = f"{stats['blkio_stats']['io_service_bytes_recursive']}"
        msg.pids = stats['pids_stats']['current']
        msg.timestamp = rospy.Time.now()

        pub.publish(msg)
    except Exception as e:
        rospy.logwarn(f"Failed to fetch stats for {container.name}: {e}")

def get_docker_stats():
    """Fetches and publishes stats for all running containers in parallel."""
    while not rospy.is_shutdown():
        containers = client.containers.list()
        with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
            executor.map(fetch_and_publish_stats, containers)  # Run in parallel
        rate.sleep()

if __name__ == '__main__':
    try:
        get_docker_stats()
    except rospy.ROSInterruptException:
        pass
