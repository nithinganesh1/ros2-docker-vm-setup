# ROS2 Docker + VM Multi-Device Setup

This guide helps you set up **ROS2 Humble** with Docker on a host system and Ubuntu VM, enabling multi-device communication via DDS.

---

## Prerequisites

* **Host System:** Linux (tested on Linux Mint / Ubuntu)
* **Docker:** Installed
* **Docker Compose:** Installed (`docker-compose version 1.29.2` or higher)
* **VirtualBox VM:** Ubuntu 22.04 with ROS2 Humble installed
* **Network:** Devices on the same subnet (bridged adapter recommended)

---

## Docker Setup

### 1️⃣ Directory Structure

```bash
mkdir -p ~/ros2_setup/ros2_ws/src
cd ~/ros2_setup
```

* `ros2_ws` will be the ROS2 workspace mounted inside Docker.

### 2️⃣ Docker Compose File

Create `docker-compose.yaml`:

```yaml
version: "3.9"

services:
  ros2-desktop:
    image: osrf/ros:humble-desktop
    container_name: ros2-desktop
    network_mode: "host"        # Required for ROS2 DDS discovery
    ipc: "host"
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - ROS_LOCALHOST_ONLY=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix   # Forward X11 for GUI apps
      - ./ros2_ws:/root/ros2_ws         # Mount host workspace
    working_dir: /root/ros2_ws
    tty: true
    stdin_open: true
    command: >
      bash -c "source /opt/ros/humble/setup.bash && \
               [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash; \
               exec bash"
```

### 3️⃣ Start Container

```bash
docker-compose up -d
docker-compose exec ros2-desktop bash
```

* Container starts with **ROS2 sourced automatically** and workspace mounted.

### 4️⃣ Workspace Setup

Inside container:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_first_pkg --build-type ament_python
```

Build workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

* Workspace changes persist on host due to volume mount.

---

## VM Setup (Ubuntu)

1. Install ROS2 Humble.
2. Set same `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=0
```

3. Ensure networking: Bridged Adapter, check IP: `hostname -I`

4. Open ROS2 discovery UDP ports if firewall active (UFW):

```bash
sudo ufw allow 7400:7500/udp
sudo ufw reload
```

---

## Host-VM Communication

1. Ping test:

```bash
ping <VM_IP>    # from host/container
ping <host_IP>  # from VM
```

2. Run ROS2 test nodes:

* On container/host:

```bash
ros2 run demo_nodes_cpp talker
```

* On VM:

```bash
ros2 run demo_nodes_cpp listener
```

* `listener` should receive messages from `talker`.

3. Verify nodes:

```bash
ros2 node list
```

---

## Quick Launch Script

Create `run_ros.sh` in `ros2_setup`:

```bash
#!/bin/bash

docker-compose exec ros2-desktop bash -c "source /opt/ros/humble/setup.bash && \
                                        [ -f /root/ros2_ws/install/setup.bash ] && \
                                        source /root/ros2_ws/install/setup.bash && \
                                        exec bash"
```

Make executable and run:

```bash
chmod +x run_ros.sh
./run_ros.sh
```

---

## Notes

* Always ensure **same ROS\_DOMAIN\_ID** across host, Docker, and VM.
* Use **network\_mode: host** for DDS discovery in Docker.
* Mount your workspace to persist code outside container.
* GUI tools like `rqt_graph` work via X11 forwarding.

---

## References

* [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
* [Docker ROS Images](https://hub.docker.com/r/osrf/ros/)
* [ROS2 Networking & DDS](https://index.ros.org/doc/ros2/Concepts/About-ROS-2-Networking/)
