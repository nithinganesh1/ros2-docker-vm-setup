# ROS2 Humble Docker + VM Multi-Device Setup

This detailed guide explains how to set up **ROS2 Humble** with Docker on a host Linux system and an Ubuntu VirtualBox VM, enabling seamless multi-device communication via DDS. It covers workspace setup, Docker Compose configuration, network setup, firewall, and quick launch scripts.

---

## Prerequisites

* **Host OS:** Linux (Ubuntu / Linux Mint tested)
* **VirtualBox VM:** Ubuntu 22.04
* **ROS2 Humble:** Installed on VM
* **Docker & Docker Compose:** Installed on host
* **Network:** Devices on same subnet (Bridged Adapter recommended)
* **Firewall:** UFW installed and optionally configured

---

## 1️⃣ Docker Setup on Host

### 1.1 Directory Structure

```bash
mkdir -p ~/ros2_setup/ros2_ws/src
cd ~/ros2_setup
```

* `ros2_ws` will be the workspace mounted inside the Docker container.

### 1.2 Docker Compose File

Create a file `docker-compose.yaml` in `~/ros2_setup`:

```yaml
version: "3.9"

services:
  ros2-desktop:
    image: osrf/ros:humble-desktop
    container_name: ros2-desktop
    network_mode: "host"  # Needed for ROS2 DDS discovery
    ipc: "host"
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - ROS_LOCALHOST_ONLY=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix   # X11 GUI forwarding
      - ./ros2_ws:/root/ros2_ws         # Workspace persistence
    working_dir: /root/ros2_ws
    tty: true
    stdin_open: true
    command: >
      bash -c "source /opt/ros/humble/setup.bash && \
               [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash; \
               exec bash"
```

### 1.3 Start Docker Container

```bash
docker-compose up -d
docker-compose exec ros2-desktop bash
```

* Container starts with ROS2 sourced and workspace mounted.

### 1.4 Create ROS2 Package Inside Container

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

## 2️⃣ VirtualBox VM Setup

### 2.1 Network Adapter

* Use **Bridged Adapter** in VirtualBox for VM networking.
* Check IP inside VM:

```bash
hostname -I
```

### 2.2 ROS2 Environment

* Install ROS2 Humble.
* Set same ROS\_DOMAIN\_ID as host:

```bash
export ROS_DOMAIN_ID=0
```

### 2.3 Firewall (UFW)

Check UFW status:

```bash
sudo ufw status
```

* If active, open ROS2 discovery ports:

```bash
sudo ufw allow 7400:7500/udp
sudo ufw reload
```

* If inactive, nothing to do.

---

## 3️⃣ Host-VM Communication Test

### 3.1 Ping Test

From host/container:

```bash
ping <VM_IP>
```

From VM:

```bash
ping <host_IP>
```

### 3.2 ROS2 Test Nodes

* On container/host:

```bash
ros2 run demo_nodes_cpp talker
```

* On VM:

```bash
ros2 run demo_nodes_cpp listener
```

* `listener` should receive messages from `talker`.

Verify nodes:

```bash
ros2 node list
```

---

## 4️⃣ Quick Launch Script for Docker ROS2

Create `run_ros.sh` in `~/ros2_setup`:

```bash
#!/bin/bash

docker-compose exec ros2-desktop bash -c "source /opt/ros/humble/setup.bash && \
                                        [ -f /root/ros2_ws/install/setup.bash ] && \
                                        source /root/ros2_ws/install/setup.bash && \
                                        exec bash"
```

Make it executable:

```bash
chmod +x run_ros.sh
```

Run with:

```bash
./run_ros.sh
```

* Provides direct access to container with ROS2 workspace sourced.

---

## 5️⃣ Notes & Best Practices

* Always **keep ROS\_DOMAIN\_ID same** across host, Docker, and VM.
* Use **network\_mode: host** in Docker for proper DDS discovery.
* Mount workspace to persist code outside Docker container.
* GUI tools like `rqt_graph` work via X11 forwarding.
* When using UFW, only open necessary ports for DDS (7400-7500/UDP).
* Bridged Adapter ensures VM is reachable by host and container.
* Quick launch script saves time for repeated container access.

---

## References

* [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
* [Docker ROS Images](https://hub.docker.com/r/osrf/ros/)
* [ROS2 Networking & DDS](https://index.ros.org/doc/ros2/Concepts/About-ROS-2-Networking/)
