# TurtleBot3 on Kubernetes (RKE2 + ROS 2 Humble)

This repository contains the full system software for running a TurtleBot3 Burger under Kubernetes orchestration with ROS 2 Humble and CycloneDDS. It accompanies the paper:

> **Orchestrating Mobile Robot Control with Kubernetes: A ROS 2 Integration on Bare-Metal Edge Clusters**  
> *IEEE International Conference on Collaboration and Internet Computing (CIC) 2026*

The system decomposes the robot software into two independently managed pods—a **bringup pod** that owns the hardware drivers on the Raspberry Pi 4, and a **controller pod** that runs a feedback-linearization trajectory controller on a desktop node—and bridges them over a Flannel overlay network using CycloneDDS unicast peer discovery.

---

## Repository Layout

```
turtlebot3-k8s/
├── bringup/                     # Hardware-driver pod (runs on RPi4)
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── run.sh                   # Standalone Docker run (no K8s)
│   └── launch/
│       └── bringup_with_camera.launch.py
├── controller/                  # Feedback-linearization controller pod
│   ├── Dockerfile
│   ├── entrypoint.sh
│   ├── run.sh                   # Standalone Docker run (no K8s)
│   ├── enhanced_fdb_eight.py    # Main ROS 2 node (figure-8 trajectory)
│   └── plot_trajectory.py       # Post-experiment visualisation script
├── k8s/                         # Kubernetes manifests
│   ├── cyclonedds-configmap.yaml
│   ├── bringup-deployment.yaml
│   └── controller-deployment.yaml
└── scripts/
    └── run-experiment.sh        # End-to-end experiment orchestration
```

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Robot | TurtleBot3 Burger |
| On-board SBC | Raspberry Pi 4 (4 GB, ARM64) |
| LiDAR | LDS-02 (ROBOTIS) |
| Camera | USB (V4L2-compatible) |
| Controller node | x86-64 desktop / workstation |
| Cluster nodes | ≥ 2 nodes in an RKE2 cluster |
| CNI | Flannel (VXLAN overlay) |

---

## Cluster Setup

### 1. RKE2

Install RKE2 on each node following the [official RKE2 quickstart](https://docs.rke2.io/install/quickstart). The server node is the desktop; the RPi4 is an agent node.

```bash
# Server (desktop)
curl -sfL https://get.rke2.io | sh -
systemctl enable rke2-server --now

# Agent (RPi4) — replace <SERVER_IP> and <TOKEN>
curl -sfL https://get.rke2.io | INSTALL_RKE2_TYPE="agent" sh -
cat > /etc/rancher/rke2/config.yaml <<EOF
server: https://<SERVER_IP>:9345
token: <TOKEN>
EOF
systemctl enable rke2-agent --now
```

### 2. Label the RPi4 node

```bash
kubectl label node <rpi4-hostname> role=robot
```

### 3. CycloneDDS peer addresses

Edit `k8s/cyclonedds-configmap.yaml` and replace the placeholder IP addresses with the actual host-network IPs of your cluster nodes:

```yaml
<Peer Address="192.168.1.100"/>  # desktop (controller)
<Peer Address="192.168.1.101"/>  # RPi4   (bringup)
```

`hostNetwork: true` is set on both deployments so the pods use the node's physical network interface; the Flannel overlay is bypassed for ROS 2 traffic.

---

## Building the Docker Images

Multi-architecture images (linux/amd64 + linux/arm64) are built with `docker buildx`.

```bash
# One-time buildx setup
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap

# Bringup image (builds on amd64, cross-compiles for arm64)
cd bringup
docker buildx build \
    --platform linux/amd64,linux/arm64 \
    -t gcrilab/turtlebot3-burger-bringup-3:latest \
    --push .

# Controller image
cd ../controller
docker buildx build \
    --platform linux/amd64,linux/arm64 \
    -t gcrilab/turtlebot3-figure8:latest \
    --push .
```

> **Note:** The bringup image builds TurtleBot3 packages from source and takes ~15–20 minutes on first build.

---

## Running an Experiment

### Option A — Fully orchestrated via Kubernetes

```bash
chmod +x scripts/run-experiment.sh
./scripts/run-experiment.sh
```

The script applies the ConfigMap, deploys both pods in order, waits for `/odom` to appear before starting the controller, tails logs, and reports the CSV log location when the experiment completes.

### Option B — Apply manifests manually

```bash
kubectl apply -f k8s/cyclonedds-configmap.yaml
kubectl apply -f k8s/bringup-deployment.yaml

# Wait until bringup pod is Running and /odom is publishing, then:
kubectl apply -f k8s/controller-deployment.yaml

# Monitor
kubectl logs -f deployment/turtlebot3-controller
```

### Option C — Standalone Docker (no K8s)

```bash
# On the RPi4:
cd bringup && bash run.sh

# On the desktop (after bringup is running):
cd controller && bash run.sh
```

---

## Controller Parameters

The controller is a feedback-linearization law for the kinematic unicycle model, tracking a Lissajous (figure-8) reference trajectory. All parameters can be overridden at launch via `--ros-args`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `a` | `0.5` | Figure-8 semi-axis in X (m) |
| `b` | `0.25` | Figure-8 semi-axis in Y (m) |
| `k_p1`, `k_p2` | `2.0` | Proportional gains |
| `k_d1`, `k_d2` | `2.5` | Derivative gains |
| `v_max` | `0.18` | Maximum forward speed (m/s) |
| `v_min` | `0.06` | Minimum forward speed (m/s) |
| `v_cap` | `0.22` | Absolute speed cap (m/s) |
| `lambda` | `2.0` | Curvature speed-reduction factor |
| `rate_hz` | `200.0` | Control loop rate (Hz) |
| `experiment_duration` | `600.0` | Experiment length (s) |

**Example — run at reduced speed for 3 minutes:**

```bash
kubectl exec -it deployment/turtlebot3-controller -- \
    python3 /app/enhanced_fdb_eight.py \
    --ros-args -p v_max:=0.12 -p experiment_duration:=180.0
```

---

## Analyzing Results

After an experiment, copy the CSV log from the host-path volume and run the visualization script:

```bash
scp u1@<desktop-ip>:/home/u1/turtlebot3-logs/enhanced_log_figure8_1.csv .
cd controller
python3 plot_trajectory.py enhanced_log_figure8_1.csv
```

The script produces a six-panel figure (XY trajectory, tracking error norm, error components, X/Y time-series, summary statistics) and saves it as a PNG alongside the CSV.

**Experimental results (180 s run, figure-8 a=0.5 m, b=0.25 m):**

| Metric | Value |
|--------|-------|
| Control rate (realized) | 199.2 Hz |
| Tracking error — mean | 6.70 cm |
| Tracking error — RMSE | 6.96 cm |
| Tracking error — max | 11.65 cm |
| /odom inter-pod latency — mean | 2.67 ms |
| /odom inter-pod latency — max | 11.49 ms |

---

## Citation

If you use this software in your research, please cite:

```bibtex
@inproceedings{ajeigbe2026turtlebot3k8s,
  title     = {Orchestrating Mobile Robot Control with {Kubernetes}:
               A {ROS} 2 Integration on Bare-Metal Edge Clusters},
  author    = {Ajeigbe, Oluwafemi, Grace Harris, and Roy, Sandip},
  booktitle = {Proc. IEEE International Conference on Collaboration
               and Internet Computing (CIC)},
  year      = {2026},
  address   = {San Jose, CA},
}
```

---

## License

MIT — see [LICENSE](LICENSE).
