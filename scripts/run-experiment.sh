#!/usr/bin/env bash
# =============================================================
# run-experiment.sh
# Orchestrate a TurtleBot3 figure-8 experiment on the K8s cluster
#
# Steps:
#   1. Apply the CycloneDDS ConfigMap
#   2. Deploy the bringup pod (hardware drivers on RPi4)
#   3. Wait for bringup pod to be Running
#   4. Deploy the controller pod (feedback-linearization on u1-desktop)
#   5. Tail controller logs until the experiment completes
#   6. Retrieve the CSV log from the host-path volume
#
# Prerequisites:
#   - kubectl configured and pointing at the RKE2 cluster
#   - gcrilab/turtlebot3-burger-bringup-3 and gcrilab/turtlebot3-figure8
#     images already pushed to Docker Hub
#   - CycloneDDS peer IPs in k8s/cyclonedds-configmap.yaml are correct
#   - Log host path /home/u1/turtlebot3-logs exists on u1-desktop
# =============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
K8S_DIR="${SCRIPT_DIR}/../k8s"
LOG_HOST_PATH="/home/u1/turtlebot3-logs"
NAMESPACE="default"

# ── 1. Apply ConfigMap ────────────────────────────────────────
echo "[1/5] Applying CycloneDDS ConfigMap..."
kubectl apply -f "${K8S_DIR}/cyclonedds-configmap.yaml"

# ── 2. Deploy bringup pod ─────────────────────────────────────
echo "[2/5] Deploying bringup pod..."
kubectl apply -f "${K8S_DIR}/bringup-deployment.yaml"

# ── 3. Wait for bringup pod to be Running ─────────────────────
echo "[3/5] Waiting for bringup pod to be Running (timeout 120s)..."
kubectl rollout status deployment/turtlebot3-bringup \
    -n "${NAMESPACE}" --timeout=120s

BRINGUP_POD=$(kubectl get pod -n "${NAMESPACE}" \
    -l app=turtlebot3,component=bringup \
    -o jsonpath='{.items[0].metadata.name}')
echo "    Bringup pod: ${BRINGUP_POD}"

# Wait until /odom is publishing before starting the controller
echo "    Waiting for /odom topic..."
kubectl exec -n "${NAMESPACE}" "${BRINGUP_POD}" -- \
    bash -c "source /opt/ros/humble/setup.bash && \
             timeout 60 bash -c 'until ros2 topic hz /odom --once 2>/dev/null | grep -q Hz; do sleep 2; done'" \
    || { echo "ERROR: /odom not detected within 60 s. Check bringup pod logs."; exit 1; }
echo "    /odom is active."

# ── 4. Deploy controller pod ──────────────────────────────────
echo "[4/5] Deploying controller pod..."
kubectl apply -f "${K8S_DIR}/controller-deployment.yaml"
kubectl rollout status deployment/turtlebot3-controller \
    -n "${NAMESPACE}" --timeout=60s

CTRL_POD=$(kubectl get pod -n "${NAMESPACE}" \
    -l app=turtlebot3,component=controller \
    -o jsonpath='{.items[0].metadata.name}')
echo "    Controller pod: ${CTRL_POD}"

# ── 5. Tail logs ──────────────────────────────────────────────
echo "[5/5] Tailing controller logs (Ctrl-C to detach)..."
kubectl logs -n "${NAMESPACE}" -f "${CTRL_POD}" &
LOG_PID=$!

# Wait for "Experiment complete" message
kubectl exec -n "${NAMESPACE}" "${CTRL_POD}" -- \
    bash -c "until grep -q 'Experiment complete' /proc/1/fd/1 2>/dev/null; do sleep 5; done" \
    2>/dev/null || true

kill "${LOG_PID}" 2>/dev/null || true

# ── 6. Report log location ────────────────────────────────────
echo ""
echo "Experiment complete."
echo "CSV logs are at: ${LOG_HOST_PATH}/ on node u1-desktop"
echo "To copy logs to this machine:"
echo "  scp u1@<u1-desktop-ip>:${LOG_HOST_PATH}/enhanced_log_figure8_*.csv ."
echo ""
echo "To analyse and plot:"
echo "  cd controller && python3 plot_trajectory.py enhanced_log_figure8_<N>.csv"
