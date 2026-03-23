#!/usr/bin/env bash
# Install ROS2 rover systemd services
# Usage: sudo bash setup_services.sh

set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSTEMD_DIR="/etc/systemd/system"
SERVICES=(ros2-ublox ros2-ntrip ros2-mavlink ros2-look-ahead ros2-maestro)

# Must run as root
if [[ $EUID -ne 0 ]]; then
    echo "Error: Run this script with sudo"
    exit 1
fi

echo "=== ROS2 Rover Service Installer ==="

# 1. Make scripts executable
echo "Setting script permissions..."
chmod +x "$DEPLOY_DIR"/env.sh
chmod +x "$DEPLOY_DIR"/scripts/*.sh

# 2. Symlink service files to systemd
echo "Creating systemd symlinks..."
for svc in "${SERVICES[@]}"; do
    ln -sf "$DEPLOY_DIR/systemd/${svc}.service" "$SYSTEMD_DIR/${svc}.service"
    echo "  Linked ${svc}.service"
done

# 3. Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload

# 4. Enable all services
echo "Enabling services..."
systemctl enable "${SERVICES[@]}"

# 5. Check dialout group
ROVER_USER="kikai"
if id -nG "$ROVER_USER" | grep -qw dialout; then
    echo "User '$ROVER_USER' is in the dialout group."
else
    echo "WARNING: User '$ROVER_USER' is NOT in the dialout group."
    echo "  Serial ports (u-blox, Maestro) will not work."
    echo "  Fix with: sudo usermod -aG dialout $ROVER_USER"
    echo "  Then log out and back in."
fi

echo ""
echo "=== Installation complete ==="
echo ""
echo "Usage:"
echo "  Start all:    sudo systemctl start ${SERVICES[*]}"
echo "  Stop all:     sudo systemctl stop ${SERVICES[*]}"
echo "  Status:       systemctl status ${SERVICES[*]}"
echo "  View logs:    journalctl -u ros2-mavlink -f"
echo "  Disable all:  sudo systemctl disable ${SERVICES[*]}"
