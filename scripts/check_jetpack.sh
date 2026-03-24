#!/usr/bin/env bash
set -euo pipefail

echo "=== /etc/nv_tegra_release ==="
cat /etc/nv_tegra_release
echo
echo "=== nvidia-l4t-core ==="
apt-cache policy nvidia-l4t-core | sed -n '1,8p'
echo
echo "=== nvidia-jetpack ==="
apt-cache policy nvidia-jetpack | sed -n '1,8p'
