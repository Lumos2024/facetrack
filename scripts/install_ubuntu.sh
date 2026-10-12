#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble 的 rclpy/std_msgs 由 apt 安装在系统 Python 中，因此需要继承系统包。
if ! command -v python3.10 >/dev/null 2>&1; then
    echo "错误：需要 Ubuntu 22.04 的 Python 3.10。" >&2
    exit 1
fi

python3.10 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/python -m pytest

echo "安装完成。运行：source .venv/bin/activate"
echo "然后执行：facetrack --display  # 自动选择 W19 摄像头"

