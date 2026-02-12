#!/usr/bin/env bash
# 在 WSL/Ubuntu 下安装 PX4 依赖并编译 fmu-v5 固件
# 用法: 在 WSL 中进入 PX4 源码根目录后执行: bash Tools/setup/install_and_build_fmuv5.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PX4_ROOT}"

echo "=========================================="
echo "PX4 依赖安装与 fmu-v5 编译"
echo "项目路径: ${PX4_ROOT}"
echo "=========================================="

# 1. 安装依赖（仅 NuttX 与基础工具，不安装仿真器以节省时间）
if [ -f "${SCRIPT_DIR}/ubuntu.sh" ]; then
	echo ""
	echo "[1/3] 正在安装依赖 (ubuntu.sh --no-sim-tools)..."
	echo "      如需安装仿真器，请去掉 --no-sim-tools 后重新运行 ubuntu.sh"
	bash "${SCRIPT_DIR}/ubuntu.sh" --no-sim-tools
else
	echo "错误: 未找到 Tools/setup/ubuntu.sh"
	exit 1
fi

# 2. 初始化/更新 Git 子模块（已改为 Gitee 镜像）
echo ""
echo "[2/3] 正在同步并拉取子模块 (Gitee)..."
git submodule sync --recursive
git submodule update --init --recursive

# 3. 编译 fmu-v5
echo ""
echo "[3/3] 正在编译 px4_fmu-v5_default ..."
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
make px4_fmu-v5_default

echo ""
echo "=========================================="
echo "编译完成。固件位置:"
echo "  build/px4_fmu-v5_default/px4_fmu-v5_default.elf"
echo "  build/px4_fmu-v5_default/px4_fmu-v5_default.px4"
echo "=========================================="
