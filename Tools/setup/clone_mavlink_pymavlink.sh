#!/usr/bin/env bash
# 单独拉取 mavlink 的嵌套子模块 pymavlink（解决 mavgen.py missing）
# 在项目根目录执行: bash Tools/setup/clone_mavlink_pymavlink.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MAVLINK_DIR="${PX4_ROOT}/src/modules/mavlink/mavlink"
PYMAVLINK_DIR="${MAVLINK_DIR}/pymavlink"

cd "${PX4_ROOT}"

if [ ! -d "$MAVLINK_DIR" ]; then
	echo "错误: 未找到 mavlink 目录 $MAVLINK_DIR，请先运行 clone_submodules_standalone.sh"
	exit 1
fi

if [ -f "${PYMAVLINK_DIR}/tools/mavgen.py" ]; then
	echo "pymavlink 已存在且含 mavgen.py，无需拉取。"
	exit 0
fi

echo "拉取 mavlink 的嵌套子模块 pymavlink ..."
if [ -d "$PYMAVLINK_DIR/.git" ] || [ -f "$PYMAVLINK_DIR/.git" ]; then
	(cd "$MAVLINK_DIR" && git submodule update --init --recursive) || true
fi
if [ ! -f "${PYMAVLINK_DIR}/tools/mavgen.py" ]; then
	echo "使用 git submodule 未成功，改为直接 clone pymavlink ..."
	rm -rf "$PYMAVLINK_DIR"
	git clone --depth 1 https://gitee.com/px4-autopilot_v1-14/pymavlink.git "$PYMAVLINK_DIR"
fi

if [ -f "${PYMAVLINK_DIR}/tools/mavgen.py" ]; then
	echo "完成。pymavlink/tools/mavgen.py 已就绪。"
else
	echo "失败：未找到 pymavlink/tools/mavgen.py"
	exit 1
fi
