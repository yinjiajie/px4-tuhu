#!/usr/bin/env bash
# 当 Gitee 上的 NuttX/NuttX-apps 拉取失败或内容不完整时，用 GitHub 拉取这两个子模块
# 在项目根目录执行: bash Tools/setup/submodule_fallback_nuttx_github.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PX4_ROOT}"

NUTTX_BRANCH="px4_firmware_nuttx-10.3.0+"
NUTTX_PATH="platforms/nuttx/NuttX/nuttx"
APPS_PATH="platforms/nuttx/NuttX/apps"

echo "=========================================="
echo "从 GitHub 拉取 NuttX 与 NuttX-apps 子模块"
echo "（用于 Gitee 不可用或分支缺失时）"
echo "=========================================="

# 若目录已有内容但缺少 Makefile，说明克隆不完整，先删空再拉
for p in "$NUTTX_PATH" "$APPS_PATH"; do
	if [ -d "$p" ] && [ ! -f "$p/Makefile" ] && [ ! -f "$p/.git" ]; then
		echo "移除不完整的目录: $p"
		rm -rf "$p"
	fi
done

# 取消这两个子模块的当前注册，改用 GitHub 临时拉取
git config --remove-section submodule."${NUTTX_PATH}" 2>/dev/null || true
git config --remove-section submodule."${APPS_PATH}" 2>/dev/null || true
rm -rf ".git/modules/${NUTTX_PATH}" ".git/modules/${APPS_PATH}" 2>/dev/null || true

# 若目录存在且是 git 子模块，deinit
git submodule deinit -f "$NUTTX_PATH" 2>/dev/null || true
git submodule deinit -f "$APPS_PATH" 2>/dev/null || true

# 用 GitHub 克隆到正确路径
if [ ! -f "${NUTTX_PATH}/Makefile" ]; then
	echo "[1/2] 克隆 NuttX (GitHub)..."
	git clone --depth 1 --branch "$NUTTX_BRANCH" https://github.com/PX4/NuttX.git "$NUTTX_PATH"
fi
if [ ! -f "${APPS_PATH}/Makefile" ]; then
	echo "[2/2] 克隆 NuttX-apps (GitHub)..."
	git clone --depth 1 --branch "$NUTTX_BRANCH" https://github.com/PX4/NuttX-apps.git "$APPS_PATH"
fi

echo ""
echo "NuttX 与 NuttX-apps 已就绪。可执行: make px4_fmu-v5_default"
echo "（请先 export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:\$PATH）"
