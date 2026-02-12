#!/usr/bin/env bash
# 强制按 .gitmodules（Gitee 等）重新初始化所有子模块，解决 path 未匹配、目录为空等问题
# 在项目根目录执行: bash Tools/setup/submodule_init_gitee.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PX4_ROOT}"

echo "=========================================="
echo "强制重新初始化 Git 子模块"
echo "项目路径: ${PX4_ROOT}"
echo "=========================================="

# NuttX/nuttx 与 NuttX/apps 若存在但缺少 Makefile，说明克隆不完整，删掉以便重新克隆
for p in platforms/nuttx/NuttX/nuttx platforms/nuttx/NuttX/apps; do
	if [ -d "${PX4_ROOT}/$p" ] && [ ! -f "${PX4_ROOT}/$p/Makefile" ]; then
		echo "移除不完整目录以便重新克隆: $p"
		rm -rf "${PX4_ROOT}/$p"
	fi
done

# 若子模块曾用其他 URL 初始化过，先取消注册，再按当前 .gitmodules 重新来
echo "[1/3] 取消已有子模块注册（保留 .gitmodules）..."
git submodule deinit -f --all 2>/dev/null || true

echo "[2/3] 同步子模块 URL（.gitmodules -> .git/config）..."
git submodule sync --recursive

echo "[3/3] 拉取并检出所有子模块（NuttX/NuttX-apps 使用 GitHub，其余用 Gitee）..."
git submodule update --init --recursive

echo ""
echo "子模块初始化完成。可执行: make px4_fmu-v5_default"
echo "（请先 export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:\$PATH）"
