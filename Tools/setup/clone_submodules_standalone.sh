#!/usr/bin/env bash
# 不依赖 git submodule 命令，直接按 .gitmodules 用 git clone 拉取所有子模块
# 当 git submodule update --init 无效果时使用。在项目根目录执行: bash Tools/setup/clone_submodules_standalone.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${PX4_ROOT}"

GITMODULES="${PX4_ROOT}/.gitmodules"
if [ ! -f "$GITMODULES" ]; then
	echo "错误: 未找到 .gitmodules"
	exit 1
fi

echo "=========================================="
echo "按 .gitmodules 直接 clone 子模块（不依赖 git submodule）"
echo "项目路径: ${PX4_ROOT}"
echo "=========================================="

# 从 .gitmodules 解析出 submodule 名称列表（即 path 的 key 名，与 config 一致）
get_submodule_names() {
	awk '
		/^\[submodule "/ {
			sub(/^\[submodule "/, "")
			sub(/"\]$/, "")
			print $0
		}
	' "$GITMODULES"
}

clone_one() {
	local name="$1"
	local path url branch
	path=$(git config -f "$GITMODULES" submodule."$name".path)
	url=$(git config -f "$GITMODULES" submodule."$name".url)
	branch=$(git config -f "$GITMODULES" submodule."$name".branch 2>/dev/null || true)

	[ -z "$path" ] || [ -z "$url" ] && return 0
	path="${PX4_ROOT}/${path}"

	# 已存在且是有效 git 仓库则跳过
	if [ -d "$path/.git" ] || [ -f "$path/.git" ]; then
		echo "  跳过（已存在）: $path"
		return 0
	fi

	# 目录存在但为空或不完整则删除后重新 clone
	if [ -d "$path" ]; then
		echo "  移除不完整目录: $path"
		rm -rf "$path"
	fi

	echo "  拉取: $path"
	mkdir -p "$(dirname "$path")"
	if [ -n "$branch" ]; then
		git clone --depth 1 -b "$branch" "$url" "$path" || {
			echo "  分支 $branch 拉取失败，尝试默认分支"
			git clone --depth 1 "$url" "$path"
		}
	else
		git clone --depth 1 "$url" "$path"
	fi

	# 若该子模块自身也有 .gitmodules（如 mavlink 含 pymavlink），拉取嵌套子模块
	if [ -f "$path/.gitmodules" ]; then
		echo "  拉取嵌套子模块: $path"
		(cd "$path" && git submodule update --init --recursive) || true
	fi
}

total=0
done=0
while read -r name; do
	[ -z "$name" ] && continue
	total=$((total + 1))
	echo "[$total] $name"
	clone_one "$name" && done=$((done + 1))
	echo ""
done < <(get_submodule_names)

echo "=========================================="
echo "完成。已处理 $done 个子模块。"
echo "可执行: export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:\$PATH && make px4_fmu-v5_default"
echo "=========================================="
