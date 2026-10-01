#!/usr/bin/env bash
# 构建工作区。首次构建或改动 C++ 后运行。
#
#   scripts/build.sh                  # 安全限流构建（总并发 2，推荐默认）
#   scripts/build.sh --clean          # 先清空 build/install/log 再全量构建
#   scripts/build.sh -j4              # 放宽到总并发 4（机器空闲时）
#   scripts/build.sh camera vins      # 只构建指定包
#   scripts/build.sh -- -j8           # 显式指定并发上限（不建议本机用）
#
# 为什么默认限流：本机 32 核但只有 14G 内存。colcon 默认按 CPU 核数起并行任务，
# 且每个包内 make 又按核数展开（见 colcon_cmake 的 _get_make_arguments），
# 12 个包同时编译会瞬间打满内存导致机器卡死。
# 这里用「单包串行 + 包内 make 限并发」把总编译并发压到 -j 的值。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

JOBS=2
CLEAN=0
PACKAGES=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean) CLEAN=1; shift ;;
        -j*)    JOBS="${1#-j}"; shift ;;
        --)     shift; PACKAGES+=("$@"); break ;;
        -*)     echo "未知选项: $1" >&2; exit 1 ;;
        *)      PACKAGES+=("$1"); shift ;;
    esac
done

if [[ "$CLEAN" -eq 1 ]]; then
    echo "==> 清理 build/ install/ log/"
    rm -rf build install log
fi

if ! command -v colcon >/dev/null 2>&1; then
    source /opt/ros/humble/setup.bash
fi

echo "==> colcon build（包间并发 1，包内 -j${JOBS}）${PACKAGES[*]:-}"
MAKEFLAGS="-j${JOBS} -l${JOBS}" colcon build \
    --symlink-install \
    --parallel-workers 1 \
    "${PACKAGES[@]+--packages-select}" "${PACKAGES[@]+${PACKAGES[@]}}"

echo
echo "==> 构建完成。source install/setup.bash 后即可 ros2 launch launch/simulation.launch.py"