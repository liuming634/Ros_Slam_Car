#!/usr/bin/env bash
# 清理构建产物与运行时输出。不会碰源码、配置、地图。
#
#   scripts/clean.sh            # 清 build/ install/ log/ + assets/runtime 内容
#   scripts/clean.sh --all      # 额外清空 assets/maps（地图不可恢复，慎用）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "==> 清理 build/ install/ log/"
rm -rf build install log

echo "==> 清理 assets/runtime/*"
find assets/runtime -mindepth 1 -not -name '.gitkeep' -delete 2>/dev/null || true

if [[ "${1:-}" == "--all" ]]; then
    echo "==> 清理 assets/maps/*（地图将丢失）"
    find assets/maps -mindepth 1 -not -name '.gitkeep' -delete 2>/dev/null || true
fi

echo "==> 完成。重新构建：scripts/build.sh"