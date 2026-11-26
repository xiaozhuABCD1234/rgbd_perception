#!/usr/bin/env bash
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

git config core.hooksPath .githooks
chmod +x .githooks/commit-msg .githooks/pre-commit scripts/*.py scripts/*.sh

echo "Installed repository Git hooks (core.hooksPath=.githooks)."
