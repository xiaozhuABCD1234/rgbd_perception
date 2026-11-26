#!/usr/bin/env bash
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

git config core.hooksPath .githooks
git config commit.template .gitmessage
chmod +x .githooks/commit-msg .githooks/pre-commit scripts/*.py scripts/*.sh

echo "Installed repository Git hooks (core.hooksPath=.githooks)."
echo "Commit template configured from .gitmessage."
