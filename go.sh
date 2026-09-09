#!/usr/bin/env bash
cd "$HOME/safe_landing"
setsid bash run_headless.sh < /dev/null > logs/launcher.log 2>&1 &
disown
echo "GO_DISPATCHED"
