#!/usr/bin/env bash
# Runs a PowerShell job in the tiny10 VM's console window, where it can be watched from the VM's
# desktop, and streams its output back here. The job is the file named, or standard input.
#
#   tools/win10/vm-run.sh <label> [job.ps1]
#   tools/win10/vm-run.sh git-status <<'EOF'
#   Set-Location C:\src\MissionPlannerRust; git status --short
#   EOF
#
# The window is console-runner.ps1, started at logon by the ClaudeConsole task that
# install-console.ps1 registers. Exits with the job's exit code; 125 when the window is not
# running (nobody logged on at the VM). VM_SSH_PORT and VM_SSH_TARGET override the SSH address.
set -euo pipefail
port=${VM_SSH_PORT:-2222}
target=${VM_SSH_TARGET:-user@localhost}
label=${1:?usage: tools/win10/vm-run.sh <label> [job.ps1]}
job=${2:-/dev/stdin}
name="$(date +%Y%m%d-%H%M%S)-$$-${label//[^A-Za-z0-9_-]/_}"
tmp=$(mktemp --suffix=.ps1)
trap 'rm -f "$tmp"' EXIT
# Windows PowerShell 5.1 reads a script without a byte-order mark as the ANSI code page.
printf '\xef\xbb\xbf' > "$tmp"
cat "$job" >> "$tmp"
scp -q -P "$port" -o BatchMode=yes "$tmp" "$target:C:/setup/queue/$name.part"
exec ssh -p "$port" -o BatchMode=yes "$target" \
    "powershell -NoProfile -ExecutionPolicy Bypass -File C:\\setup\\console-wait.ps1 -Name $name; exit \$LASTEXITCODE"
