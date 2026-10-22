#!/usr/bin/env bash
# Temporary debugging aid for the platform-dependent runtime of
# VSSGenMBInfoSolver.Default7: solves an exported model with gurobi_cl, using
# the same non-default parameters and hints as the test, and reports the key
# log lines to the job summary.
#
# Usage: replay.sh <gurobi_cl> <model_dir> <model_tag> <runner_tag> <time_limit>
set -u
gurobi_cl=$1
dir=$2
model=$3
runner=$4
time_limit=$5
log="$dir/replay-$model-on-$runner.log"

if [ ! -f "$dir/$model.mps" ] || [ ! -f "$dir/$model.hnt" ]; then
  echo "::warning title=Default7 replay::model $model missing in $dir"
  exit 0
fi

start=$(date +%s)
"$gurobi_cl" NonConvex=2 "TimeLimit=$time_limit" "InputFile=$dir/$model.hnt" "LogFile=$log" \
  "$dir/$model.mps" >/dev/null 2>&1 || true
wall=$(($(date +%s) - start))

line() { grep -m1 -E "$1" "$log" | tr -d '\r' | sed -E "s/^$1//"; }
cpu=$(line "CPU model: ")
threads=$(line "Thread count: ")
fingerprint=$(line "Model fingerprint: ")
presolved=$(line "Presolved: ")
root=$(line "Root relaxation: objective ")
explored=$(grep -E "^Explored" "$log" | tail -1 | tr -d '\r')
best=$(grep -E "^Best objective" "$log" | tail -1 | tr -d '\r')
incumbents=$(grep -cE "^[H*] " "$log")

{
  echo "#### Default7 model \`$model\` solved on \`$runner\` (${wall}s wall)"
  echo "- CPU: $cpu; threads: $threads"
  echo "- fingerprint: $fingerprint; presolved: $presolved"
  echo "- root relaxation: $root"
  echo "- heuristic/branching incumbents: $incumbents"
  echo "- $explored"
  echo "- $best"
  echo
} >>"$GITHUB_STEP_SUMMARY"
echo "::notice title=Default7 $model on $runner::${wall}s | $fingerprint | root $root | $explored | $best | CPU $cpu"
