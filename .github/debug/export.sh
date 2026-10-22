#!/usr/bin/env bash
# Temporary debugging aid for the platform-dependent runtime of
# VSSGenMBInfoSolver.Default7: runs the test with a gurobi.env in its working
# directory (test/) so that Gurobi writes the model and the variable hints
# exactly as this platform builds them. A time limit of one second suffices,
# since only the model is of interest.
#
# Usage: export.sh <model_tag>
set -u
tag=$1
out="$PWD/debug-model"
mkdir -p "$out"
# Gurobi is a native binary, hence on Windows it needs a Windows path
gurobi_out=$out
if command -v cygpath >/dev/null 2>&1; then
  gurobi_out=$(cygpath -m "$out")
fi

for ext in mps hnt; do
  printf 'TimeLimit 1\nResultFile %s\n' "$gurobi_out/$tag.$ext" >test/gurobi.env
  ctest -C RelWithDebInfo --test-dir build -R '^VSSGenMBInfoSolver\.Default7$' --output-on-failure || true
done
rm -f test/gurobi.env
ls -la "$out"
