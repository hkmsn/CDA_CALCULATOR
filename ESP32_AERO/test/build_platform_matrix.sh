#!/usr/bin/env zsh

set -eu

PROJECT_DIR="${0:A:h:h}"
cd "$PROJECT_DIR"

environments=(
  m5stack-capsule-v1_1
  m5stack-stamps3-generic
  esp32-s3-devkitc-1
)

failed=0
for environment in $environments; do
  print "\n=== Building $environment ==="
  if ! pio run --environment "$environment"; then
    failed=1
  fi
done

if (( failed )); then
  print "\nPLATFORM MATRIX FAILED" >&2
  exit 1
fi

print "\nPLATFORM MATRIX PASSED"
