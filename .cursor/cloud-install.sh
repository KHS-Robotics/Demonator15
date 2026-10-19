#!/usr/bin/env bash
# Cloud agent Build: prepare Demonator15 (idempotent — safe on every Build).
set -euo pipefail
cd "$(dirname "$0")/.."
chmod +x gradlew
# Unit tests use Java 17 via Gradle toolchain (build.gradle). Full roboRIO cross-build runs in GitHub Actions CI.
./gradlew test --no-daemon
