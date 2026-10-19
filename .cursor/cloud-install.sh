#!/usr/bin/env bash
# Cloud agent install for Demonator15. Idempotent — safe on every build.
# Prepares the WPILib Java 17 toolchain and compiles robot code.
# Unit tests and the full jar stay verification steps (`./gradlew test`, `./gradlew build`).
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -x /usr/lib/jvm/java-17-openjdk-amd64/bin/java ]]; then
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y openjdk-17-jdk
fi

export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
export PATH="${JAVA_HOME}/bin:${PATH}"

chmod +x gradlew
./gradlew classes --no-daemon
