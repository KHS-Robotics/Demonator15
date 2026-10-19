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

# FRC desktop tools (Flutter/GTK) for cloud Desktop tab: PathPlanner + Elastic (WPILib NT).
install_frc_desktop_tools() {
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    curl unzip libegl1 libgl1 ca-certificates

  local pp_version="2026.1.2"
  local elastic_version="2026.1.2"

  if [[ ! -x /opt/pathplanner/pathplanner ]]; then
    curl -fsSL -o /tmp/PathPlanner-Linux.zip \
      "https://github.com/mjansen4857/pathplanner/releases/download/v${pp_version}/PathPlanner-Linux-v${pp_version}.zip"
    sudo rm -rf /opt/pathplanner
    sudo unzip -o /tmp/PathPlanner-Linux.zip -d /opt/pathplanner
    sudo chmod +x /opt/pathplanner/pathplanner
  fi

  if [[ ! -x /opt/elastic/elastic_dashboard ]]; then
    curl -fsSL -o /tmp/Elastic-WPILib-Linux.zip \
      "https://github.com/Gold872/elastic_dashboard/releases/download/v${elastic_version}/Elastic-WPILib-Linux.zip"
    sudo rm -rf /opt/elastic
    sudo unzip -o /tmp/Elastic-WPILib-Linux.zip -d /opt/elastic
    sudo chmod +x /opt/elastic/elastic_dashboard
  fi

  sudo ln -sf /opt/pathplanner/pathplanner /usr/local/bin/pathplanner
  sudo ln -sf /opt/elastic/elastic_dashboard /usr/local/bin/elastic
}

install_frc_desktop_tools
