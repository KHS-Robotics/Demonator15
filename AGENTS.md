# Agent instructions (Demonator15)

## Build and verify

```bash
chmod +x gradlew
./gradlew test    # local verification
./gradlew build   # full compile + tests (matches GitHub Actions CI)
```

Cloud install (`.cursor/environment.json` → `cloud-install.sh`) installs OpenJDK 17 at `/usr/lib/jvm/java-17-openjdk-amd64` and compiles with `./gradlew classes`. There is no boot service; this is a Gradle robot project. Verify with `./gradlew test` and `./gradlew build`.

## Architecture

- WPILib **command-based** robot. Subsystems own hardware; `frc.robot.commands` holds command factories; `RobotContainer` binds controls and PathPlanner **named commands**.
- Swerve: `subsystems/drive/`, teleop commands in `commands/drive/` when present on branch.

## Cursor rules

See `.cursor/rules/*.mdc` for FRC-specific conventions.

## PR expectations

- Keep robot behavior changes scoped and describe driver/auto impact in the PR.
- Ensure CI (`build`) passes before merge.
