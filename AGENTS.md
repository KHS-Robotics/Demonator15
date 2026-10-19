# Agent instructions (Demonator15)

## Build and verify

```bash
chmod +x gradlew
./gradlew test    # local / Cursor Cloud install script
./gradlew build   # full compile + tests (matches GitHub Actions CI)
```

Cloud environment config: `.cursor/environment.json` (runs `cloud-install.sh` → `./gradlew test`).

## Architecture

- WPILib **command-based** robot. Subsystems own hardware; `frc.robot.commands` holds command factories; `RobotContainer` binds controls and PathPlanner **named commands**.
- Swerve: `subsystems/drive/`, teleop commands in `commands/drive/` when present on branch.

## Cursor rules

See `.cursor/rules/*.mdc` for FRC-specific conventions.

## PR expectations

- Keep robot behavior changes scoped and describe driver/auto impact in the PR.
- Ensure CI (`build`) passes before merge.
