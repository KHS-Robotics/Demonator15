# Agent instructions (Demonator15)

## Build and verify

```bash
chmod +x gradlew
./gradlew test    # local verification
./gradlew build   # full compile + tests (matches GitHub Actions CI)
```

CI runs the same `build` task in the WPILib cross container. Use Java **17** (Gradle toolchain in `build.gradle`).

## Cloud agent environment

Config is **as code**: `.cursor/environment.json` runs `.cursor/cloud-install.sh`, which:

- Installs OpenJDK 17 and runs `./gradlew classes`
- Installs **PathPlanner** (`pathplanner`, 2026.1.2) and **Elastic** WPILib (`elastic`, 2026.1.2) for the agent **Desktop** tab

There is no long-running boot service; this is a Gradle robot project. After changing `cloud-install.sh`, merge to `main` and refresh the team environment **build**.

**Desktop sim workflow (optional verification):**

1. `./gradlew simulateJava` — enable the robot in Driver Station sim
2. `elastic` — connect to NT; import layout from `src/main/deploy/elastic/elastic-layout.json`
3. `pathplanner` — **Open Robot Project** at the repo root (`src/main/deploy/pathplanner/`)

Vendor REV/Phoenix libraries come from Gradle `vendordeps` for compile/sim; no separate vendor desktop tools are installed in the cloud image.

## Architecture

WPILib **command-based** layout:

| Layer | Location | Responsibility |
| --- | --- | --- |
| Robot / container | `Robot.java`, `RobotContainer.java` | Scheduler, subsystem instances, bindings, PathPlanner named commands |
| Subsystems | `subsystems/**` | Hardware, `periodic()`, imperative APIs (`SwerveDrive.drive()`, setpoints, odometry) |
| Commands | `commands/**` | Command factories (`DriveCommands`, mechanism commands) — no CAN IDs here |
| Constants | `constants/`, `*Config`, `RobotMap` | Field poses (`FieldConstants`), tuning, CAN IDs |
| Deploy | `src/main/deploy/` | PathPlanner paths/autos, Elastic layout |

**Swerve (current structure):**

- `subsystems/drive/` — `SwerveDrive`, modules, `SwerveDynamics`, `SwerveModuleOptimization`, `SwerveDriveConfig`
- `commands/drive/` — `DriveCommands`, `SwerveTeleopInputs` (teleop, go-slow, heading hold, reset pose, stop)
- PathPlanner `AutoBuilder` stays on `SwerveDrive`; do not break `getPose`, `resetPose`, `getChassisSpeeds`, or the module-state consumer without updating PathPlanner.

## Tests

- Run `./gradlew test` after drive or command changes.
- Pure logic: `SwerveDynamics`, `SwerveModuleOptimization`, `SwerveTeleopInputs`, `AngleUtils` (see `.cursor/rules/frc-unit-tests.mdc`).
- Do not construct full `SwerveDrive` in unit tests (`RobotContainer` static init); use package-visible helpers or module-level sim tests.

## Cursor rules

See `.cursor/rules/*.mdc` for FRC-specific conventions (subsystems vs commands, PathPlanner naming, unit tests).

## PR expectations

- Keep robot behavior changes scoped; describe **driver** and **auto** impact in the PR.
- PathPlanner named command strings in Java must **exactly** match the PathPlanner GUI.
- Ensure CI (`build`) passes before merge.
