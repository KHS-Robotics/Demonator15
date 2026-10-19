# Agent instructions (Demonator15)

## Build and verify

```bash
chmod +x gradlew
./gradlew test    # local verification
./gradlew build   # full compile + tests (matches GitHub Actions CI)
```

CI uses the same `build` task (WPILib cross container, Java 17 per `build.gradle`).

## Cloud agent environment

Environment is **config as code**: `.cursor/environment.json` runs `.cursor/cloud-install.sh` (JDK, `./gradlew classes`, and desktop **PathPlanner** / **Elastic** on the agent VM for the Desktop tab). There is no boot service. After changing the install script, merge to `main` and run a new team environment **build**.

For manual checks on Desktop: start WPILib sim (`./gradlew simulateJava`), enable the robot in Driver Station sim, then use `elastic` (NT dashboard) and `pathplanner` (open the **repo root** as the robot project). Deploy assets live under `src/main/deploy/`. Vendor libraries for sim/build come from Gradle `vendordeps`, not separate vendor desktop installers.

## Architecture

WPILib **command-based** robot:

- **`Robot` / `RobotContainer`** — subsystem instances, default commands, button bindings, PathPlanner named commands.
- **`subsystems/`** — hardware, `periodic()`, and imperative control APIs (no Xbox wiring when avoidable).
- **`commands/`** — command factories grouped by mechanism (e.g. drive utilities under `commands/drive/`).
- **`hid/`** — driver HID wrappers (`DemonCommandXboxController`, `OperatorStick`), shared axis math in `HIDUtils`; put stick/button semantics here, not in commands.
- **`constants/`**, **`*Config`**, **`RobotMap`** — field/season poses, tuning, CAN IDs.
- **`src/main/deploy/`** — PathPlanner paths/autos, Elastic layout, other deploy files.

**Swerve:** drivetrain code under `subsystems/drive/`; operator-facing drive commands under `commands/drive/`. PathPlanner autos integrate through `SwerveDrive`’s pose and chassis-speed APIs—keep that contract stable when refactoring.

## Tests and conventions

Run `./gradlew test` (or `build`) when changing behavior. Use **Arrange / Act / Assert** section comments in tests (`// Arrange`, `// Act`, `// Assert`); details in `.cursor/rules/frc-unit-tests.mdc`. Other FRC conventions: `.cursor/rules/*.mdc`.

## PR expectations

- Scope changes narrowly; note **driver** and **auto** impact.
- PathPlanner named commands in Java must match the PathPlanner GUI strings exactly.
- CI `build` should pass before merge.
