package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.hid.HIDUtils;
import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Converts Xbox controller axes into chassis speeds for {@link DriveCommands}.
 *
 * <p>Axis helpers ({@link #translationFromAxes}, {@link #rotationFromAxis}) are package-visible for
 * unit tests without HAL. Alliance sign flips field-relative translation on red alliance.
 */
final class SwerveTeleopInputs {
  private SwerveTeleopInputs() {}

  /**
   * Reads left stick with deadband, cubic smoothing, and current max translational speed.
   *
   * @see SwerveDrive#maxTranslationalSpeedMetersPerSecond
   */
  static TranslationalInput readTranslation(
      CommandXboxController controller,
      double joystickDeadband,
      double joystickSensitivity) {
    return translationFromAxes(
        controller.getLeftY(),
        controller.getLeftX(),
        joystickDeadband,
        joystickSensitivity,
        SwerveDrive.maxTranslationalSpeedMetersPerSecond);
  }

  /** Reads right stick X as angular rate using {@link SwerveDrive#maxAngularSpeedRadiansPerSecond}. */
  static double readRotation(
      CommandXboxController controller,
      double joystickDeadband,
      double joystickSensitivity) {
    return rotationFromAxis(
        controller.getRightX(),
        joystickDeadband,
        joystickSensitivity,
        SwerveDrive.maxAngularSpeedRadiansPerSecond);
  }

  /**
   * Pure function version of {@link #readTranslation} for tests and simulation.
   *
   * @param leftY raw left Y axis (−1 forward on Xbox)
   * @param leftX raw left X axis
   */
  static TranslationalInput translationFromAxes(
      double leftY,
      double leftX,
      double joystickDeadband,
      double joystickSensitivity,
      double maxTranslationalSpeedMetersPerSecond) {
    double xSpeed = 0.0;
    double leftYInput = -leftY;
    if (Math.abs(leftYInput) > joystickDeadband) {
      xSpeed = HIDUtils.smoothInputWithCubic(leftYInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
    }

    double ySpeed = 0.0;
    double leftXInput = -leftX;
    if (Math.abs(leftXInput) > joystickDeadband) {
      ySpeed = HIDUtils.smoothInputWithCubic(leftXInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
    }

    return new TranslationalInput(xSpeed, ySpeed);
  }

  /** Pure function version of {@link #readRotation} for tests. */
  static double rotationFromAxis(
      double rightX,
      double joystickDeadband,
      double joystickSensitivity,
      double maxAngularSpeedRadiansPerSecond) {
    double rotationSpeed = 0.0;
    double rightXInput = -rightX;
    if (Math.abs(rightXInput) > joystickDeadband) {
      rotationSpeed = HIDUtils.smoothInputWithCubic(rightXInput, joystickSensitivity) * maxAngularSpeedRadiansPerSecond;
    }
    return rotationSpeed;
  }

  /** Uses {@link DriverStation#getAlliance()} to flip translation when field-relative on red. */
  static int fieldRelativeAllianceSign(BooleanSupplier fieldRelative) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return fieldRelativeAllianceSign(fieldRelative.getAsBoolean(), alliance);
  }

  /**
   * @return {@code -1} when field-relative and red alliance (so blue-oriented field coords feel correct),
   *     otherwise {@code 1}
   */
  static int fieldRelativeAllianceSign(boolean fieldRelative, Alliance alliance) {
    return fieldRelative && alliance == Alliance.Red ? -1 : 1;
  }

  /**
   * Robot-frame translational speeds after deadband and scaling.
   *
   * @param xMetersPerSecond forward (m/s)
   * @param yMetersPerSecond left strafe (m/s)
   */
  record TranslationalInput(double xMetersPerSecond, double yMetersPerSecond) {}
}
