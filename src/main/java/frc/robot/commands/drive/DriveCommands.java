package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.SwerveDrive;

/**
 * Factory methods for swerve {@link Command}s bound in {@link frc.robot.RobotContainer}.
 *
 * <p>Keeps {@link SwerveDrive} focused on hardware and control loops; commands encode operator
 * intent (teleop, defense, pose reset, etc.).
 */
public final class DriveCommands {
  private static final double kSlowTranslationalSpeedMetersPerSecond = 0.5;
  private static final double kSlowAngularSpeedRadiansPerSecond = Math.toRadians(90);
  private static final double kNormalTranslationalSpeedMetersPerSecond = 4.6;
  private static final double kNormalAngularSpeedRadiansPerSecond = Math.toRadians(540);

  private DriveCommands() {}

  /**
   * Default teleop drive: left stick translation, right stick rotation, optional field-relative.
   *
   * @param swerve drivetrain subsystem
   * @param controller driver Xbox controller
   * @param fieldRelative when true, translation is field-oriented
   * @param joystickDeadband axis magnitude treated as zero below this value
   * @param joystickSensitivity cubic blend [0, 1]; 0 = linear, 1 = full cubic
   * @return command that runs until interrupted
   */
  public static Command driveWithXboxController(
      SwerveDrive swerve,
      CommandXboxController controller,
      BooleanSupplier fieldRelative,
      double joystickDeadband,
      double joystickSensitivity) {
    return swerve.runEnd(
        () -> {
          var translation = SwerveTeleopInputs.readTranslation(controller, joystickDeadband, joystickSensitivity);
          double rotationSpeed = SwerveTeleopInputs.readRotation(controller, joystickDeadband, joystickSensitivity);
          int sign = SwerveTeleopInputs.fieldRelativeAllianceSign(fieldRelative);
          swerve.drive(sign * translation.xMetersPerSecond(), sign * translation.yMetersPerSecond(), rotationSpeed,
              fieldRelative.getAsBoolean());
        },
        swerve::stop)
        .withName("DriveWithXboxController");
  }

  /**
   * Captures heading on start, then translates while PID holds that heading (defense / bump alignment).
   */
  public static Command holdCurrentHeadingWhileDriving(
      SwerveDrive swerve,
      CommandXboxController controller,
      BooleanSupplier fieldRelative,
      double joystickDeadband,
      double joystickSensitivity) {
    Rotation2d[] headingToHold = new Rotation2d[1];
    Command setHeading = swerve.runOnce(() -> headingToHold[0] = swerve.getPose().getRotation());
    Command holdHeading = swerve.runEnd(
        () -> {
          var translation = SwerveTeleopInputs.readTranslation(controller, joystickDeadband, joystickSensitivity);
          int sign = SwerveTeleopInputs.fieldRelativeAllianceSign(fieldRelative);
          swerve.holdAngleWhileDriving(
              sign * translation.xMetersPerSecond(),
              sign * translation.yMetersPerSecond(),
              headingToHold[0],
              1.0,
              fieldRelative.getAsBoolean());
        },
        swerve::stop);
    return Commands.sequence(setHeading, holdHeading).withName("SwerveHoldCurrentHeadingWhileDriving");
  }

  /**
   * Temporarily lowers {@link SwerveDrive#maxTranslationalSpeedMetersPerSecond} and max angular speed
   * while held (e.g. fine alignment when elevated).
   */
  public static Command goSlow(SwerveDrive swerve) {
    return Commands.startEnd(
        () -> {
          SwerveDrive.maxTranslationalSpeedMetersPerSecond = kSlowTranslationalSpeedMetersPerSecond;
          SwerveDrive.maxAngularSpeedRadiansPerSecond = kSlowAngularSpeedRadiansPerSecond;
        },
        () -> {
          SwerveDrive.maxTranslationalSpeedMetersPerSecond = kNormalTranslationalSpeedMetersPerSecond;
          SwerveDrive.maxAngularSpeedRadiansPerSecond = kNormalAngularSpeedRadiansPerSecond;
        })
        .withName("SwerveGoSlow");
  }

  /**
   * Resets odometry pose to ({@code x}, {@code y}) with heading 0° (blue alliance forward).
   *
   * @param xMeters field X in meters
   * @param yMeters field Y in meters
   */
  public static Command resetHeading(SwerveDrive swerve, DoubleSupplier xMeters, DoubleSupplier yMeters) {
    return swerve.runOnce(() -> swerve.resetPose(
        new Pose2d(xMeters.getAsDouble(), yMeters.getAsDouble(), Rotation2d.kZero)))
        .withName("ResetRobotHeading");
  }

  /** One-shot {@link SwerveDrive#stop()}; registered as PathPlanner named command {@code STOPSwerve}. */
  public static Command stop(SwerveDrive swerve) {
    return swerve.runOnce(swerve::stop).withName("StopSwerve");
  }

  /**
   * Holds modules in an X configuration ({@link SwerveDrive#lock()}) while active, then stops on end.
   */
  public static Command lock(SwerveDrive swerve) {
    return swerve.runEnd(swerve::lock, swerve::stop).withName("SwerveDriveLock");
  }
}
