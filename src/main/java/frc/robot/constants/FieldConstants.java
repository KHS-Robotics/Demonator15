package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Field layout values that change each FRC season (poses, landmarks, practice positions).
 *
 * <p>Keep game-specific numbers here instead of in {@link frc.robot.subsystems.drive.SwerveDrive} or
 * commands so they are easy to update year over year.
 */
public final class FieldConstants {
  private FieldConstants() {}

  /**
   * Pose used when the robot boots before vision or a driver reset. Update for the current field and
   * default auto start.
   */
  public static final Pose2d kDefaultOdometryPose = new Pose2d(8.5, 4.0, Rotation2d.kZero);

  /** Field X (m) for driver “reset heading” practice spot — see {@link frc.robot.commands.drive.DriveCommands#resetHeading}. */
  public static final double kDriverPracticeResetX = 1.016;
  /** Field Y (m) for driver practice reset. */
  public static final double kDriverPracticeResetY = 4.0;
}
