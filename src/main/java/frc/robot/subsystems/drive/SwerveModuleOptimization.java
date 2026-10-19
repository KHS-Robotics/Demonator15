package frc.robot.subsystems.drive;

/**
 * WPILib swerve module optimizations: shortest pivot path and cosine drive scaling.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html">Swerve drive kinematics</a>
 */
public final class SwerveModuleOptimization {
  private SwerveModuleOptimization() {}

  /**
   * Result of pivot angle optimization.
   *
   * @param angleDegrees pivot setpoint to track (may differ from requested angle by 180°)
   * @param driveInverted if true, wheel speed should be negated to follow the shorter pivot path
   */
  public record PivotTarget(double angleDegrees, boolean driveInverted) {}

  /**
   * Chooses pivot target and whether to run the drive motor inverted per WPILib module optimization.
   *
   * @param currentAngleDegrees measured pivot angle
   * @param targetAngleDegrees desired module heading from kinematics
   */
  public static PivotTarget shortestPivotTarget(double currentAngleDegrees, double targetAngleDegrees) {
    double targetAngle = targetAngleDegrees;
    double deltaAngle = Math.abs(targetAngle - currentAngleDegrees);
    boolean driveInverted = deltaAngle > 90 && deltaAngle < 270;

    if (driveInverted) {
      if (targetAngle > 0 || targetAngle == 0 && currentAngleDegrees < 0) {
        targetAngle -= 180;
      } else if (targetAngle < 0 || targetAngle == 0 && currentAngleDegrees > 0) {
        targetAngle += 180;
      }
    }

    return new PivotTarget(targetAngle, driveInverted);
  }

  /**
   * Cosine compensation factor in [0, 1] based on pivot error (reduces wheel speed while turning).
   */
  public static double cosineScale(double currentAngleDegrees, double targetAngleDegrees) {
    return Math.cos(Math.toRadians(Math.abs(currentAngleDegrees - targetAngleDegrees)));
  }
}
