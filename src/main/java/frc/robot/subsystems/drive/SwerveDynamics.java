package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Second-order kinematics correction for swerve (curvature-induced translation skew).
 *
 * <p>Based on Team 254 and the Chief Delphi
 * <a href="https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964">whitepaper</a>.
 * Applied in {@link SwerveDrive} before {@link edu.wpi.first.math.kinematics.SwerveDriveKinematics}
 * converts chassis speeds to module states.
 */
public final class SwerveDynamics {
  private static final double kDefaultLoopTimeSeconds = 0.02;

  private SwerveDynamics() {}

  /**
   * Corrects chassis speeds for one 20 ms loop using the default control period.
   *
   * @param originalSpeeds desired speeds before skew correction
   * @return adjusted speeds to send to module kinematics
   */
  public static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    return correctForDynamics(originalSpeeds, kDefaultLoopTimeSeconds);
  }

  /**
   * @param loopTimeSeconds integration period (s), typically {@code 0.02}
   */
  static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds, double loopTimeSeconds) {
    Pose2d futureRobotPose = new Pose2d(
        originalSpeeds.vxMetersPerSecond * loopTimeSeconds,
        originalSpeeds.vyMetersPerSecond * loopTimeSeconds,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * loopTimeSeconds));
    Twist2d twistForPose = poseLog(futureRobotPose);
    return new ChassisSpeeds(
        twistForPose.dx / loopTimeSeconds,
        twistForPose.dy / loopTimeSeconds,
        twistForPose.dtheta / loopTimeSeconds);
  }

  /** Lie algebra log map for a small pose delta (internal; covered by unit tests). */
  static Twist2d poseLog(Pose2d transform) {
    final double epsilon = 1E-9;
    final double dTheta = transform.getRotation().getRadians();
    final double halfDTheta = 0.5 * dTheta;
    final double cosMinusOne = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halfThetaByTanOfHalfDTheta;
    if (Math.abs(cosMinusOne) < epsilon) {
      halfThetaByTanOfHalfDTheta = 1.0 - 1.0 / 12.0 * dTheta * dTheta;
    } else {
      halfThetaByTanOfHalfDTheta = -(halfDTheta * Math.sin(transform.getRotation().getRadians())) / cosMinusOne;
    }
    Translation2d translationPart = transform.getTranslation()
        .rotateBy(new Rotation2d(halfThetaByTanOfHalfDTheta, -halfDTheta));
    return new Twist2d(translationPart.getX(), translationPart.getY(), dTheta);
  }
}
