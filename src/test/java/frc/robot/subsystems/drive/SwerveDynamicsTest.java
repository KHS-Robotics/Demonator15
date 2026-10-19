package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDynamicsTest {
  @Test
  void correctForDynamics_pureTranslation_isUnchanged() {
    // Arrange
    ChassisSpeeds input = new ChassisSpeeds(2.0, -1.0, 0.0);

    // Act
    ChassisSpeeds output = SwerveDynamics.correctForDynamics(input);

    // Assert
    assertEquals(input.vxMetersPerSecond, output.vxMetersPerSecond, 1e-9);
    assertEquals(input.vyMetersPerSecond, output.vyMetersPerSecond, 1e-9);
    assertEquals(input.omegaRadiansPerSecond, output.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void correctForDynamics_rotationOnly_preservesOmega() {
    // Arrange
    ChassisSpeeds input = new ChassisSpeeds(0.0, 0.0, Math.PI);

    // Act
    ChassisSpeeds output = SwerveDynamics.correctForDynamics(input);

    // Assert
    assertEquals(0.0, output.vxMetersPerSecond, 1e-9);
    assertEquals(0.0, output.vyMetersPerSecond, 1e-9);
    assertEquals(input.omegaRadiansPerSecond, output.omegaRadiansPerSecond, 1e-6);
  }

  @Test
  void correctForDynamics_curvedMotion_adjustsTranslation() {
    // Arrange
    ChassisSpeeds input = new ChassisSpeeds(3.0, 0.0, Math.PI / 2.0);

    // Act
    ChassisSpeeds output = SwerveDynamics.correctForDynamics(input);

    // Assert
    assertEquals(Math.PI / 2.0, output.omegaRadiansPerSecond, 1e-6);
    assertNotEquals(input.vyMetersPerSecond, output.vyMetersPerSecond, 1e-6);
    assertEquals(input.vxMetersPerSecond, output.vxMetersPerSecond, 1e-2);
  }

  @Test
  void poseLog_zeroTransform_isZeroTwist() {
    // Act
    Twist2d twist = SwerveDynamics.poseLog(Pose2d.kZero);

    // Assert
    assertEquals(0.0, twist.dx, 1e-9);
    assertEquals(0.0, twist.dy, 1e-9);
    assertEquals(0.0, twist.dtheta, 1e-9);
  }

  @Test
  void poseLog_pureTranslation_matchesTranslation() {
    // Arrange
    Pose2d delta = new Pose2d(0.5, -0.25, Rotation2d.kZero);

    // Act
    Twist2d twist = SwerveDynamics.poseLog(delta);

    // Assert
    assertEquals(0.5, twist.dx, 1e-6);
    assertEquals(-0.25, twist.dy, 1e-6);
    assertEquals(0.0, twist.dtheta, 1e-9);
  }

  @Test
  void poseLog_smallRotation_usesStableApproximation() {
    // Arrange — triggers epsilon branch in poseLog
    Pose2d delta = new Pose2d(0.1, 0.0, Rotation2d.fromRadians(1e-10));

    // Act
    Twist2d twist = SwerveDynamics.poseLog(delta);

    // Assert
    assertEquals(0.1, twist.dx, 1e-6);
    assertEquals(0.0, twist.dy, 1e-6);
  }
}
