package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveKinematicsTest {
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      SwerveDriveConfig.kFrontLeftModuleOffset,
      SwerveDriveConfig.kFrontRightModuleOffset,
      SwerveDriveConfig.kRearLeftModuleOffset,
      SwerveDriveConfig.kRearRightModuleOffset);

  @Test
  void desaturateWheelSpeeds_limitsFastestWheelToMaxTranslationalSpeed() {
    // Arrange
    ChassisSpeeds corrected = SwerveDynamics.correctForDynamics(new ChassisSpeeds(10.0, 10.0, 0.0));
    double maxTranslational = SwerveDrive.maxTranslationalSpeedMetersPerSecond;

    // Act
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(corrected);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxTranslational);

    // Assert
    double fastestWheel = 0.0;
    for (SwerveModuleState state : states) {
      fastestWheel = Math.max(fastestWheel, state.speedMetersPerSecond);
    }
    assertEquals(maxTranslational, fastestWheel, 1e-9);
  }

  @Test
  void toChassisSpeeds_roundTripsRobotRelativeTranslation() {
    // Arrange
    ChassisSpeeds requested = new ChassisSpeeds(2.5, -0.5, 0.0);
    ChassisSpeeds corrected = SwerveDynamics.correctForDynamics(requested);

    // Act
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(corrected);
    ChassisSpeeds reconstructed = kinematics.toChassisSpeeds(states);

    // Assert
    assertEquals(corrected.vxMetersPerSecond, reconstructed.vxMetersPerSecond, 1e-6);
    assertEquals(corrected.vyMetersPerSecond, reconstructed.vyMetersPerSecond, 1e-6);
    assertEquals(corrected.omegaRadiansPerSecond, reconstructed.omegaRadiansPerSecond, 1e-6);
  }

  @Test
  void fromFieldRelativeSpeeds_rotatesTranslationWithRobotHeading() {
    // Arrange — same transform {@link SwerveDrive#drive} uses when field-relative is enabled
    Rotation2d heading = Rotation2d.fromDegrees(90);

    // Act
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(3.0, 0.0, 0.0, heading);

    // Assert
    assertEquals(0.0, robotRelative.vxMetersPerSecond, 1e-6);
    assertEquals(-3.0, robotRelative.vyMetersPerSecond, 1e-6);
    assertEquals(0.0, robotRelative.omegaRadiansPerSecond, 1e-6);
  }
}
