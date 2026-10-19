package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

class SwerveDriveMotionTest {
  private PIDController thetaPid;

  @BeforeEach
  void setUpPid() {
    thetaPid = new PIDController(
        SwerveDriveConfig.DRIVE_ANGLE_P,
        SwerveDriveConfig.DRIVE_ANGLE_I,
        SwerveDriveConfig.DRIVE_ANGLE_D);
    thetaPid.enableContinuousInput(-180.0, 180.0);
  }

  @Test
  void shouldCoastSwerveModules_allBelowDeadband_isTrue() {
    assertTrue(SwerveDrive.shouldCoastSwerveModules(0.0, 0.0, 0.0));
    assertTrue(SwerveDrive.shouldCoastSwerveModules(0.004, -0.004, 0.004));
  }

  @Test
  void shouldCoastSwerveModules_anyAxisAboveDeadband_isFalse() {
    assertFalse(SwerveDrive.shouldCoastSwerveModules(0.006, 0.0, 0.0));
    assertFalse(SwerveDrive.shouldCoastSwerveModules(0.0, -0.006, 0.0));
    assertFalse(SwerveDrive.shouldCoastSwerveModules(0.0, 0.0, 0.006));
  }

  @Test
  void computeHeadingHoldOmegaRadians_headingError_producesCorrectSign() {
    // Arrange — robot at 0°, want 90°
    double maxOmega = Math.toRadians(540);

    // Act
    double omega = SwerveDrive.computeHeadingHoldOmegaRadians(
        thetaPid, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), 1.0, maxOmega);

    // Assert — positive CCW correction, scaled by P gain and max-speed fraction
    assertTrue(omega > 0.0);
    assertTrue(omega <= maxOmega);
  }

  @Test
  void computeHeadingHoldOmegaRadians_wrapsSetpointAcross180Boundary() {
    // Arrange — current 170°, setpoint -170° (20° error, not 340°)
    double maxOmega = Math.toRadians(100);

    // Act
    double omega = SwerveDrive.computeHeadingHoldOmegaRadians(
        thetaPid, Rotation2d.fromDegrees(170), Rotation2d.fromDegrees(-170), 1.0, maxOmega);

    // Assert — small positive correction
    assertTrue(omega > 0.0);
    assertTrue(Math.abs(omega) < maxOmega);
  }

  @Test
  void computeHeadingHoldOmegaRadians_maxSpeedFraction_clampsOutput() {
    // Arrange
    double maxOmega = Math.toRadians(540);

    // Act
    double omega = SwerveDrive.computeHeadingHoldOmegaRadians(
        thetaPid, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), 0.25, maxOmega);

    // Assert
    assertEquals(0.25 * maxOmega, omega, 1e-6);
  }
}
