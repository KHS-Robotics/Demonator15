package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;

class FieldConstantsTest {
  @Test
  void defaultOdometryPose_isExpectedStartPose() {
    // Arrange & Act
    var pose = FieldConstants.kDefaultOdometryPose;

    // Assert
    assertEquals(8.5, pose.getX(), 1e-9);
    assertEquals(4.0, pose.getY(), 1e-9);
    assertEquals(0.0, pose.getRotation().getDegrees(), 1e-9);
    assertEquals(Rotation2d.kZero, pose.getRotation());
  }

  @Test
  void driverPracticeReset_hasExpectedFieldCoordinates() {
    // Arrange & Act — season constants (no mutation)
    double resetX = FieldConstants.kDriverPracticeResetX;
    double resetY = FieldConstants.kDriverPracticeResetY;

    // Assert
    assertEquals(1.016, resetX, 1e-9);
    assertEquals(4.0, resetY, 1e-9);
  }
}
