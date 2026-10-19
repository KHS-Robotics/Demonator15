package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SwerveModuleOptimizationTest {
  @Test
  void shortestPivotTarget_smallError_doesNotFlipDrive() {
    // Arrange
    double currentAngleDegrees = 10;
    double targetAngleDegrees = 40;

    // Act
    SwerveModuleOptimization.PivotTarget result =
        SwerveModuleOptimization.shortestPivotTarget(currentAngleDegrees, targetAngleDegrees);

    // Assert
    assertFalse(result.driveInverted());
    assertEquals(40, result.angleDegrees(), 1e-9);
  }

  @Test
  void shortestPivotTarget_largeError_flipsDriveAndOffsetsTarget() {
    // Arrange
    double currentAngleDegrees = 10;
    double targetAngleDegrees = 170;

    // Act
    SwerveModuleOptimization.PivotTarget result =
        SwerveModuleOptimization.shortestPivotTarget(currentAngleDegrees, targetAngleDegrees);

    // Assert
    assertTrue(result.driveInverted());
    assertEquals(-10, result.angleDegrees(), 1e-9);
  }

  @Test
  void cosineScale_alignedModule_isOne() {
    // Arrange
    double currentAngleDegrees = 45;
    double targetAngleDegrees = 45;

    // Act
    double scale = SwerveModuleOptimization.cosineScale(currentAngleDegrees, targetAngleDegrees);

    // Assert
    assertEquals(1.0, scale, 1e-9);
  }

  @Test
  void cosineScale_perpendicularModule_isZero() {
    // Arrange
    double currentAngleDegrees = 0;
    double targetAngleDegrees = 90;

    // Act
    double scale = SwerveModuleOptimization.cosineScale(currentAngleDegrees, targetAngleDegrees);

    // Assert
    assertEquals(0.0, scale, 1e-9);
  }
}
