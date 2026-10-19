package frc.robot.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DemonCommandXboxControllerTest {
  @Test
  void driveTranslationFromAxes_belowDeadband_isZero() {
    // Arrange
    double leftY = 0.01;
    double leftX = 0.01;
    double deadband = 0.05;
    double maxTranslation = 4.0;

    // Act
    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        leftY, leftX, deadband, 0.0, maxTranslation);

    // Assert
    assertEquals(0.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void driveTranslationFromAxes_fullForward_usesInvertedLeftY() {
    // Arrange
    double leftY = -1.0;
    double leftX = 0.0;
    double deadband = 0.05;
    double maxTranslation = 4.0;

    // Act
    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        leftY, leftX, deadband, 0.0, maxTranslation);

    // Assert
    assertEquals(4.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_redAllianceAndFieldRelative_flipsSign() {
    // Arrange & Act
    int fieldRelativeRed = DemonCommandXboxController.fieldRelativeAllianceSign(true, Alliance.Red);
    int notFieldRelativeRed = DemonCommandXboxController.fieldRelativeAllianceSign(false, Alliance.Red);

    // Assert
    assertEquals(-1, fieldRelativeRed);
    assertEquals(1, notFieldRelativeRed);
  }

  @Test
  void driveTranslationFromAxes_fullStick_appliesCubicSmoothingToBothAxes() {
    // Arrange
    double maxTranslation = 4.6;
    double sensitivity = 0.5;
    double expectedX = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;
    double expectedY = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;

    // Act
    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        -1.0, -1.0, 0.05, sensitivity, maxTranslation);

    // Assert
    assertEquals(expectedX, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(expectedY, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void rotationFromAxis_fullStick_appliesCubicSmoothing() {
    // Arrange
    double maxAngular = Math.toRadians(540);
    double sensitivity = 0.5;
    double expectedOmega = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxAngular;

    // Act
    double omega = DemonCommandXboxController.rotationFromAxis(-1.0, 0.05, sensitivity, maxAngular);

    // Assert
    assertEquals(expectedOmega, omega, 1e-9);
  }

  @Test
  void rotationFromAxis_belowDeadband_isZero() {
    // Arrange
    double rawAxis = 0.02;
    double deadband = 0.05;
    double maxAngular = Math.toRadians(540);

    // Act
    double omega = DemonCommandXboxController.rotationFromAxis(rawAxis, deadband, 0.0, maxAngular);

    // Assert
    assertEquals(0.0, omega, 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_blueAlliance_doesNotFlip() {
    // Arrange & Act
    int sign = DemonCommandXboxController.fieldRelativeAllianceSign(true, Alliance.Blue);

    // Assert
    assertEquals(1, sign);
  }
}
