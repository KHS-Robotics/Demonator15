package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.hid.HIDUtils;

class SwerveTeleopInputsTest {
  @Test
  void translationFromAxes_belowDeadband_isZero() {
    // Arrange
    double leftY = 0.01;
    double leftX = 0.01;
    double deadband = 0.05;
    double sensitivity = 0.0;
    double maxSpeedMetersPerSecond = 4.0;

    // Act
    SwerveTeleopInputs.TranslationalInput speeds = SwerveTeleopInputs.translationFromAxes(
        leftY, leftX, deadband, sensitivity, maxSpeedMetersPerSecond);

    // Assert
    assertEquals(0.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void translationFromAxes_fullForward_usesInvertedLeftY() {
    // Arrange
    double leftY = -1.0;
    double leftX = 0.0;
    double deadband = 0.05;
    double sensitivity = 0.0;
    double maxSpeedMetersPerSecond = 4.0;

    // Act
    SwerveTeleopInputs.TranslationalInput speeds = SwerveTeleopInputs.translationFromAxes(
        leftY, leftX, deadband, sensitivity, maxSpeedMetersPerSecond);

    // Assert
    assertEquals(4.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_redAllianceAndFieldRelative_flipsSign() {
    // Arrange — field-relative on red alliance

    // Act
    int signWhenFieldRelative = SwerveTeleopInputs.fieldRelativeAllianceSign(true, Alliance.Red);
    int signWhenRobotRelative = SwerveTeleopInputs.fieldRelativeAllianceSign(false, Alliance.Red);

    // Assert
    assertEquals(-1, signWhenFieldRelative);
    assertEquals(1, signWhenRobotRelative);
  }

  @Test
  void translationFromAxes_fullStick_appliesCubicSmoothingToBothAxes() {
    // Arrange
    double maxTranslation = 4.6;
    double deadband = 0.05;
    double sensitivity = 0.5;
    double expectedX = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;
    double expectedY = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;

    // Act
    SwerveTeleopInputs.TranslationalInput speeds = SwerveTeleopInputs.translationFromAxes(
        -1.0, -1.0, deadband, sensitivity, maxTranslation);

    // Assert
    assertEquals(expectedX, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(expectedY, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void rotationFromAxis_fullStick_appliesCubicSmoothing() {
    // Arrange
    double maxAngular = Math.toRadians(540);
    double deadband = 0.05;
    double sensitivity = 0.5;
    double expectedOmega = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxAngular;

    // Act
    double omega = SwerveTeleopInputs.rotationFromAxis(-1.0, deadband, sensitivity, maxAngular);

    // Assert
    assertEquals(expectedOmega, omega, 1e-9);
  }

  @Test
  void rotationFromAxis_belowDeadband_isZero() {
    // Act
    double omega = SwerveTeleopInputs.rotationFromAxis(0.02, 0.05, 0.0, Math.toRadians(540));

    // Assert
    assertEquals(0.0, omega, 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_blueAlliance_doesNotFlip() {
    // Arrange — field-relative on blue alliance

    // Act
    int sign = SwerveTeleopInputs.fieldRelativeAllianceSign(true, Alliance.Blue);

    // Assert
    assertEquals(1, sign);
  }
}
