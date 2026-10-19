package frc.robot.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

class DemonCommandXboxControllerTest {
  @Test
  void driveTranslationFromAxes_belowDeadband_isZero() {
    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        0.01, 0.01, 0.05, 0.0, 4.0);

    assertEquals(0.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void driveTranslationFromAxes_fullForward_usesInvertedLeftY() {
    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        -1.0, 0.0, 0.05, 0.0, 4.0);

    assertEquals(4.0, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(0.0, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_redAllianceAndFieldRelative_flipsSign() {
    assertEquals(-1, DemonCommandXboxController.fieldRelativeAllianceSign(true, Alliance.Red));
    assertEquals(1, DemonCommandXboxController.fieldRelativeAllianceSign(false, Alliance.Red));
  }

  @Test
  void driveTranslationFromAxes_fullStick_appliesCubicSmoothingToBothAxes() {
    double maxTranslation = 4.6;
    double sensitivity = 0.5;
    double expectedX = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;
    double expectedY = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxTranslation;

    DemonCommandXboxController.DriveTranslation speeds = DemonCommandXboxController.driveTranslationFromAxes(
        -1.0, -1.0, 0.05, sensitivity, maxTranslation);

    assertEquals(expectedX, speeds.xMetersPerSecond(), 1e-9);
    assertEquals(expectedY, speeds.yMetersPerSecond(), 1e-9);
  }

  @Test
  void rotationFromAxis_fullStick_appliesCubicSmoothing() {
    double maxAngular = Math.toRadians(540);
    double sensitivity = 0.5;
    double expectedOmega = HIDUtils.smoothInputWithCubic(1.0, sensitivity) * maxAngular;

    double omega = DemonCommandXboxController.rotationFromAxis(-1.0, 0.05, sensitivity, maxAngular);

    assertEquals(expectedOmega, omega, 1e-9);
  }

  @Test
  void rotationFromAxis_belowDeadband_isZero() {
    double omega = DemonCommandXboxController.rotationFromAxis(0.02, 0.05, 0.0, Math.toRadians(540));

    assertEquals(0.0, omega, 1e-9);
  }

  @Test
  void fieldRelativeAllianceSign_blueAlliance_doesNotFlip() {
    assertEquals(1, DemonCommandXboxController.fieldRelativeAllianceSign(true, Alliance.Blue));
  }
}
