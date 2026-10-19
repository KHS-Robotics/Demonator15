package frc.robot.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class HIDUtilsTest {
  @Test
  void smoothInputWithCubic_linearWhenSensitivityZero() {
    // Arrange
    double sensitivity = 0.0;

    // Act
    double positiveOutput = HIDUtils.smoothInputWithCubic(0.5, sensitivity);
    double negativeOutput = HIDUtils.smoothInputWithCubic(-0.25, sensitivity);

    // Assert
    assertEquals(0.5, positiveOutput, 1e-9);
    assertEquals(-0.25, negativeOutput, 1e-9);
  }

  @Test
  void smoothInputWithCubic_cubicWhenSensitivityOne() {
    // Arrange
    double input = 0.5;
    double sensitivity = 1.0;

    // Act
    double output = HIDUtils.smoothInputWithCubic(input, sensitivity);

    // Assert
    assertEquals(0.125, output, 1e-9);
  }

  @Test
  void smoothInputWithCubic_zeroInput_isZero() {
    // Arrange
    double input = 0.0;
    double sensitivity = 0.75;

    // Act
    double output = HIDUtils.smoothInputWithCubic(input, sensitivity);

    // Assert
    assertEquals(0.0, output, 1e-9);
  }
}
