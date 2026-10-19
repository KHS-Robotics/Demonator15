package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class AngleUtilsTest {
  @ParameterizedTest
  @CsvSource({
      "0, 0",
      "180, 180",
      "-180, -180",
      "360, 0",
      "-360, 0",
      "270, -90",
      "-270, 90",
      "450, 90",
      "-450, -90",
      "181, -179",
      "-181, 179"
  })
  void normalizeDegrees_wrapsToSignedRange(double inputDegrees, double expectedDegrees) {
    // Arrange — inputDegrees and expectedDegrees from @CsvSource

    // Act
    double actualDegrees = AngleUtils.normalizeDegrees(inputDegrees);

    // Assert
    assertEquals(expectedDegrees, actualDegrees, 1e-9);
  }
}
