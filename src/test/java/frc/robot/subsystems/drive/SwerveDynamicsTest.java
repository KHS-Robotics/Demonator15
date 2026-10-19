package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import org.junit.jupiter.api.Test;

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
}
