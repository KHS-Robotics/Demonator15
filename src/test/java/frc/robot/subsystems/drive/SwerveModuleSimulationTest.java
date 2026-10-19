package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.testsupport.WpilibSimulationExtension;

/**
 * Integration-style tests that exercise REV {@link com.revrobotics.sim.SparkMaxSim} on a real
 * {@link SwerveModule} instance (isolated CAN IDs). Requires
 * {@link frc.robot.testsupport.WpilibSimulationExtension}.
 */
@ExtendWith(WpilibSimulationExtension.class)
class SwerveModuleSimulationTest {
  private static final int kTestDriveCanId = 60;
  private static final int kTestPivotCanId = 61;
  private static final int kTestPivotEncoderCanId = 62;

  private SwerveModule module;

  @BeforeEach
  void setUpModule() {
    // Arrange — module with sim-backed hardware (HAL from WpilibSimulationExtension)
    module = createTestModule(0.0);
    module.pivotEncoderSimState().setSupplyVoltage(12.0);
    module.pivotEncoderSimState().setRawPosition(Units.Rotations.of(0.0));
    refreshPivotEncoder();
  }

  @AfterEach
  void tearDownModule() {
    if (module != null) {
      module.close();
      module = null;
    }
  }

  @Test
  void setDesiredState_commandsPivotMotorWhenAngleErrorExists() {
    // Arrange
    module.pivotEncoderSimState().setRawPosition(Units.Rotations.of(0.0));
    refreshPivotEncoder();

    // Act
    module.setDesiredState(0.0, 90.0);
    for (int i = 0; i < 5; i++) {
      module.simulationStep();
      SimHooks.stepTiming(0.02);
    }

    // Assert
    assertTrue(Math.abs(module.pivotSparkSim().getAppliedOutput()) > 0.01);
  }

  @Test
  void setDesiredState_smallPivotError_doesNotInvertDrive() {
    // Arrange
    module.pivotEncoderSimState().setRawPosition(Units.Rotations.of(0.0));
    refreshPivotEncoder();

    // Act
    module.setDesiredState(2.0, 20.0);

    // Assert
    assertFalse(module.isOptimizingAngle());
  }

  @Test
  void setDesiredState_largePivotError_invertsDrive() {
    // Arrange
    module.pivotEncoderSimState().setRawPosition(Units.Rotations.of(0.0));
    refreshPivotEncoder();

    // Act
    module.setDesiredState(2.0, 170.0);

    // Assert
    assertTrue(module.isOptimizingAngle());
  }

  private static SwerveModule createTestModule(double pivotOffsetDegrees) {
    return new SwerveModule(
        "Test",
        kTestDriveCanId,
        kTestPivotCanId,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
        SwerveDriveConfig.DRIVE_MODULE_P,
        SwerveDriveConfig.DRIVE_MODULE_I,
        SwerveDriveConfig.DRIVE_MODULE_D,
        SwerveDriveConfig.DRIVE_MODULE_KS,
        SwerveDriveConfig.DRIVE_MODULE_KV,
        SwerveDriveConfig.DRIVE_MODULE_KA,
        kTestPivotEncoderCanId,
        pivotOffsetDegrees);
  }

  private void refreshPivotEncoder() {
    SimHooks.stepTiming(0.02);
    module.refreshPivotAbsolutePosition();
  }
}
