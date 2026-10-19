package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.testsupport.WpilibSimulationExtension;

/**
 * Constructs a real {@link SwerveDrive} (four sim-backed modules) without loading {@link frc.robot.RobotContainer}.
 */
@ExtendWith(WpilibSimulationExtension.class)
class SwerveDriveTest {
  private static final double DELTA = 1e-3;

  private SwerveDrive swerve;

  @BeforeEach
  void setUpSwerve() {
    // Arrange — injected heading avoids RobotContainer ↔ SwerveDrive static init cycle
    swerve = new SwerveDrive(() -> Rotation2d.kZero, false);
    swerve.seedSimulationPivotEncodersZero();
    SimHooks.stepTiming(0.02);
  }

  @AfterEach
  void tearDownSwerve() {
    if (swerve != null) {
      swerve.closeModulesForTest();
      swerve = null;
    }
  }

  @Test
  void resetPose_updatesEstimatedPose() {
    // Arrange
    Pose2d expected = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(30));

    // Act
    swerve.resetPose(expected);

    // Assert
    assertEquals(expected.getX(), swerve.getPose().getX(), DELTA);
    assertEquals(expected.getY(), swerve.getPose().getY(), DELTA);
    assertEquals(expected.getRotation().getDegrees(), swerve.getPose().getRotation().getDegrees(), DELTA);
  }

  @Test
  void drive_belowDeadband_zerosModuleWheelSpeeds() {
    // Arrange
    double belowDeadband = 0.001;

    // Act
    swerve.drive(belowDeadband, belowDeadband, belowDeadband, false);
    swerve.runSimulationSteps(1);

    // Assert
    for (SwerveModuleState state : swerve.getSwerveModuleStates()) {
      assertEquals(0.0, state.speedMetersPerSecond, DELTA);
    }
  }

  @Test
  void drive_forwardRobotRelative_commandsDriveMotors() {
    // Arrange
    double forwardSpeed = 1.0;

    // Act
    swerve.drive(forwardSpeed, 0.0, 0.0, false);
    swerve.runSimulationSteps(5);

    // Assert — commanded output on sim motors (measured wheel speed may still be 0 without encoder motion)
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void holdAngleWhileDriving_headingError_commandsPivotMotors() {
    // Arrange
    swerve.resetPose(Pose2d.kZero);

    // Act
    swerve.holdAngleWhileDriving(0.0, 0.0, Rotation2d.fromDegrees(90), 1.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsPivotMotorOutput() > 0.01);
  }

}
