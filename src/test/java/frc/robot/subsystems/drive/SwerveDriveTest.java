package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.FieldConstants;
import frc.robot.testsupport.WpilibSimulationExtension;

/** One sim {@link SwerveDrive} per class — avoids re-opening the same CAN devices every test method. */
@ExtendWith(WpilibSimulationExtension.class)
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SwerveDriveTest {
  private static final double DELTA = 1e-3;

  private SwerveDriveTestFixture fixture;

  @AfterAll
  void destroyFixture() {
    SwerveDriveTestFixture.closeActive();
    fixture = null;
  }

  @BeforeEach
  void resetSwerveBetweenTests() {
    fixture = SwerveDriveTestFixture.open();
    CommandScheduler.getInstance().cancelAll();
    SwerveDrive swerve = fixture.swerve();
    swerve.stop();
    fixture.setHeading(Rotation2d.kZero);
    swerve.resetPose(FieldConstants.kDefaultOdometryPose);
    swerve.seedSimulationPivotEncodersZero();
    SimHooks.stepTiming(0.02);
  }

  @Test
  void constructor_seedsPoseFromFieldConstants() {
    // Arrange & Act
    Pose2d pose = fixture.swerve().getPose();

    // Assert
    assertEquals(FieldConstants.kDefaultOdometryPose.getX(), pose.getX(), DELTA);
    assertEquals(FieldConstants.kDefaultOdometryPose.getY(), pose.getY(), DELTA);
  }

  @Test
  void resetPose_updatesEstimatedPose() {
    // Arrange
    Pose2d expected = new Pose2d(2.0, 1.5, Rotation2d.fromDegrees(30));
    SwerveDrive swerve = fixture.swerve();

    // Act
    swerve.resetPose(expected);

    // Assert
    assertEquals(expected.getX(), swerve.getPose().getX(), DELTA);
    assertEquals(expected.getY(), swerve.getPose().getY(), DELTA);
    assertEquals(expected.getRotation().getDegrees(), swerve.getPose().getRotation().getDegrees(), DELTA);
  }

  @Test
  void periodic_usesInjectedHeadingSupplier() {
    SwerveDrive swerve = fixture.swerve();

    // Arrange
    swerve.resetPose(Pose2d.kZero);
    fixture.setHeading(Rotation2d.fromDegrees(45));

    // Act
    SimHooks.stepTiming(0.02);
    swerve.periodic();

    // Assert
    assertEquals(45.0, swerve.getPose().getRotation().getDegrees(), DELTA);
  }

  @Test
  void getChassisSpeeds_atRest_isNearZero() {
    // Arrange & Act
    ChassisSpeeds speeds = fixture.swerve().getChassisSpeeds();

    // Assert
    assertEquals(0.0, speeds.vxMetersPerSecond, DELTA);
    assertEquals(0.0, speeds.vyMetersPerSecond, DELTA);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, DELTA);
  }

  @Test
  void drive_belowDeadband_zerosModuleWheelSpeeds() {
    SwerveDrive swerve = fixture.swerve();
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
    SwerveDrive swerve = fixture.swerve();

    // Act
    swerve.drive(1.0, 0.0, 0.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void drive_strafeRobotRelative_commandsDriveMotors() {
    SwerveDrive swerve = fixture.swerve();

    // Act
    swerve.drive(0.0, 1.0, 0.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void drive_pureRotation_commandsMotors() {
    SwerveDrive swerve = fixture.swerve();

    // Act
    swerve.drive(0.0, 0.0, 1.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void drive_fieldRelative_commandsDriveMotorsWhenPoseRotated() {
    SwerveDrive swerve = fixture.swerve();

    // Arrange — robot facing 90°
    swerve.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90)));

    // Act — field-relative forward
    swerve.drive(2.0, 0.0, 0.0, true);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void holdAngleWhileDriving_headingError_commandsPivotMotors() {
    SwerveDrive swerve = fixture.swerve();
    swerve.resetPose(Pose2d.kZero);

    // Act
    swerve.holdAngleWhileDriving(0.0, 0.0, Rotation2d.fromDegrees(90), 1.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsPivotMotorOutput() > 0.01);
  }

  @Test
  void holdAngleWhileDriving_withTranslation_commandsDriveMotors() {
    SwerveDrive swerve = fixture.swerve();
    swerve.resetPose(Pose2d.kZero);

    // Act
    swerve.holdAngleWhileDriving(1.5, 0.0, Rotation2d.fromDegrees(0), 1.0, false);
    swerve.runSimulationSteps(5);

    // Assert
    assertTrue(swerve.maxAbsDriveMotorOutput() > 0.01);
  }

  @Test
  void lock_doesNotThrowAndStopsModulesOnStop() {
    SwerveDrive swerve = fixture.swerve();

    // Act
    swerve.lock();
    swerve.runSimulationSteps(2);
    swerve.stop();

    // Assert
    assertEquals(0.0, swerve.maxAbsDriveMotorOutput(), DELTA);
    assertEquals(0.0, swerve.maxAbsPivotMotorOutput(), DELTA);
  }

  @Test
  void addVisionMeasurement_fusesPoseTowardMeasurement() {
    SwerveDrive swerve = fixture.swerve();
    Matrix<N3, N1> tightVisionStdDevs = VecBuilder.fill(0.02, 0.02, Double.MAX_VALUE);

    // Arrange
    swerve.resetPose(Pose2d.kZero);
    for (int i = 0; i < 5; i++) {
      SimHooks.stepTiming(0.02);
      swerve.periodic();
    }
    double timestamp = Timer.getFPGATimestamp() - 0.01;
    Pose2d visionPose = new Pose2d(2.0, 0.5, Rotation2d.kZero);

    // Act
    swerve.addVisionMeasurementForOdometry(visionPose, timestamp, tightVisionStdDevs);
    SimHooks.stepTiming(0.02);
    swerve.periodic();

    // Assert
    assertTrue(swerve.getPose().getX() > 0.5);
    assertTrue(swerve.getPose().getY() > 0.1);
  }
}
