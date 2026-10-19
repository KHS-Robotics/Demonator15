package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.FieldConstants;
import frc.robot.hid.DemonCommandXboxController;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveDriveTestFixture;
import frc.robot.testsupport.WpilibSimulationExtension;

/** End-to-end {@link DriveCommands} against a sim {@link SwerveDrive} (one drivetrain per class). */
@ExtendWith(WpilibSimulationExtension.class)
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveCommandsSwerveIntegrationTest {
  private static final int kControllerPort = 3;
  private static final double DELTA = 1e-3;

  private SwerveDriveTestFixture fixture;
  private SwerveDrive swerve;
  private DemonCommandXboxController controller;
  private XboxControllerSim controllerSim;

  @BeforeEach
  void setUp() {
    fixture = SwerveDriveTestFixture.open();
    swerve = fixture.swerve();
    // New instances each test: WpilibSimulationExtension resets DriverStation data before each method.
    controller = new DemonCommandXboxController(kControllerPort);
    controllerSim = new XboxControllerSim(kControllerPort);
    fixture.resetBetweenTests();
    resetControllerSticks();
    setAllianceBlue();
  }

  @AfterEach
  void tearDown() {
    fixture.resetBetweenTests();
    resetControllerSticks();
    setAllianceBlue();
  }

  private void resetControllerSticks() {
    controllerSim.setLeftY(0.0);
    controllerSim.setLeftX(0.0);
    controllerSim.setRightX(0.0);
    controllerSim.notifyNewData();
  }

  private void setAllianceBlue() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @Test
  void driveWithXboxController_centeredThenFullStick_togglesMotorOutput() {
    // Arrange
    Command teleop = DriveCommands.driveWithXboxController(
        swerve, controller, () -> false, 0.05, 0.0);
    CommandScheduler.getInstance().schedule(teleop);

    // Act — centered sticks
    resetControllerSticks();
    SimHooks.stepTiming(0.02);
    for (int i = 0; i < 3; i++) {
      CommandScheduler.getInstance().run();
      fixture.runSimulationSteps(1);
    }

    // Act — full forward
    controllerSim.setLeftY(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);
    for (int i = 0; i < 8; i++) {
      CommandScheduler.getInstance().run();
      fixture.runSimulationSteps(1);
    }

    // Assert
    assertTrue(fixture.maxAbsDriveMotorOutput() > 0.1);
  }

  @Test
  void resetHeading_setsPracticeFieldPose() {
    // Arrange
    Command reset = DriveCommands.resetHeading(
        swerve, () -> FieldConstants.kDriverPracticeResetX, () -> FieldConstants.kDriverPracticeResetY);

    // Act
    CommandScheduler.getInstance().schedule(reset);
    CommandScheduler.getInstance().run();

    // Assert
    assertEquals(FieldConstants.kDriverPracticeResetX, swerve.getPose().getX(), DELTA);
    assertEquals(FieldConstants.kDriverPracticeResetY, swerve.getPose().getY(), DELTA);
    assertEquals(0.0, swerve.getPose().getRotation().getDegrees(), DELTA);
  }

  @Test
  void stopCommand_invokesSubsystemStop() {
    // Arrange
    swerve.drive(1.0, 0.0, 0.0, false);
    fixture.runSimulationSteps(3);
    assertTrue(fixture.maxAbsDriveMotorOutput() > 0.01);
    Command stop = DriveCommands.stop(swerve);

    // Act
    CommandScheduler.getInstance().schedule(stop);
    CommandScheduler.getInstance().run();

    // Assert
    assertEquals(0.0, fixture.maxAbsDriveMotorOutput(), DELTA);
  }

  @Test
  void lockCommand_whileScheduled_runsUntilCancelled() {
    // Arrange
    Command lock = DriveCommands.lock(swerve);

    // Act
    CommandScheduler.getInstance().schedule(lock);
    CommandScheduler.getInstance().run();
    boolean scheduled = CommandScheduler.getInstance().isScheduled(lock);
    CommandScheduler.getInstance().cancel(lock);
    CommandScheduler.getInstance().run();

    // Assert
    assertTrue(scheduled);
    assertEquals(0.0, fixture.maxAbsDriveMotorOutput(), DELTA);
  }

  @Test
  void goSlow_whileScheduled_limitsTeleopMaxSpeed() {
    // Arrange
    Command goSlow = DriveCommands.goSlow(swerve);
    Command teleop = DriveCommands.driveWithXboxController(
        swerve, controller, () -> false, 0.05, 0.0);
    controllerSim.setLeftY(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    // Act
    CommandScheduler.getInstance().schedule(goSlow);
    CommandScheduler.getInstance().schedule(teleop);
    CommandScheduler.getInstance().run();

    // Assert
    assertEquals(0.5, SwerveDrive.maxTranslationalSpeedMetersPerSecond, DELTA);
    assertFalse(Double.isNaN(controller.getDriveTranslation(0.05, 0.0, SwerveDrive.maxTranslationalSpeedMetersPerSecond)
        .xMetersPerSecond()));
  }

  @Test
  void holdCurrentHeadingWhileDriving_schedulesAndRunsWithoutInterrupting() {
    // Arrange
    swerve.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
    Rotation2d headingBefore = swerve.getPose().getRotation();
    Command defense = DriveCommands.holdCurrentHeadingWhileDriving(
        swerve, controller, () -> false, 0.05, 0.0);
    controllerSim.setLeftY(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    // Act
    CommandScheduler.getInstance().schedule(defense);
    for (int i = 0; i < 8; i++) {
      CommandScheduler.getInstance().run();
      fixture.runSimulationSteps(1);
    }

    // Assert
    assertTrue(CommandScheduler.getInstance().isScheduled(defense) || defense.isFinished());
    assertEquals(headingBefore.getDegrees(), swerve.getPose().getRotation().getDegrees(), DELTA);
  }

  @Test
  void fieldRelativeAllianceSign_readsDriverStationSimAlliance() {
    // Arrange & Act — red
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // Assert
    assertEquals(Alliance.Red, DriverStation.getAlliance().orElse(Alliance.Blue));
    assertEquals(-1, DemonCommandXboxController.fieldRelativeAllianceSign(() -> true));

    // Arrange & Act — blue
    setAllianceBlue();

    // Assert
    assertEquals(Alliance.Blue, DriverStation.getAlliance().orElse(Alliance.Blue));
    assertEquals(1, DemonCommandXboxController.fieldRelativeAllianceSign(() -> true));
  }
}
