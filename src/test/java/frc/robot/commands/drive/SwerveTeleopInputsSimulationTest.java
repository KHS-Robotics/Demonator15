package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.testsupport.WpilibSimulationExtension;

/**
 * Verifies teleop input path through WPILib {@link edu.wpi.first.wpilibj.simulation.XboxControllerSim}
 * and a live {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}.
 */
@ExtendWith(WpilibSimulationExtension.class)
class SwerveTeleopInputsSimulationTest {
  private static final double DELTA = 1e-2;
  private static final int kControllerPort = 0;

  private CommandXboxController controller;
  private XboxControllerSim controllerSim;

  @BeforeEach
  void setUpController() {
    // Arrange — shared per test (HAL initialized by WpilibSimulationExtension)
    SwerveDrive.maxTranslationalSpeedMetersPerSecond = 4.0;
    controller = new CommandXboxController(kControllerPort);
    controllerSim = new XboxControllerSim(kControllerPort);
  }

  @Test
  void readTranslation_usesSimulatedJoystickAxes() {
    // Arrange
    controllerSim.setLeftY(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    // Act
    SwerveTeleopInputs.TranslationalInput speeds =
        SwerveTeleopInputs.readTranslation(controller, 0.05, 0.0);

    // Assert
    assertEquals(4.0, speeds.xMetersPerSecond(), DELTA);
    assertEquals(0.0, speeds.yMetersPerSecond(), DELTA);
  }

  @Test
  void readRotation_usesSimulatedRightStick() {
    // Arrange
    SwerveDrive.maxAngularSpeedRadiansPerSecond = Math.toRadians(360);
    controllerSim.setRightX(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    // Act
    double omega = SwerveTeleopInputs.readRotation(controller, 0.05, 0.0);

    // Assert
    assertEquals(Math.toRadians(360), omega, DELTA);
  }
}
