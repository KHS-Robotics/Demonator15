package frc.robot.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

import frc.robot.testsupport.WpilibSimulationExtension;

@ExtendWith(WpilibSimulationExtension.class)
class DemonCommandXboxControllerSimulationTest {
  private static final double DELTA = 1e-2;
  private static final int kControllerPort = 0;

  private DemonCommandXboxController controller;
  private XboxControllerSim controllerSim;

  @BeforeEach
  void setUpController() {
    controller = new DemonCommandXboxController(kControllerPort);
    controllerSim = new XboxControllerSim(kControllerPort);
  }

  @Test
  void getDriveTranslation_usesSimulatedJoystickAxes() {
    controllerSim.setLeftY(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    DemonCommandXboxController.DriveTranslation speeds = controller.getDriveTranslation(0.05, 0.0, 4.0);

    assertEquals(4.0, speeds.xMetersPerSecond(), DELTA);
    assertEquals(0.0, speeds.yMetersPerSecond(), DELTA);
  }

  @Test
  void getDriveRotation_usesSimulatedRightStick() {
    controllerSim.setRightX(-1.0);
    controllerSim.notifyNewData();
    SimHooks.stepTiming(0.02);

    double omega = controller.getDriveRotation(0.05, 0.0, Math.toRadians(360));

    assertEquals(Math.toRadians(360), omega, DELTA);
  }
}
