package frc.robot.testsupport;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.extension.BeforeEachCallback;
import org.junit.jupiter.api.extension.ExtensionContext;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * JUnit 5 extension that initializes WPILib HAL and a enabled, attached driver station before each test.
 *
 * <p>Apply with {@code @ExtendWith(WpilibSimulationExtension.class)} on tests that construct
 * {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}, REV Spark, or CTRE devices.
 *
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html">WPILib unit testing</a>
 */
public final class WpilibSimulationExtension implements BeforeEachCallback {
  private static boolean halInitialized;

  /** {@inheritDoc} */
  @Override
  public void beforeEach(ExtensionContext context) {
    if (!halInitialized) {
      assertTrue(HAL.initialize(500, 0), "HAL.initialize failed");
      halInitialized = true;
    }
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.0);
    SimHooks.stepTiming(0.02);
  }
}
