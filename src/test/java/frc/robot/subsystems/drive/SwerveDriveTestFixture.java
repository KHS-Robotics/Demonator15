package frc.robot.subsystems.drive;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.constants.FieldConstants;

/**
 * Constructs a sim-backed {@link SwerveDrive} without {@link frc.robot.RobotContainer}.
 *
 * <p>Call {@link #close()} (or use try-with-resources) to release module CAN handles.
 */
public final class SwerveDriveTestFixture implements AutoCloseable {
  private static SwerveDriveTestFixture active;

  private final AtomicReference<Rotation2d> heading = new AtomicReference<>(Rotation2d.kZero);
  private final SwerveDrive swerve;

  /** Opens the shared sim drivetrain for this JVM (one instance across swerve test classes). */
  public static SwerveDriveTestFixture open() {
    if (active == null) {
      active = new SwerveDriveTestFixture();
    }
    return active;
  }

  /**
   * Closes the shared instance if it was opened.
   *
   * <p>Do not call from per-class {@code @AfterAll} when multiple test classes share this fixture —
   * class execution order is undefined and closing early breaks other classes. Prefer leaving the
   * sim drivetrain open for the JVM; use this only from try-with-resources in isolated tests.
   */
  public static void closeActive() {
    if (active != null) {
      active.close();
      active = null;
    }
  }

  private SwerveDriveTestFixture() {
    swerve = new SwerveDrive(headingSupplier(), false);
    swerve.seedSimulationPivotEncodersZero();
    SimHooks.stepTiming(0.02);
  }

  public SwerveDrive swerve() {
    return swerve;
  }

  public Supplier<Rotation2d> headingSupplier() {
    return heading::get;
  }

  public void setHeading(Rotation2d rotation) {
    heading.set(rotation);
  }

  public void reseedPivotEncoders() {
    swerve.seedSimulationPivotEncodersZero();
  }

  /**
   * Clears scheduler state, restores teleop speed limits, and resets drivetrain pose/heading for the next test.
   * Call from {@code @BeforeEach} / {@code @AfterEach} so tests do not depend on run order.
   */
  public void resetBetweenTests() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();
    SwerveDrive.maxTranslationalSpeedMetersPerSecond = 4.6;
    SwerveDrive.maxAngularSpeedRadiansPerSecond = Math.toRadians(540);
    heading.set(Rotation2d.kZero);
    swerve.stop();
    swerve.seedSimulationPivotEncodersZero();
    swerve.resetPose(FieldConstants.kDefaultOdometryPose);
    SimHooks.stepTiming(0.02);
  }

  public void runSimulationSteps(int iterations) {
    swerve.runSimulationSteps(iterations);
  }

  public double maxAbsDriveMotorOutput() {
    return swerve.maxAbsDriveMotorOutput();
  }

  public double maxAbsPivotMotorOutput() {
    return swerve.maxAbsPivotMotorOutput();
  }

  @Override
  public void close() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterSubsystem(swerve);
    swerve.closeModulesForTest();
  }
}
