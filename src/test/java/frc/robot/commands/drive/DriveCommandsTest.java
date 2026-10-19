package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.testsupport.WpilibSimulationExtension;

@ExtendWith(WpilibSimulationExtension.class)
class DriveCommandsTest {
  private double savedTranslationalMax;
  private double savedAngularMax;

  @BeforeEach
  void setUp() {
    savedTranslationalMax = SwerveDrive.maxTranslationalSpeedMetersPerSecond;
    savedAngularMax = SwerveDrive.maxAngularSpeedRadiansPerSecond;
    CommandScheduler.getInstance().cancelAll();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    SwerveDrive.maxTranslationalSpeedMetersPerSecond = savedTranslationalMax;
    SwerveDrive.maxAngularSpeedRadiansPerSecond = savedAngularMax;
  }

  @Test
  void goSlow_whileScheduled_lowersMaxSpeeds_restoresOnEnd() {
    // Arrange
    Command goSlow = DriveCommands.goSlow(null);

    // Act — start
    CommandScheduler.getInstance().schedule(goSlow);
    CommandScheduler.getInstance().run();

    // Assert — slow limits
    assertEquals(0.5, SwerveDrive.maxTranslationalSpeedMetersPerSecond, 1e-9);
    assertEquals(Math.toRadians(90), SwerveDrive.maxAngularSpeedRadiansPerSecond, 1e-9);

    // Act — end
    CommandScheduler.getInstance().cancel(goSlow);
    CommandScheduler.getInstance().run();

    // Assert — normal limits restored
    assertEquals(4.6, SwerveDrive.maxTranslationalSpeedMetersPerSecond, 1e-9);
    assertEquals(Math.toRadians(540), SwerveDrive.maxAngularSpeedRadiansPerSecond, 1e-9);
  }

  @Test
  void goSlow_hasExpectedCommandName() {
    // Arrange & Act
    Command goSlow = DriveCommands.goSlow(null);

    // Assert
    assertEquals("SwerveGoSlow", goSlow.getName());
  }
}
