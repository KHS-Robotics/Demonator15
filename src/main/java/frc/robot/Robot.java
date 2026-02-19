// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // silence disconnected joystick warnings when not FMS attached
    DriverStation.silenceJoystickConnectionWarning(true);

    // for debugging when not FMS attached
    CommandScheduler.getInstance().onCommandInitialize((cmd) -> {
      if (!DriverStation.isFMSAttached()) {
        System.out.println(cmd.getName() + " started.");
      }
    });
    CommandScheduler.getInstance().onCommandInterrupt((cmd) -> {
      if (!DriverStation.isFMSAttached()) {
        System.out.println(cmd.getName() + " interrupted.");
      }
    });
    CommandScheduler.getInstance().onCommandFinish((cmd) -> {
      if (!DriverStation.isFMSAttached()) {
        System.out.println(cmd.getName() + " ended.");
      }
    });

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html
    CommandScheduler.getInstance().run();

    // Update the robot's position and heading on the Dashboard GUI
    RobotContainer.kField.setRobotPose(RobotContainer.kSwerveDrive.getPose());

    // Joystick connection info
    SmartDashboard.putBoolean("Joysticks/Xbox Controller", RobotContainer.kDriverController.isConnected());
    SmartDashboard.putBoolean("Joysticks/Operator Stick", RobotContainer.kOperatorStick.isConnected());
    SmartDashboard.putBoolean("NavX-Connected", RobotContainer.kNavx.isConnected());
    SmartDashboard.putBoolean("IsHubActive", HubUtility.isHubActive().getAsBoolean());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
