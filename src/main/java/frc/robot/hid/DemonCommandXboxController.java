// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}.
 * 
 * @see edu.wpi.first.wpilibj2.command.button.CommandXboxController
 */
public class DemonCommandXboxController extends CommandXboxController {

  /** Ranges from [0, 1] where 0 is full linear and 1 is full cubic. */
  public static final double kJoystickSensitivity = 0.5;

  /** Deadband since joysticks vary in how well they snap back to zero. */
  public static final double kJoystickDeadband = 0.035;

  public DemonCommandXboxController(int port) {
    super(port);
  }

  public Trigger resetRobotHeading() {
    return this.start().debounce(0.5);
  }

  public Trigger goSlow() {
    return this.leftBumper();
  }

  public Trigger lockHeadingForDefense() {
    return this.rightBumper();
  }

  public Trigger robotRelative() {
    return this.a();
  }

}