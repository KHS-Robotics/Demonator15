// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Thrustmaster T16000M
 */
public class OperatorStick extends Joystick {
  public BooleanSupplier IntakeAtTop;
  public OperatorStick(int port) {
    super(port);
  }

  public Trigger runIntake(){
    return new Trigger(() -> this.getRawButton(ButtonMap.INTAKE_BUTTON) && !IntakeAtTop.getAsBoolean());
  }

  public Trigger outtake(){
    return new Trigger(() -> this.getRawButton(ButtonMap.OUTTAKE_BUTTON) && !IntakeAtTop.getAsBoolean());
  }

  public Trigger stowIntake(){
    return new Trigger(() -> this.getRawButton(ButtonMap.STOW_INTAKE_BUTTON));
  }

  public Trigger agitateIntake(){
    return new Trigger(() -> this.getRawButton(ButtonMap.AGITATE_INTAKE_BUTTON));
  }

  public Trigger deployIntake(){
    return new Trigger(() -> this.getRawButton(ButtonMap.DEPLOY_INTAKE_BUTTON));
  }

  public Trigger autoClimb(){
    return new Trigger(() -> this.getRawButton(ButtonMap.CLIMB_BUTTON));
  }

  public Trigger stopClimber(){
    return new Trigger(() -> this.getRawButton(ButtonMap.STOP_CLIMBER_BUTTON));
  }

  public Trigger forwardIndex(){
    return new Trigger(() -> this.getRawButton(ButtonMap.FORWARD_INDEX_BUTTON));
  }

  public Trigger reverseIndex(){
    return new Trigger(() -> this.getRawButton(ButtonMap.REVERSE_INDEX_BUTTON));
  }

  public Trigger autoShoot(){
    return new Trigger(() -> this.getRawButton(ButtonMap.AUTO_FIRE_BUTTON));
  }

  public Trigger switchAimingMode(){
    return new Trigger(() -> this.getRawButton(ButtonMap.SWITCH_AIMING_MODE));
  }
}
