// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** hopper **/

public class Hopper extends SubsystemBase {
    private enum HopperPosition {
        Retract("Retract"),
        Deploy("Deploy"),
        Stow("Stow");

        private final String state;

    private HopperPosition(String s) {
      state = s;
    }

    public String toString() {
      return this.state;
    }
    }

    private HopperPosition hopperPosition = HopperPosition.Stow;
    private final Servo rightAcuator, leftAcuator;
    private double setPointHopperStowed = IntakeConfig.HopperSetPoints.HOPPERSTOW;
    private HopperPosition state = HopperPosition.Retract;

    public Hopper() {
        SmartDashboard.putData(this);

        rightAcuator = new Servo(RobotMap.DEPLOYER_RIGHT_SERVO);
        leftAcuator = new Servo(RobotMap.DEPLOYER_LEFT_SERVO);

    }

    public void retractHopper() {
        rightAcuator.set(IntakeConfig.HopperSetPoints.HOPPERRETRACT);
        leftAcuator.set(IntakeConfig.HopperSetPoints.HOPPERRETRACT);
    }

    public void deployHopper() {
        rightAcuator.set(IntakeConfig.HopperSetPoints.HOPPERDEPLOY);
        leftAcuator.set(IntakeConfig.HopperSetPoints.HOPPERDEPLOY);
    }

    public boolean hopperIsSetPoint() {
        var error1 = Math.abs(setPointHopperStowed - rightAcuator.getAngle());
        var error2 = Math.abs(setPointHopperStowed - leftAcuator.getAngle());
        return (error1 < 0.5 && error2 < 0.5);
    }

    public Command retractHopperCommand() {
        var setHopperState = runOnce(() -> state = HopperPosition.Retract);

        var cmd = setHopperState
            .andThen(this.run(() -> retractHopper()).until(this::hopperIsSetPoint))
            .andThen(Commands.waitSeconds(0.5));
        return cmd;
    }

    public Command deployHopperCommand() {
        var setHopperState = runOnce(() -> state = HopperPosition.Deploy);

        var cmd = setHopperState
            .andThen(this.run(() -> deployHopper()).until(this::hopperIsSetPoint))
            .andThen(Commands.waitSeconds(0.5));
        return cmd;
    }

    public Supplier<HopperPosition> state() {
        return () -> this.state;
    }

    public BooleanSupplier isBlockingIntake() {
        return () -> state().get() == HopperPosition.Retract;
    }

    /** {@inheritDoc}} */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        builder.addStringProperty("Hopper State", () -> hopperPosition.toString(), null);
        builder.addBooleanProperty("Intake Blocked?", isBlockingIntake(), null); 
    }

    public void stop() {
    rightAcuator.setDisabled();
    leftAcuator.setDisabled();
  }
}
