// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig;

/** motor used in a rack and pinion to deploy the hopper **/

public class Hopper extends SubsystemBase {
    private enum HopperPosition {
        Extended, Stowed, Moving
    }

    private final SparkMax motor;
    private final SparkClosedLoopController pid;
    private final RelativeEncoder relativeEncoder;
    private HopperPosition state = HopperPosition.Stowed;
    private HopperPosition setpointState = HopperPosition.Stowed;

    public Hopper() {
        
    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(IntakeConfig.HopperConfig.kHopperEncoderPositionConversionFactor)
        .velocityConversionFactor(IntakeConfig.HopperConfig.kHopperEncoderPositionConversionFactor);

        var pidConfig = new ClosedLoopConfig()
        .pid(IntakeConfig.HopperConfig.kDeployerP, IntakeConfig.HopperConfig.kDeployerI, IntakeConfig.HopperConfig.kDeployerD)
        .minOutput(-0.5)
        .maxOutput(0.5);

        var motorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .follow(RobotMap.HOPPER_EXTENDER_ID, true)
        .apply(pidConfig)
        .apply(relativeEncoderConfig);
    motor = new SparkMax(RobotMap.INTAKE_DEPLOYER_FOLLOWER_ID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pid = motor.getClosedLoopController();
    relativeEncoder = motor.getAlternateEncoder();

    SmartDashboard.putData(this);
    }

    public void retractHopper() {
    }

    public void deployHopper(HopperPosition setpointState) {
        pid.setSetpoint(getPositionOfHopperState(setpointState), ControlType.kPosition);
    }

    public double getPositionOfHopperState(HopperPosition state) {
        var position = 0.0;
        if (state == HopperPosition.Extended){
            position = IntakeConfig.HopperConfig.kExtendedExtension;
        }else if (state == HopperPosition.Stowed){
            position = IntakeConfig.HopperConfig.kStowedExtension;
        }
        return position;
    }

    public HopperPosition getState() {
        double currentPosition = relativeEncoder.getPosition();
        HopperPosition currentState = HopperPosition.Moving;
        //0.5 as error
        if (Math.abs(currentPosition - IntakeConfig.HopperConfig.kExtendedExtension) < 0.5){
            currentState = HopperPosition.Extended;
        }else if (Math.abs(currentPosition - IntakeConfig.HopperConfig.kStowedExtension) < 0.5){
            currentState = HopperPosition.Stowed;
        }
        return currentState;
    }

    public boolean isAtSetPoint() {
        var isAtPoint = (state == getState());
        return isAtPoint;
    }

    public Command retractHopperCommand() {
        var setHopperState = runOnce(() -> state = HopperPosition.Stowed);

        var cmd = setHopperState
            .andThen(this.run(() -> retractHopper()).until(this::isAtSetPoint))
            .andThen(Commands.waitSeconds(0.5));
        return cmd;
    }

    public Command deployHopperCommand() {
        var setHopperState = runOnce(() -> state = HopperPosition.Extended);

        var cmd = setHopperState
            .andThen(this.run(() -> deployHopper(state)).until(this::isAtSetPoint))
            .andThen(Commands.waitSeconds(0.5));
        return cmd;
    }

    public Supplier<HopperPosition> state() {
        return () -> this.state;
    }

    public BooleanSupplier isBlockingIntake() {
        return () -> state().get() == HopperPosition.Stowed;
    }

    /** {@inheritDoc}} */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        builder.addBooleanProperty("IntakeBlocked", isBlockingIntake(), null); 
    }

    public void stop() {
    motor.stopMotor();
  }
}
