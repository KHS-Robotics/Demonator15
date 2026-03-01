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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/** motor used in a rack and pinion to deploy the hopper **/

public class Hopper extends SubsystemBase {
    private enum HopperState {
        Deployed(IntakeConfig.HopperSetPoints.DEPLOY),
        Stowed(IntakeConfig.HopperSetPoints.STOW);

        private final double hopperPosition;

        private HopperState(double hopperPosition) {
            this.hopperPosition = hopperPosition;
        }
    }

    private final SparkMax motor;
    private final SparkClosedLoopController pid;
    private final RelativeEncoder relativeEncoder;
    private HopperState setpointState = HopperState.Stowed;

    public Hopper() {

        var relativeEncoderConfig = new EncoderConfig()
                .positionConversionFactor(IntakeConfig.HopperConfig.kHopperEncoderPositionConversionFactor)
                .velocityConversionFactor(IntakeConfig.HopperConfig.kHopperEncoderPositionConversionFactor);

        var pidConfig = new ClosedLoopConfig()
                .pid(IntakeConfig.HopperConfig.kDeployerP, IntakeConfig.HopperConfig.kDeployerI,
                        IntakeConfig.HopperConfig.kDeployerD)
                .minOutput(-0.5)
                .maxOutput(0.5);

        var motorConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .follow(RobotMap.HOPPER_EXTENDER_ID, true)
                .apply(pidConfig)
                .apply(relativeEncoderConfig);
        motor = new SparkMax(RobotMap.HOPPER_EXTENDER_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pid = motor.getClosedLoopController();
        relativeEncoder = motor.getEncoder();

        SmartDashboard.putData(this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        return cmd.withName("StopHopper");
    }

    public void moveHopper(HopperState setpointState) {
        pid.setSetpoint(setpointState.hopperPosition, ControlType.kPosition);
    }

    public boolean isAtSetPoint() {
        var error = Math.abs(setpointState.hopperPosition - relativeEncoder.getPosition());
        return (error < 0.5);
    }

    public Command retractHopperCommand() {
        var setHopperState = runOnce(() -> setpointState = HopperState.Stowed);

        var cmd = setHopperState
                .andThen(this.run(() -> moveHopper(setpointState)).until(this::isAtSetPoint));
        return cmd;
    }

    public Command deployHopperCommand() {
        var setHopperState = runOnce(() -> setpointState = HopperState.Deployed);

        var cmd = setHopperState
                .andThen(this.run(() -> moveHopper(setpointState)).until(this::isAtSetPoint));
        return cmd;
    }

    public Supplier<HopperState> state() {
        return () -> this.setpointState;
    }

    public BooleanSupplier isBlockingIntake() {
        return () -> state().get() == HopperState.Stowed;
    }

    /** {@inheritDoc}} */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());
        builder.addBooleanProperty("IntakeBlocked", isBlockingIntake(), null);
    }

}
