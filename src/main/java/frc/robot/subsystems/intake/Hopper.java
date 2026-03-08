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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
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

    private final SparkFlex motor;
    private final RelativeEncoder relativeEncoder;
    private HopperState setpointState = HopperState.Stowed;

    public Hopper() {

        var motorConfig = new SparkFlexConfig()
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
        motor = new SparkFlex(RobotMap.HOPPER_EXTENDER_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

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

    public Command primitiveSetVoltage() {
        var cmd = runEnd(() -> motor.setVoltage(4.0), this::stop);
        return cmd;
    }

    public void moveHopper(HopperState setpointState) {
        double volts;
        if (setpointState == HopperState.Stowed) {
            volts = -4.0;
        } else {
            volts = 4.0;
        }
        motor.setVoltage(volts);
    }

    public boolean isAtSetPoint() {
        var error = Math.abs(setpointState.hopperPosition - relativeEncoder.getPosition());
        return (error < 0.5);
    }

    public Command retractHopperCommand() {
        var setHopperState = runOnce(() -> setpointState = HopperState.Stowed);

        var cmd = (this.run(() -> moveHopper(HopperState.Stowed)).withTimeout(1.0)).andThen(setHopperState)
                .andThen(this::stop);
        return cmd;
    }

    public Command deployHopperCommand() {
        var setHopperState = runOnce(() -> setpointState = HopperState.Deployed);

        var cmd = (this.run(() -> moveHopper(HopperState.Deployed)).withTimeout(1.5)).andThen(setHopperState)
                .andThen(this::stop);
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
