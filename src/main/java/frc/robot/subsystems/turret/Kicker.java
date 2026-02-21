package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Kicker extends SubsystemBase { 
    private final SparkFlex motor;

    public Kicker() {
        super(Kicker.class.getSimpleName() + "/" + Kicker.class.getSimpleName());

        var motorConfig = new SparkFlexConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(true);
        motor = new SparkFlex(RobotMap.TURRET_KICKER_MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartDashboard.putData(getName(), this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        return cmd.withName("StopKicker");
    }

    public void start() {
        motor.setVoltage(12);
    }

    public Command startCommand() {
        var cmd = runOnce(this::start);
        return cmd.withName("StartKicker");
    }

}

// indexer to tower motor <- put the belt into indexer

// pre-shooter motor
