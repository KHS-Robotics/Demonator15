package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class Feeder extends SubsystemBase {
    private final SparkMax motor;

    public Feeder() {
        super(Feeder.class.getSimpleName() + "/" + Feeder.class.getSimpleName());

        var motorConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(true);
        motor = new SparkMax(RobotMap.TURRET_FEEDER_MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SmartDashboard.putData(getName(), this);
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        return cmd.withName("StopFeeder");
    }

    public void start() {
        motor.setVoltage(12);
    }

    public Command startCommand() {
        var cmd = runOnce(this::start);
        return cmd.withName("StartFeeder");
    }
}
