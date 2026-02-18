package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Spitter extends SubsystemBase {

  private final SparkMax motor;

  public Spitter() {
    super(Spitter.class.getSimpleName() + "/" + Spitter.class.getSimpleName());

    var motorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true);
    motor = new SparkMax(RobotMap.TURRET_SPITTER_LEADER_ID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putData(getName(), this);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopSpitter");
  }

  public void start() {
    motor.setVoltage(2.75);
  }

  public Command startCommand() {
    var cmd = runOnce(this::start);
    return cmd.withName("StartSpitter");

  }
}
// two shooter motors
//