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

  private final SparkMax leader,follower;

  public Spitter() {
    super(Spitter.class.getSimpleName() + "/" + Spitter.class.getSimpleName());

    var leaderConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true);
    leader = new SparkMax(RobotMap.TURRET_SPITTER_LEADER_MOTOR_ID, MotorType.kBrushless);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .follow(RobotMap.TURRET_SPITTER_LEADER_ID, true);
    follower= new SparkMax(RobotMap.TURRET_SPITTER_FOLLOWER_ID, MotorType.kBrushless);
    follower.configure(followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putData(getName(), this);
  }

  public void stop() {
    leader.stopMotor();
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopSpitter");
  }

  public void start() {
    leader.setVoltage(2.75);
  }

  public Command startCommand() {
    var cmd = runOnce(this::start);
    return cmd.withName("StartSpitter");

  }
}
// two shooter motors
//