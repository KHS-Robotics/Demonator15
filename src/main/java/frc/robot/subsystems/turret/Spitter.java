package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import frc.robot.RobotMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Spitter extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkClosedLoopController pid;
  private final RelativeEncoder relativeEncoder;

  public Spitter() {
    super(Spitter.class.getSimpleName() + "/" + Spitter.class.getSimpleName());

    var pidConfig = new ClosedLoopConfig()
      .pid(TurretConfig.SpitterConfig.kSpitterP, TurretConfig.SpitterConfig.kSpitterI, TurretConfig.SpitterConfig.kSpitterD)
      .iZone(200)
      .minOutput(0)
      .maxOutput(1);
    pidConfig.feedForward.kV(0.00179);

    var encoderConfig = new EncoderConfig()
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    var motorConfig = new SparkFlexConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60, 30)
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .apply(pidConfig)
        .apply(encoderConfig);
    motor = new SparkFlex(RobotMap.TURRET_SPITTER_LEADER_ID, MotorType.kBrushless);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pid = motor.getClosedLoopController();

    relativeEncoder = motor.getEncoder();

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
    pid.setSetpoint(TurretConfig.SpitterConfig.kSpitterRPM, ControlType.kVelocity);
  }

  public double getAppliedOutput(){
    return motor.getAppliedOutput();
  }

  public boolean isAtSetpoint() {
    return Math.abs(relativeEncoder.getVelocity() - TurretConfig.SpitterConfig.kSpitterRPM) < 50;
  }

  public Command startCommand() {
    var cmd = this.run(() -> this.start()).until(this::isAtSetpoint);
    return cmd.withName("StartSpitter");

  }

  public double getVelocity(){
    return relativeEncoder.getVelocity();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("Flywheel Speed", () -> getVelocity(), null);
      builder.addDoubleProperty("Flywheel Error", () -> Math.abs(getVelocity() - TurretConfig.SpitterConfig.kSpitterRPM), null);
      builder.addDoubleProperty("Flywheel Applied Output", () -> this.getAppliedOutput(), null);
  }
}
// two shooter motors
//