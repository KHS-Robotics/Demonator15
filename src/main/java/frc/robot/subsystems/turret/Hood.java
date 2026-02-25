package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.HoodConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hood extends SubsystemBase {
  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  //private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;

  private double setpointAngleDegrees;

  public Hood() {

    //var absoluteEncoderConfig = new AbsoluteEncoderConfig()
        //.inverted(true);

    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(TurretConfig.HoodConfig.kHoodEncoderPositionConversionFactor)
        .velocityConversionFactor(TurretConfig.HoodConfig.kHoodEncoderVelocityConversionFactor);

    
    var softLimitConfig = new SoftLimitConfig()
      .forwardSoftLimit(TurretConfig.HoodConfig.kMaxSoftLimit)
      .reverseSoftLimit(TurretConfig.HoodConfig.kMinSoftLimit)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    var pidConfig = new ClosedLoopConfig()
        .pid(TurretConfig.HoodConfig.kHoodP, TurretConfig.HoodConfig.kHoodI, TurretConfig.HoodConfig.kHoodD)
        .iZone(3)
        .minOutput(-0.5)
        .maxOutput(0.5);

    var hoodConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(true)
        .apply(relativeEncoderConfig)
        .apply(softLimitConfig)
        .apply(pidConfig)
        .idleMode(IdleMode.kBrake);
        //.apply(absoluteEncoderConfig);

    motor = new SparkMax(RobotMap.TURRET_AIMER_HOOD_ID, MotorType.kBrushless);
    motor.configure(hoodConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    relativeEncoder = motor.getEncoder();

    //absoluteEncoder = motor.getAbsoluteEncoder();

    pid = motor.getClosedLoopController();
    SmartDashboard.putData(this);
  }
  // use angler as base

  public Command setAngleCommand(double angleDegrees) {
    var cmd = this.run(() -> setSetpointAngle(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetHoodSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopHood");
  }

  public double getAngle() {
    return relativeEncoder.getPosition();
  }

  public void setSetpointAngle(double setpointDegrees) {
    // only reset for new setpoints
    setpointAngleDegrees = setpointDegrees;
    pid.setSetpoint(setpointDegrees, ControlType.kPosition);
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointAngleDegrees - getAngle());
    return (error < 2);
  }

  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointAngle(getAngle());
    }
  }

  public void stop() {
    motor.stopMotor();
    setSetpointAngle(getAngle());
  }

  public void periodic() {
    updateSetpointsForDisabledMode();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Setpoint", () -> setpointAngleDegrees, this::setSetpointAngle);
    builder.addDoubleProperty("Angle", () -> this.getAngle(), null);
    builder.addBooleanProperty("IsAtSetpoint", () -> this.isAtSetpoint(), null);
  }
}

// pitch adjustment