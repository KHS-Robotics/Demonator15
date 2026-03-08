package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.turret.TurretConfig.HoodConfig;
import frc.robot.subsystems.turret.TurretConfig.WaistConfig;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

@SuppressWarnings("unused")
public class Waist extends SubsystemBase {
  private final SparkMax motor;
  private final SparkClosedLoopController pid;
  private final SparkAnalogSensor absoluteEncoder;
  private final RelativeEncoder relativeEncoder;

  private double setpointRotationDegrees;

  public Waist() {

    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(TurretConfig.WaistConfig.kWaistEncoderPositionConversionFactor)
        .velocityConversionFactor(TurretConfig.WaistConfig.kWaistEncoderVelocityConversionFactor);

    var absoluteEncoderConfig = new AbsoluteEncoderConfig()
        .positionConversionFactor(TurretConfig.WaistConfig.kWaistAnalogPositionConversionFactor)
        .velocityConversionFactor(TurretConfig.WaistConfig.kWaistAnalogVelocityConversionFactor)
        .inverted(false);

    var softLimitConfig = new SoftLimitConfig()
        .forwardSoftLimit(TurretConfig.WaistConfig.kMaxSoftLimit)
        .reverseSoftLimit(TurretConfig.WaistConfig.kMinSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    var pidConfig = new ClosedLoopConfig()
        .pid(TurretConfig.WaistConfig.kAimerWaistP, TurretConfig.WaistConfig.kAimerWaistI,
            TurretConfig.WaistConfig.kAimerWaistD)
        .maxOutput(0.35)
        .minOutput(-0.35)
        .iZone(10);

    var waistConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30, 10)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .apply(relativeEncoderConfig)
        .apply(pidConfig)
        .apply(softLimitConfig)
        .apply(absoluteEncoderConfig);

    motor = new SparkMax(RobotMap.TURRET_AIMER_WAIST_ID, MotorType.kBrushless);
    motor.configure(waistConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    absoluteEncoder = motor.getAnalog();

    relativeEncoder = motor.getEncoder();

    pid = motor.getClosedLoopController();
    SmartDashboard.putData(this);

  }

  public Command setDegreesCommand(Supplier<Double> angleDegrees) {
    // 0 degrees should be perpendicular with the back
    var cmd = this.run(() -> setSetpointDegrees(angleDegrees.get())).until(this::isAtSetpoint);
    return cmd.withName("SetWaistSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopWaist");
  }

  public double getDegrees() {
    return relativeEncoder.getPosition();
  }

  public double getAbsoluteDegrees() {
    return absoluteEncoder.getVoltage();
  }

  public double getSetpointRotationDegrees(){
    return setpointRotationDegrees;
  }

  public double getAbsoluteReading() {
    return absoluteEncoder.getPosition();
  }

  public void calibrateRelativeEncoder() {
    double absolutePosition = absoluteEncoder.getPosition() * 1;
    relativeEncoder.setPosition(absolutePosition);
  }

  public Command calibrateRelativeEncoderCommand(){
    var cmd = runOnce(this::calibrateRelativeEncoder);
    return cmd.withName("calibrate relative");
  }

  public void setSetpointDegrees(double setpointDegrees) {
    // only reset for new setpoints
    setpointRotationDegrees = MathUtil.clamp(setpointDegrees, TurretConfig.WaistConfig.kMinSoftLimit,
        TurretConfig.WaistConfig.kMaxSoftLimit);
    pid.setSetpoint(setpointDegrees, ControlType.kPosition);
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointRotationDegrees - getDegrees());
    return (error < 1);
  }

  public double waistError(){
    return Math.abs(setpointRotationDegrees - getDegrees());
  }

  // private void updateSetpointsForDisabledMode() {
  //   if (RobotState.isDisabled()) {
  //     setSetpointDegrees(getDegrees());
  //   }
  // }

  public void stop() {
    motor.stopMotor();
    setSetpointDegrees(getDegrees());
  }

  public void periodic() {
    // updateSetpointsForDisabledMode();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Waist Rotation Angle", () -> getDegrees(), null);
    builder.addDoubleProperty("Waist Rotation Absolute Angle", () -> this.getAbsoluteReading(), null);
    builder.addBooleanProperty("Is Waist At Setpoint?", () -> this.isAtSetpoint(), null);
    builder.addDoubleProperty("Waist Setpoint", () -> this.getSetpointRotationDegrees(), null);
    builder.addDoubleProperty("Waist Error", () -> this.waistError(), null);
    builder.addDoubleProperty("Absolute Waist Voltage", () -> getAbsoluteDegrees(), null);
  }

}
// yaw adjustment
// o–(•o•)–o
// / \
// o o yay!!!
