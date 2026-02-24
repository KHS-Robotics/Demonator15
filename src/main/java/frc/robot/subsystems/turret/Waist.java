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
        .inverted(true);

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

  public Command setDegreesCommand(double angleDegrees) {
    // 0 degrees should be perpendicular with the back
    var cmd = this.run(() -> setSetpointDegrees(angleDegrees)).until(this::isAtSetpoint);
    return cmd.withName("SetWaistSetpoint");
  }

  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopWaist");
  }

  public double getDegrees() {
    return relativeEncoder.getPosition();
  }

  public double getAbsoluteReading() {
    return absoluteEncoder.getPosition();
  }

  public void setSetpointDegrees(double setpointDegrees) {
    // only reset for new setpoints
    setpointRotationDegrees = MathUtil.clamp(setpointDegrees, TurretConfig.WaistConfig.kMinSoftLimit,
        TurretConfig.WaistConfig.kMinSoftLimit);
    pid.setSetpoint(setpointDegrees, ControlType.kPosition);
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointRotationDegrees - getDegrees());
    return (error < 1);
  }

  private double getAngleToPosition(Translation2d fromPose, Translation2d toPose) {
    double yDistanceTo = toPose.getY() - fromPose.getY();
    double xDistanceTo = toPose.getX() - fromPose.getX();

    var angleToHub = Math.atan2(yDistanceTo, xDistanceTo);
    return angleToHub;
  }

  public Command aimWaistSimple(Translation2d towards) {
    // the subtracting of the yaw is to account for the rotation of the robot
    double angle = getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), towards)
        - RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
    // clamping the value because our turret only goes 240(?) degrees
    angle = Math.toDegrees(angle) % 360;
    angle = MathUtil.clamp(angle, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);
    // + TurretConfig.WaistConfig.kWaistDegreesOffset);
    var cmd = setDegreesCommand(angle);
    return cmd;
  }

  public Rotation2d aimWaistSimpleAngle(Translation2d towards) {
    // the subtracting of the yaw is to account for the rotation of the robot
    double angle = getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), towards)
        - RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
    // clamping the value because our turret only goes 240(?) degrees
    angle = Math.toDegrees(angle) % 360;
    angle = MathUtil.clamp(angle, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);
    var finalAngle = new Rotation2d(Math.toRadians(angle));
    // + TurretConfig.WaistConfig.kWaistDegreesOffset);
    return finalAngle;
  }

  public Rotation2d aimWaistSimpleAngleFieldRelative(Translation2d towards) {
    // the subtracting of the yaw is to account for the rotation of the robot
    double angle = getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), towards)
        - RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
    // clamping the value because our turret only goes 240(?) degrees
    angle = Math.toDegrees(angle) % 360;
    angle = MathUtil.clamp(angle, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);
    var finalAngle = new Rotation2d(Math.toRadians(angle + RobotContainer.kSwerveDrive.getPose().getRotation().getDegrees()));
    // + TurretConfig.WaistConfig.kWaistDegreesOffset);
    return finalAngle;
  }

  private void updateSetpointsForDisabledMode() {
    if (RobotState.isDisabled()) {
      setSetpointDegrees(getDegrees());
    }
  }

  public void stop() {
    motor.stopMotor();
    setSetpointDegrees(getDegrees());
  }

  public void periodic() {
    updateSetpointsForDisabledMode();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Angle", () -> getDegrees(), null);
    builder.addDoubleProperty("AngleAbsolute", () -> this.getAbsoluteReading(), null);
  }

}
// yaw adjustment
// o–(•o•)–o
// / \
// o o yay!!!
