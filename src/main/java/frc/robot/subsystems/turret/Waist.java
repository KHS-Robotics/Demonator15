package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;

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
  // private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;

  private double setpointRotationDegrees;

  private boolean hasImpossibleShot;
  private double robotRelativeAngleForHub, fieldRelativeAngleForHub;

  public Waist() {

    var relativeEncoderConfig = new EncoderConfig()
        .positionConversionFactor(TurretConfig.WaistConfig.kWaistEncoderPositionConversionFactor)
        .velocityConversionFactor(TurretConfig.WaistConfig.kWaistEncoderVelocityConversionFactor);

    // var absoluteEncoderConfig = new AbsoluteEncoderConfig()
    // .inverted(true);

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
        .apply(softLimitConfig);
    // .apply(absoluteEncoderConfig);

    motor = new SparkMax(RobotMap.TURRET_AIMER_WAIST_ID, MotorType.kBrushless);
    motor.configure(waistConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // absoluteEncoder = motor.getAbsoluteEncoder();

    relativeEncoder = motor.getEncoder();

    pid = motor.getClosedLoopController();
  
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addBooleanProperty("Hub-HasImpossibleShot", () -> hasImpossibleShot, null);
    builder.addDoubleProperty("Hub-TurretRobotRelativeAngle", () -> robotRelativeAngleForHub, null);
    builder.addDoubleProperty("Hub-TurretFieldRelativeAngle", () -> fieldRelativeAngleForHub, null);
  }

  /**
   * REF:
   * https://www.chiefdelphi.com/t/turret-tracking-hub-using-odometry/512844/7
   * 
   * @param target the target to aim at (eg hardcoded red or blue Hub positions
   *               depending on your current alliance)
   */
  public Command defaultAimWaistToHub() {
    var cmd = this.runEnd(() -> {
      // Get robot pose with turret offset
      Pose2d robotPose = RobotContainer.kSwerveDrive.getPose();
      // Apply turret offset to robot pose
      // TODO: save turret X+Y offsets wrt to center of robot to TurretConfig
      Transform2d turretOffset = new Transform2d(
          Units.inchesToMeters(0), // x offset of turret
          Units.inchesToMeters(0), // y offset of turret
          Rotation2d.fromDegrees(0));
      Pose2d turretPose = robotPose.plus(turretOffset);

      // Calculate vector to target
      var target = TurretConfig.TurretFieldAndRobotInfo.getCurrentHubPosition();
      double dY = target.getY() - turretPose.getY();
      double dX = target.getX() - turretPose.getX();

      // Calculate field-relative angle to target
      Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));

      // Convert to robot-relative angle
      Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

      // Robot turret has min/max limits due to potential R106 violations (see game manual)
      hasImpossibleShot = false;
      if (robotRelativeAngle.getDegrees() < TurretConfig.WaistConfig.kMinSoftLimit) {
        hasImpossibleShot = true;
      }
      if (robotRelativeAngle.getDegrees() > TurretConfig.WaistConfig.kMaxSoftLimit) {
        hasImpossibleShot = true;
      }

      // we care about robotRelativeAngle for the turret...
      // but also save fieldRelativeAngle for sanity's sake+debugging
      robotRelativeAngleForHub = robotRelativeAngle.getDegrees();
      fieldRelativeAngleForHub = fieldRelativeAngle.getDegrees();

      // Clamp target angle to min/max values although
      var targetAngleForWaist = MathUtil.clamp(robotRelativeAngleForHub, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);

      // Command the turret using clamped robot relative angle
      setSetpointDegrees(targetAngleForWaist);

      // Update GUI
      var turretFieldRelativePose = new Pose2d(turretPose.getX(), turretPose.getY(), fieldRelativeAngle);
      RobotContainer.kField.getObject("Turret").setPose(turretFieldRelativePose);
    }, this::stop);
    return cmd.withName("DefaultWaistAimToHub");
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
    // 0 should be perpendicular with the back
    var angle = Units.rotationsToDegrees(relativeEncoder.getPosition()) * HoodConfig.kRotationsToDegreesConversion;
    return angle;
  }

  public void setSetpointDegrees(double setpointDegrees) {
    // only reset for new setpoints
    setpointRotationDegrees = MathUtil.clamp(setpointDegrees, -180.0, 180.0);
    pid.setSetpoint(setpointDegrees, ControlType.kPosition);
  }

  public boolean isAtSetpoint() {
    var error = Math.abs(setpointRotationDegrees - getDegrees());
    return (error < 1);
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

}
// yaw adjustment
// o–(•o•)–o
// / \
// o o yay!!!
