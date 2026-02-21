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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

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

  private static Transform2d getTurretOffset() {
    return new Transform2d(
        Units.inchesToMeters(TurretConfig.WaistConfig.kTurretOffsetXInches),
        Units.inchesToMeters(TurretConfig.WaistConfig.kTurretOffsetYInches),
        Rotation2d.fromDegrees(0));
  }

  /**
   * Applies a field-relative aim angle: converts to robot-relative, checks
   * limits, clamps, sets waist setpoint, and updates GUI.
   */
  private void applyWaistAimToFieldAngle(Pose2d robotPose, Pose2d turretPose, Rotation2d fieldRelativeAngle) {
    Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

    // R106: turret can't point outside min/max
    hasImpossibleShot = false;
    if (robotRelativeAngle.getDegrees() < TurretConfig.WaistConfig.kMinSoftLimit) {
      hasImpossibleShot = true;
    }
    if (robotRelativeAngle.getDegrees() > TurretConfig.WaistConfig.kMaxSoftLimit) {
      hasImpossibleShot = true;
    }

    robotRelativeAngleForHub = robotRelativeAngle.getDegrees();
    fieldRelativeAngleForHub = fieldRelativeAngle.getDegrees();
    var targetAngleForWaist = MathUtil.clamp(robotRelativeAngleForHub, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);

    setSetpointDegrees(targetAngleForWaist);

    // Field widget: show turret pose and aim direction
    var turretFieldRelativePose = new Pose2d(turretPose.getX(), turretPose.getY(), fieldRelativeAngle);
    RobotContainer.kField.getObject("Turret").setPose(turretFieldRelativePose);
  }

  /**
   * REF:
   * https://www.chiefdelphi.com/t/turret-tracking-hub-using-odometry/512844/7
   * Aims waist at hub (stationary robot). For shooting while moving, use
   * defaultAimWaistToHubWhileMoving().
   */
  public Command defaultAimWaistToHub() {
    var cmd = this.runEnd(() -> {
      Pose2d robotPose = RobotContainer.kSwerveDrive.getPose();
      Pose2d turretPose = robotPose.plus(getTurretOffset());

      var target = TurretConfig.TurretFieldAndRobotInfo.getCurrentHubPosition();
      // Vector from turret to hub
      double dY = target.getY() - turretPose.getY();
      double dX = target.getX() - turretPose.getX();
      Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));

      applyWaistAimToFieldAngle(robotPose, turretPose, fieldRelativeAngle);
    }, this::stop);
    return cmd.withName("DefaultWaistAimToHub");
  }

  public Command defaultAimWaistToHubWhileMoving(DoubleSupplier hoodAngleDegrees) {
    var cmd = this.runEnd(() -> {
      Pose2d robotPose = RobotContainer.kSwerveDrive.getPose();
      Transform2d turretOffset = getTurretOffset();
      Pose2d turretPose = robotPose.plus(turretOffset);

      var target = TurretConfig.TurretFieldAndRobotInfo.getCurrentHubPosition();

      Translation2d toHub = target.minus(turretPose.getTranslation());
      double distToHub = toHub.getNorm();
      if (distToHub < 1e-6) {
        toHub = new Translation2d(1, 0);
        distToHub = 1;
      }
      // Hood 0° = barrel up, 90° = horizontal; horizontal component = v*sin(hood)
      double horizontalSpeed = TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity
          * Math.sin(Math.toRadians(hoodAngleDegrees.getAsDouble()));
      Translation2d desiredFuelVelocityField = toHub.times(horizontalSpeed / distToHub);

      // Robot velocity at turret (field frame); centerPoint so omega is included if offset
      Translation2d turretCenterInRobot = new Translation2d(turretOffset.getX(), turretOffset.getY());
      ChassisSpeeds robotRelativeSpeeds = RobotContainer.kSwerveDrive.getChassisSpeedsAt(turretCenterInRobot);
      Translation2d robotVelocityField = new Translation2d(
          robotRelativeSpeeds.vxMetersPerSecond,
          robotRelativeSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

      // Launch = desired - scale*robot; scale tunable if over/under-correcting (1.0 = full comp)
      Translation2d launchVelocityField = desiredFuelVelocityField.minus(
          robotVelocityField.times(TurretConfig.WaistConfig.kVelocityCompScale));
      double launchNorm = launchVelocityField.getNorm();
      Rotation2d fieldRelativeAngle;
      if (launchNorm < 1e-6) {
        // Fallback: aim at hub when launch vector negligible (e.g. stationary)
        fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(toHub.getY(), toHub.getX()));
      } else {
        fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(launchVelocityField.getY(), launchVelocityField.getX()));
      }

      applyWaistAimToFieldAngle(robotPose, turretPose, fieldRelativeAngle);
    }, this::stop);
    return cmd.withName("DefaultWaistAimToHubWhileMoving");
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
    return relativeEncoder.getPosition();
  }

  public void setSetpointDegrees(double setpointDegrees) {
    double clamped = MathUtil.clamp(setpointDegrees, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);
    setpointRotationDegrees = clamped;
    pid.setSetpoint(clamped, ControlType.kPosition);
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
