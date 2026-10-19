/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.AngleUtils;

/**
 * One swerve corner: NEO drive on a Spark MAX (velocity + feedforward) and pivot on a Spark MAX with
 * {@link CANcoder} absolute angle.
 *
 * <p>Applies WPILib-style module optimization via {@link SwerveModuleOptimization}. On desktop,
 * attaches {@link SparkMaxSim} for physics simulation in {@link #simulationPeriodic()}.
 *
 * @see SwerveDrive
 */
public class SwerveModule extends SubsystemBase {
  /** Module label (e.g. {@code FrontLeft}) for logging and Shuffleboard. */
  public final String name;
  /** Calibrated CANcoder offset added to absolute reading (degrees). */
  public final double offsetAngle;

  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePid;
  private final SimpleMotorFeedforward driveFeedForward;

  private final CANcoder pivotEncoder;
  private final SparkMax pivotMotor;
  private final PIDController pivotPid;

  private boolean isCurrentlyFlippedForShorterPath;

  private final SparkMaxSim driveSim;
  private final SparkMaxSim pivotSim;

  /**
   * @param name module corner name
   * @param driveMotorChannel CAN ID of the drive Spark MAX
   * @param pivotMotorChannel CAN ID of the pivot Spark MAX
   * @param pivotP pivot PID proportional gain
   * @param pivotI pivot PID integral gain
   * @param pivotD pivot PID derivative gain
   * @param driveP drive PID proportional gain (Spark closed-loop)
   * @param driveI drive PID integral gain
   * @param driveD drive PID derivative gain
   * @param drivekS drive feedforward kS (V)
   * @param drivekV drive feedforward kV (V·s/m)
   * @param drivekA drive feedforward kA (V·s²/m)
   * @param pivotEncoderId CAN ID of the pivot CANcoder
   * @param offsetAngle pivot encoder offset (degrees)
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int pivotMotorChannel,
      double pivotP,
      double pivotI,
      double pivotD,
      double driveP,
      double driveI,
      double driveD,
      double drivekS,
      double drivekV,
      double drivekA,
      int pivotEncoderId,
      double offsetAngle) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, driveP, driveI, driveD, drivekS, drivekV,
        drivekA, pivotEncoderId, false, offsetAngle);
  }

  /**
   * Full constructor including drive motor inversion.
   *
   * @param driveInverted true if the drive motor direction is reversed on the CAN bus
   */
  public SwerveModule(
      String name,
      int driveMotorChannel,
      int pivotMotorChannel,
      double pivotP,
      double pivotI,
      double pivotD,
      double driveP,
      double driveI,
      double driveD,
      double drivekS,
      double drivekV,
      double drivekA,
      int pivotEncoderId,
      boolean driveInverted,
      double offsetAngle) {
    super(SwerveDrive.class.getSimpleName() + "/Module-" + name);
    this.name = name;
    this.offsetAngle = offsetAngle;

    EncoderConfig driveEncoderConfig = new EncoderConfig()
        .positionConversionFactor(SwerveDriveConfig.kDriveEncoderPositionConversionFactor)
        .velocityConversionFactor(SwerveDriveConfig.kDriveEncoderVelocityConversionFactor);
    ClosedLoopConfig driveClosedLoopConfig = new ClosedLoopConfig()
        .pid(driveP, driveI, driveD, ClosedLoopSlot.kSlot0);
    var driveMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(driveInverted)
        .apply(driveEncoderConfig)
        .apply(driveClosedLoopConfig);

    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    drivePid = driveMotor.getClosedLoopController();
    driveFeedForward = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);
    driveEncoder = driveMotor.getEncoder();

    var pivotMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    pivotMotor = new SparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = new CANcoder(pivotEncoderId);
    pivotPid = new PIDController(pivotP, pivotI, pivotD);
    pivotPid.enableContinuousInput(-180, 180);
    pivotPid.setTolerance(1);

    if (!RobotBase.isReal()) {
      DCMotor neo = DCMotor.getNEO(1);
      driveSim = new SparkMaxSim(driveMotor, neo);
      pivotSim = new SparkMaxSim(pivotMotor, neo);
      driveSim.enable();
      pivotSim.enable();
    } else {
      driveSim = null;
      pivotSim = null;
    }

    SmartDashboard.putData(this);
  }

  /**
   * Advances REV {@link SparkMaxSim} physics for drive and pivot (desktop / unit tests only).
   */
  @Override
  public void simulationPeriodic() {
    if (driveSim == null) {
      return;
    }
    double busVoltage = RoboRioSim.getVInVoltage();
    driveSim.iterate(driveSim.getVelocity(), busVoltage, 0.02);
    pivotSim.iterate(pivotSim.getVelocity(), busVoltage, 0.02);
  }

  /** Steps REV physics sim; used by unit tests and robot simulation. */
  void simulationStep() {
    simulationPeriodic();
  }

  /** Whether the last {@link #setDesiredState} chose the inverted-drive optimization path. */
  boolean isOptimizingAngle() {
    return isCurrentlyFlippedForShorterPath;
  }

  /** @return CTRE sim state for injecting pivot angle in simulation */
  CANcoderSimState pivotEncoderSimState() {
    return pivotEncoder.getSimState();
  }

  /** Forces a fresh CANcoder absolute position sample (sim / tests). */
  void refreshPivotAbsolutePosition() {
    BaseStatusSignal.refreshAll(pivotEncoder.getAbsolutePosition());
  }

  /** @return pivot {@link SparkMaxSim}, or null on the roboRIO */
  SparkMaxSim pivotSparkSim() {
    return pivotSim;
  }

  /**
   * Closes Spark and CANcoder handles. Call from sim tests so CAN IDs can be reused in the same JVM.
   */
  public void close() {
    driveMotor.close();
    pivotMotor.close();
    pivotEncoder.close();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Speed", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("Angle", () -> getState().angle.getDegrees(), null);
    builder.addBooleanProperty("IsOptimizingAngle", () -> isCurrentlyFlippedForShorterPath, null);
    builder.addDoubleProperty("OffsetAngle", () -> offsetAngle, null);
  }

  /** @return measured drive velocity and pivot angle as a {@link SwerveModuleState} */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /** @return integrated drive distance and pivot angle for odometry */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Commands drive speed and pivot angle with shortest-path optimization enabled.
   *
   * @param state desired wheel speed (m/s) and module heading
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * @param state desired wheel speed and heading
   * @param useShortestPath if true, may flip drive direction per {@link SwerveModuleOptimization}
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    double currentPivotAngle = getAngle();
    SwerveModuleOptimization.PivotTarget pivotTarget;
    if (useShortestPath) {
      pivotTarget = SwerveModuleOptimization.shortestPivotTarget(currentPivotAngle, state.angle.getDegrees());
      isCurrentlyFlippedForShorterPath = pivotTarget.driveInverted();
    } else {
      pivotTarget = new SwerveModuleOptimization.PivotTarget(state.angle.getDegrees(), false);
      isCurrentlyFlippedForShorterPath = false;
    }
    double targetPivotAngle = pivotTarget.angleDegrees();
    double pivotOutput = MathUtil.clamp(pivotPid.calculate(currentPivotAngle, targetPivotAngle), -1, 1);
    pivotMotor.set(pivotOutput);

    int sign = pivotTarget.driveInverted() && useShortestPath ? -1 : 1;
    double cosineComp = SwerveModuleOptimization.cosineScale(currentPivotAngle, targetPivotAngle);
    double driveOutput = sign * cosineComp * state.speedMetersPerSecond;
    drivePid.setSetpoint(
        driveOutput,
        SparkMax.ControlType.kVoltage,
        ClosedLoopSlot.kSlot0,
        driveFeedForward.calculate(driveOutput));
  }

  /**
   * @param speedMetersPerSecond wheel linear speed (m/s)
   * @param angleDegrees module azimuth in degrees [-180, 180]
   */
  public void setDesiredState(double speedMetersPerSecond, double angleDegrees) {
    setDesiredState(speedMetersPerSecond, angleDegrees, true);
  }

  public void setDesiredState(double speedMetersPerSecond, double angleDegrees, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleDegrees)), useShortestPath);
  }

  /**
   * @return pivot angle in degrees [-180, 180], from CANcoder rotations plus {@link #offsetAngle}
   */
  public double getAngle() {
    double angle = pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360 + offsetAngle;
    return AngleUtils.normalizeDegrees(angle);
  }

  /** Sets drive and pivot outputs to zero and resets pivot PID. */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPid.reset();
  }

}
