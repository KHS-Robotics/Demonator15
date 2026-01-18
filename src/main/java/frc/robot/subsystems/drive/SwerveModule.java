/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;


/**
 * Swerve module for a swerve drive style drivetrain.
 * 
 * @see {@link frc.robot.subsystems.drive.SwerveDrive}
 */
public class SwerveModule extends SubsystemBase {
  public final String name;
  public final double offsetAngle;

  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController drivePID;
  private final SimpleMotorFeedforward driveFeedForward;

  private final CANcoder pivotEncoder;
  private final SparkMax pivotMotor;
  private final PIDController pivotPID;

  private boolean isCurrentlyFlippedForShorterPath;

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   * @param driveP            P value of Drive PID
   * @param driveI            I value of Drive PID
   * @param driveD            D value of Drive PID
   * @param pivotEncoderId    port number for the pivot CANCoder
   * @param reversed          true if drive motor is reversed
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, boolean reversed, double offsetAngle) {
    super(SwerveDrive.class.getSimpleName() + "/Module-" + name);
    this.name = name;

    var driveEncoderConfig = new EncoderConfig()
        .positionConversionFactor(SwerveDriveConfig.kDriveEncoderPositionConversionFactor)
        .velocityConversionFactor(SwerveDriveConfig.kDriveEncoderVelocityConversionFactor);
    var driveClosedLoopConfig = new ClosedLoopConfig()
        .pid(driveP, driveI, driveD, ClosedLoopSlot.kSlot0);
    var driveMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(45)
        .inverted(reversed)
        .apply(driveEncoderConfig)
        .apply(driveClosedLoopConfig);
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    drivePID = driveMotor.getClosedLoopController();
    driveFeedForward = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);
    driveEncoder = driveMotor.getEncoder();

    var pivotMotorConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    pivotMotor = new SparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.configure(pivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    pivotEncoder = new CANcoder(pivotEncoderId);
    pivotPID = new PIDController(pivotP, pivotI, pivotD);
    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(1);

    this.offsetAngle = offsetAngle;

    SmartDashboard.putData(this);
  }

  /**
   * Constructs a Swerve Module.
   *
   * @param name              the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP            P value of Pivot PID
   * @param pivotI            I value of Pivot PID
   * @param pivotD            D value of Pivot PID
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI,
      double pivotD, double driveP, double driveI, double driveD,
      double drivekS, double drivekV, double drivekA, int pivotEncoderId, double offsetAngle) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, driveP, driveI,
        driveD, drivekS, drivekV, drivekA, pivotEncoderId, false, offsetAngle);
  }

  /** {@inheritDoc} */
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

  /** {@inheritDoc} */
  @Override
  public void periodic() {
  }

  /**
   * Gets the current drive speed and pivot angle of the module.
   * 
   * @return the current drive speed and pivot angle of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Gets the current drive distance traveled and pivot angle of the module.
   * 
   * @return the current drive distance traveled and pivot angle of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the PID values for the pivot module.
   *
   * @param p the P value for the pivot module
   * @param i the I value for the pivot module
   * @param d the D value for the pivot module
   */
  public void setPivotPID(double p, double i, double d) {
    pivotPID.setPID(p, i, d);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state           desired state with the speed and angle
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    // pivot
    var currentPivotAngle = getAngle();
    var targetPivotAngle = useShortestPath ? calculateShortestPath(state.angle.getDegrees()) : state.angle.getDegrees();
    var pivotOutput = MathUtil.clamp(pivotPID.calculate(currentPivotAngle, targetPivotAngle), -1, 1);
    pivotMotor.set(pivotOutput);

    // drive
    var sign = isCurrentlyFlippedForShorterPath && useShortestPath ? -1 : 1;
    var cosineComp = calculateCosineCompensation(currentPivotAngle, targetPivotAngle);
    var driveOutput = sign * cosineComp * state.speedMetersPerSecond;
    drivePID.setReference(driveOutput, SparkMax.ControlType.kVoltage, ClosedLoopSlot.kSlot0,
        driveFeedForward.calculate(driveOutput));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state desired state with the speed and angle
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed           the desired speed in meters/second
   * @param angle           the desired angle in degrees from [-180, 180]
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(double speed, double angle, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)), useShortestPath);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   */
  public void setDesiredState(double speed, double angle) {
    setDesiredState(speed, angle, true);
  }

  /**
   * Gets the angle of the pivot module.
   *
   * @return the angle of the pivot module ranging from [-180,180]
   */
  public double getAngle() {
    double voltage = pivotEncoder.getAbsolutePosition().getValueAsDouble();
    double angle = voltage * 360 + offsetAngle;

    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }

    return angle;
  }

  /**
   * Stops the module and resets the pivot PID controller.
   */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPID.reset();
  }

  /**
   * Calculates the shortest path the pivot module should take, it
   * might be the given <code>targetAngle</code>. Flips the drive motor
   * if there is a shorter path.
   * 
   * <p>
   * 
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
   *
   * @param targetAngle the desired angle of the module
   * @return the shortest path to the target angle, flips the
   *         drive motor if there is a shorter path
   */
  private double calculateShortestPath(double targetAngle) {
    var currentAngle = getAngle();
    var dAngle = Math.abs(targetAngle - currentAngle);

    isCurrentlyFlippedForShorterPath = dAngle > 90 && dAngle < 270;

    if (isCurrentlyFlippedForShorterPath) {
      if (targetAngle > 0 || targetAngle == 0 && currentAngle < 0) {
        targetAngle -= 180;
      } else if (targetAngle < 0 || targetAngle == 0 && currentAngle > 0) {
        targetAngle += 180;
      }
    }

    return targetAngle;
  }

  /**
   * Control optimization technique to reduce the amount the module translates
   * before reaching the target pivot angle.
   * 
   * <p>
   * 
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
   * 
   * @param currentAngle the current angle of the module
   * @param targetAngle  the target angle of the module
   * @return the cosine of the angle between the given angles
   */
  private double calculateCosineCompensation(double currentAngle, double targetAngle) {
    var angleError = Math.abs(currentAngle - targetAngle);
    return Math.cos(Math.toRadians(angleError));
  }
}