/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.constants.FieldConstants;
import frc.robot.util.AngleUtils;

/**
 * Holonomic swerve drivetrain subsystem.
 *
 * <p>Owns four {@link SwerveModule}s, fuses wheel odometry with vision, and exposes low-level
 * {@link #drive(double, double, double, boolean)} for commands and autos. PathPlanner
 * {@link com.pathplanner.lib.auto.AutoBuilder} is configured in the constructor.
 *
 * <p>Driver-facing {@link edu.wpi.first.wpilibj2.command.Command}s are built in
 * {@link frc.robot.commands.drive.DriveCommands}.
 *
 * @see SwerveDriveConfig
 * @see frc.robot.constants.FieldConstants
 */
public class SwerveDrive extends SubsystemBase {
  private static final double kDriveVelocityDeadband = 0.005;
  private static final double kDriveOmegaDeadband = 0.005;

  /** Upper bound on translational speed used by teleop and heading hold (m/s). Tunable on Shuffleboard. */
  public static double maxTranslationalSpeedMetersPerSecond = 4.6;
  /** Upper bound on rotation rate used by teleop and heading hold (rad/s). Tunable on Shuffleboard. */
  public static double maxAngularSpeedRadiansPerSecond = Math.toRadians(540);

  private final SwerveModule kFrontLeft = createModule(
      "FrontLeft", RobotMap.FRONT_LEFT_DRIVE, RobotMap.FRONT_LEFT_PIVOT, RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontLeftPivotOffsetDegrees);
  private final SwerveModule kFrontRight = createModule(
      "FrontRight", RobotMap.FRONT_RIGHT_DRIVE, RobotMap.FRONT_RIGHT_PIVOT, RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontRightPivotOffsetDegrees);
  private final SwerveModule kRearLeft = createModule(
      "RearLeft", RobotMap.REAR_LEFT_DRIVE, RobotMap.REAR_LEFT_PIVOT, RobotMap.REAR_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearLeftPivotOffsetDegrees);
  private final SwerveModule kRearRight = createModule(
      "RearRight", RobotMap.REAR_RIGHT_DRIVE, RobotMap.REAR_RIGHT_PIVOT, RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearRightPivotOffsetDegrees);

  private final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      SwerveDriveConfig.kFrontLeftModuleOffset,
      SwerveDriveConfig.kFrontRightModuleOffset,
      SwerveDriveConfig.kRearLeftModuleOffset,
      SwerveDriveConfig.kRearRightModuleOffset);

  /** Default process noise for pose estimation (x, y, theta). Increase to trust odometry less. */
  public static final Matrix<N3, N1> kDefaultStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  /**
   * Default vision measurement noise (x, y, theta). Theta uses {@link Double#MAX_VALUE} so vision
   * does not correct heading when using wheel + gyro odometry.
   */
  public static final Matrix<N3, N1> kDefaultVisionMeasurementStdDevs = VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE);

  private final SwerveDrivePoseEstimator kPoseEstimator = new SwerveDrivePoseEstimator(
      kSwerveKinematics,
      getAngleForOdometry(),
      getSwerveModulePositions(),
      new Pose2d(),
      kDefaultStateStdDevs,
      kDefaultVisionMeasurementStdDevs);

  private final PIDController thetaPid = new PIDController(
      SwerveDriveConfig.DRIVE_ANGLE_P,
      SwerveDriveConfig.DRIVE_ANGLE_I,
      SwerveDriveConfig.DRIVE_ANGLE_D);

  /** Constructs modules, pose estimator, PathPlanner AutoBuilder, and initial field pose. */
  public SwerveDrive() {
    thetaPid.enableContinuousInput(-180.0, 180.0);
    thetaPid.setTolerance(3);

    configurePathPlannerAutoBuilder();
    resetPose(FieldConstants.kDefaultOdometryPose);

    SmartDashboard.putData(this);
  }

  private static SwerveModule createModule(
      String name, int driveMotorId, int pivotMotorId, int pivotEncoderId, double pivotOffsetDegrees) {
    return new SwerveModule(
        name,
        driveMotorId,
        pivotMotorId,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
        SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
        SwerveDriveConfig.DRIVE_MODULE_P,
        SwerveDriveConfig.DRIVE_MODULE_I,
        SwerveDriveConfig.DRIVE_MODULE_D,
        SwerveDriveConfig.DRIVE_MODULE_KS,
        SwerveDriveConfig.DRIVE_MODULE_KV,
        SwerveDriveConfig.DRIVE_MODULE_KA,
        pivotEncoderId,
        pivotOffsetDegrees);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Pose-X", () -> getPose().getX(), xPoseMeters -> {
      Pose2d currentPose = getPose();
      resetPose(new Pose2d(xPoseMeters, currentPose.getY(), currentPose.getRotation()));
    });
    builder.addDoubleProperty("Pose-Y", () -> getPose().getY(), yPoseMeters -> {
      Pose2d currentPose = getPose();
      resetPose(new Pose2d(currentPose.getX(), yPoseMeters, currentPose.getRotation()));
    });
    builder.addDoubleProperty("Pose-Theta", () -> getPose().getRotation().getDegrees(), thetaPoseDegrees -> {
      Pose2d currentPose = getPose();
      resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(thetaPoseDegrees)));
    });
    builder.addDoubleProperty("MaxTranslationalSpeed", () -> maxTranslationalSpeedMetersPerSecond,
        maxSpeedMetersPerSecond -> maxTranslationalSpeedMetersPerSecond = Math.abs(maxSpeedMetersPerSecond));
    builder.addDoubleProperty("MaxAngularSpeed", () -> Math.toDegrees(maxAngularSpeedRadiansPerSecond),
        maxSpeedDegreesPerSecond -> maxAngularSpeedRadiansPerSecond = Math.toRadians(maxSpeedDegreesPerSecond));
  }

  /** Updates {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} from module encoders and gyro. */
  @Override
  public void periodic() {
    kPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngleForOdometry(), getSwerveModulePositions());
  }

  /** Steps REV {@link com.revrobotics.sim.SparkMaxSim} on each module when running desktop simulation. */
  @Override
  public void simulationPeriodic() {
    kFrontLeft.simulationPeriodic();
    kFrontRight.simulationPeriodic();
    kRearLeft.simulationPeriodic();
    kRearRight.simulationPeriodic();
  }

  /**
   * Fuses a vision pose into odometry (e.g. Limelight/Photon callbacks in {@link frc.robot.RobotContainer}).
   *
   * @param pose field pose from vision
   * @param timestamp measurement time (seconds)
   * @param stdDevs per-axis standard deviations; use {@link #kDefaultVisionMeasurementStdDevs} for defaults
   */
  public void addVisionMeasurementForOdometry(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    kPoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  private void configurePathPlannerAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getChassisSpeeds,
          (speeds, feedforwards) -> setModuleStates(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_D),
              new PIDConstants(
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_D)),
          config,
          () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
          this);
    } catch (Exception ex) {
      ex.printStackTrace();
      DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + ex.getMessage(), false);
    }
  }

  /** @return fused field pose (meters, radians) from odometry and vision */
  public Pose2d getPose() {
    return kPoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the estimated field pose while keeping gyro and module positions.
   *
   * @param pose new field pose
   */
  public void resetPose(Pose2d pose) {
    kPoseEstimator.resetPosition(getAngleForOdometry(), getSwerveModulePositions(), pose);
  }

  /** @return current chassis speeds from measured module states (robot-relative) */
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  private void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = getOptimizedModuleStatesFromChassisSpeeds(chassisSpeeds);
    kFrontLeft.setDesiredState(desiredStates[0]);
    kFrontRight.setDesiredState(desiredStates[1]);
    kRearLeft.setDesiredState(desiredStates[2]);
    kRearRight.setDesiredState(desiredStates[3]);
  }

  private SwerveModuleState[] getOptimizedModuleStatesFromChassisSpeeds(ChassisSpeeds originalSpeeds) {
    ChassisSpeeds optimizedChassisSpeeds = SwerveDynamics.correctForDynamics(originalSpeeds);
    SwerveModuleState[] optimizedModuleStates = kSwerveKinematics.toSwerveModuleStates(optimizedChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(optimizedModuleStates, maxTranslationalSpeedMetersPerSecond);
    return optimizedModuleStates;
  }

  private Rotation2d getAngleForOdometry() {
    return RobotContainer.kNavx.getRotation2d();
  }

  /**
   * Sets chassis motion from teleop, heading hold, or internal callers.
   *
   * @param vxMetersPerSecond forward speed (m/s); positive is robot forward
   * @param vyMetersPerSecond strafe speed (m/s); positive is robot left
   * @param omegaRadiansPerSecond counter-clockwise rotation rate (rad/s)
   * @param fieldRelative if true, vx/vy are relative to the field; if false, robot-relative
   */
  public void drive(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean fieldRelative) {
    if (shouldCoastSwerveModules(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond)) {
      stopModulesButHoldPivotAngle();
      return;
    }

    ChassisSpeeds desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, getPose().getRotation())
        : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    setModuleStates(desiredChassisSpeeds);
  }

  private void stopModulesButHoldPivotAngle() {
    kFrontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontLeft.getAngle())));
    kFrontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontRight.getAngle())));
    kRearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearLeft.getAngle())));
    kRearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearRight.getAngle())));
  }

  /** @return measured module states in order FL, FR, RL, RR */
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        kFrontLeft.getState(),
        kFrontRight.getState(),
        kRearLeft.getState(),
        kRearRight.getState()
    };
  }

  /** @return integrated module positions in order FL, FR, RL, RR (for odometry) */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        kFrontLeft.getPosition(),
        kFrontRight.getPosition(),
        kRearLeft.getPosition(),
        kRearRight.getPosition()
    };
  }

  /** Stops all modules and resets the heading-hold PID controller. */
  public void stop() {
    kFrontRight.stop();
    kFrontLeft.stop();
    kRearRight.stop();
    kRearLeft.stop();
    thetaPid.reset();
  }

  /** Angles modules in an X pattern to resist being pushed on defense or on a slope. */
  public void lock() {
    kFrontRight.setDesiredState(0, -45);
    kFrontLeft.setDesiredState(0, 45);
    kRearRight.setDesiredState(0, 45);
    kRearLeft.setDesiredState(0, -45);
  }

  /**
   * Translates while a PID loop drives robot heading to {@code setpointAngle}.
   *
   * @param xSpeed forward component (m/s)
   * @param ySpeed strafe component (m/s)
   * @param setpointAngle desired field- or robot-relative heading per {@code fieldOriented}
   * @param maxSpeed fraction of {@link #maxAngularSpeedRadiansPerSecond} applied to rotation PID output
   * @param fieldOriented passed through to {@link #drive(double, double, double, boolean)}
   */
  public void holdAngleWhileDriving(
      double xSpeed,
      double ySpeed,
      Rotation2d setpointAngle,
      double maxSpeed,
      boolean fieldOriented) {
    drive(xSpeed, ySpeed, computeHeadingHoldOmegaRadians(thetaPid, getPose().getRotation(), setpointAngle, maxSpeed,
        maxAngularSpeedRadiansPerSecond), fieldOriented);
  }

  /**
   * True when teleop inputs are below {@link #kDriveVelocityDeadband} / {@link #kDriveOmegaDeadband} so modules
   * coast with pivots held (package-visible for unit tests).
   */
  static boolean shouldCoastSwerveModules(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
    return Math.abs(omegaRadiansPerSecond) < kDriveOmegaDeadband
        && Math.abs(vxMetersPerSecond) < kDriveVelocityDeadband
        && Math.abs(vyMetersPerSecond) < kDriveVelocityDeadband;
  }

  /**
   * Rotation rate from heading-hold PID (package-visible for unit tests).
   */
  static double computeHeadingHoldOmegaRadians(
      PIDController thetaPid,
      Rotation2d currentHeading,
      Rotation2d setpointAngle,
      double maxSpeedFraction,
      double maxAngularSpeedRadiansPerSecond) {
    return MathUtil.clamp(
        thetaPid.calculate(
            currentHeading.getDegrees(), AngleUtils.normalizeDegrees(setpointAngle.getDegrees())),
        -maxSpeedFraction,
        maxSpeedFraction) * maxAngularSpeedRadiansPerSecond;
  }

}
