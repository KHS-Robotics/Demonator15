/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotMap;
import frc.robot.hid.HIDUtils;
import frc.robot.subsystems.cameras.AprilTagTarget;
import frc.robot.RobotContainer;

/**
 * Swerve drive style drivetrain.
 * 
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
 * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
 */
public class SwerveDrive extends SubsystemBase {
  /** Deadband for X and Y input speeds for joystick driving. */
  private static final double kDriveVelocityDeadband = 0.005;
  /** Deadband for rotational input speed for joystick driving. */
  private static final double kDriveOmegaDeadband = 0.005;

  /** Max allowable translational speed of the robot in meters per second. */
  public static double maxTranslationalSpeedMetersPerSecond = 4.6;
  /** Max allowable angular speed of the robot in radians per second. */
  public static double maxAngularSpeedRadiansPerSecond = Math.toRadians(540);

  private Translation2d centerOfRotation = Translation2d.kZero;
  private Rotation2d headingToHold = Rotation2d.fromDegrees(0);

  private final SwerveModule kFrontLeft = new SwerveModule(
      "FrontLeft",
      RobotMap.FRONT_LEFT_DRIVE,
      RobotMap.FRONT_LEFT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.FRONT_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontLeftPivotOffsetDegrees);
  private final SwerveModule kFrontRight = new SwerveModule(
      "FrontRight",
      RobotMap.FRONT_RIGHT_DRIVE,
      RobotMap.FRONT_RIGHT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.FRONT_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kFrontRightPivotOffsetDegrees);
  private final SwerveModule kRearLeft = new SwerveModule(
      "RearLeft",
      RobotMap.REAR_LEFT_DRIVE,
      RobotMap.REAR_LEFT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.REAR_LEFT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearLeftPivotOffsetDegrees);
  private final SwerveModule kRearRight = new SwerveModule(
      "RearRight",
      RobotMap.REAR_RIGHT_DRIVE,
      RobotMap.REAR_RIGHT_PIVOT,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_P,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_I,
      SwerveDriveConfig.DRIVE_MODULE_PIVOT_D,
      SwerveDriveConfig.DRIVE_MODULE_P,
      SwerveDriveConfig.DRIVE_MODULE_I,
      SwerveDriveConfig.DRIVE_MODULE_D,
      SwerveDriveConfig.DRIVE_MODULE_KS,
      SwerveDriveConfig.DRIVE_MODULE_KV,
      SwerveDriveConfig.DRIVE_MODULE_KA,
      RobotMap.REAR_RIGHT_PIVOT_ENCODER,
      SwerveDriveConfig.kRearRightPivotOffsetDegrees);

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
   */
  private final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      SwerveDriveConfig.kFrontLeftModuleOffset, SwerveDriveConfig.kFrontRightModuleOffset,
      SwerveDriveConfig.kRearLeftModuleOffset, SwerveDriveConfig.kRearRightModuleOffset);

  /**
   * Standard deviations of the pose estimate (x position in meters, y position in
   * meters, and heading in radians).
   * <p>
   * Increase these numbers to trust the state estimate less.
   */
  public static final Matrix<N3, N1> kDefaultStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision pose measurement (x position in meters, y
   * position in meters, and heading in radians).
   * <p>
   * Increase these numbers to trust the vision pose measurement less.
   */
  public static final Matrix<N3, N1> kDefaultVisionMeasurementStdDevs = VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE);

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
   */
  private final SwerveDrivePoseEstimator kPoseEstimator = new SwerveDrivePoseEstimator(
      kSwerveKinematics,
      getAngleForOdometry(),
      getSwerveModulePositions(),
      new Pose2d(),
      kDefaultStateStdDevs,
      kDefaultVisionMeasurementStdDevs);

  private final PIDController xPid = new PIDController(SwerveDriveConfig.DRIVE_X_P, SwerveDriveConfig.DRIVE_X_I,
      SwerveDriveConfig.DRIVE_X_D);
  private final PIDController yPid = new PIDController(SwerveDriveConfig.DRIVE_Y_P, SwerveDriveConfig.DRIVE_Y_I,
      SwerveDriveConfig.DRIVE_Y_D);
  private final PIDController thetaPid = new PIDController(SwerveDriveConfig.DRIVE_ANGLE_P,
      SwerveDriveConfig.DRIVE_ANGLE_I, SwerveDriveConfig.DRIVE_ANGLE_D);

  // for april tag alignment
  private double errorX, errorY;
  private Optional<AprilTagTarget> targetOpt = Optional.empty();

  /**
   * Constructs the Swerve Drive.
   */
  public SwerveDrive() {
    // meters
    xPid.setTolerance(0.1);
    yPid.setTolerance(0.1);

    // degrees
    thetaPid.enableContinuousInput(-180.0, 180.0);
    thetaPid.setTolerance(3);

    configurePathPlannerAutoBuilder();

    resetPose(new Pose2d(8.5, 4.0, Rotation2d.fromDegrees(0)));

    SmartDashboard.putData(this);
  }

  /** {@inheritDoc} */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.setSafeState(this::stop);
    builder.setActuator(true);
    builder.addDoubleProperty("Pose-X", () -> getPose().getX(), (xPoseMeters) -> {
      var currentPose = getPose();
      var updatedPose = new Pose2d(xPoseMeters, currentPose.getY(), currentPose.getRotation());
      resetPose(updatedPose);
    });
    builder.addDoubleProperty("Pose-Y", () -> getPose().getY(), (yPoseMeters) -> {
      var currentPose = getPose();
      var updatedPose = new Pose2d(currentPose.getX(), yPoseMeters, currentPose.getRotation());
      resetPose(updatedPose);
    });
    builder.addDoubleProperty("Pose-Theta", () -> getPose().getRotation().getDegrees(), (thetaPoseDegrees) -> {
      var currentPose = getPose();
      var updatedPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(thetaPoseDegrees));
      resetPose(updatedPose);
    });
    builder.addDoubleProperty("MaxTranslationalSpeed", () -> maxTranslationalSpeedMetersPerSecond,
        (maxSpeedMetersPerSecond) -> maxTranslationalSpeedMetersPerSecond = Math.abs(maxSpeedMetersPerSecond));
    builder.addDoubleProperty("MaxAngularSpeed", () -> Math.toDegrees(maxAngularSpeedRadiansPerSecond),
        (maxSpeedDegreesPerSecond) -> maxAngularSpeedRadiansPerSecond = Math.toRadians(maxSpeedDegreesPerSecond));
  }

  /** {@inheritDoc} */
  @Override
  public void periodic() {
    updateOdometryWithModuleStates();
  }

  /**
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html#updating-the-robot-pose
   */
  private void updateOdometryWithModuleStates() {
    kPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getAngleForOdometry(), getSwerveModulePositions());
  }

  /**
   * Adjusts the robot's current position on the field using the given
   * measurements from vision.
   * 
   * @param pose      the estimated pose of the robot from the vision
   * @param timestamp the timestamp from when the robot's position was captured
   * @param stdDevs   the standard deviations to use for the vision adjustments
   */
  public void addVisionMeasurementForOdometry(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    kPoseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  /** https://pathplanner.dev/pplib-getting-started.html#holonomic-swerve */
  private void configurePathPlannerAutoBuilder() {
    // Load the RobotConfig from the GUI settings.
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> setModuleStates(speeds), // Method that will drive the robot given ROBOT
                                                             // RELATIVE
          // ChassisSpeeds. Also optionally outputs individual module
          // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_TRANSLATION_D), // Translation PID constants
              new PIDConstants(SwerveDriveConfig.DRIVE_PATHING_ROTATION_P,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_I,
                  SwerveDriveConfig.DRIVE_PATHING_ROTATION_D) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception ex) {
      ex.printStackTrace();
      DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + ex.getMessage(), false);
    }
  }

  /**
   * Gets the robot's current position on the field.
   * 
   * @return the robot's position on the field from the odometry
   */
  public Pose2d getPose() {
    return kPoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the robot's pose for odometry.
   * 
   * <p>
   * 
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html#resetting-the-robot-pose
   * 
   * @param pose the robot's new pose
   */
  public void resetPose(Pose2d pose) {
    kPoseEstimator.resetPosition(getAngleForOdometry(), getSwerveModulePositions(), pose);
  }

  /**
   * Resets the robot heading to be 0 when blue alliance and 180 when red
   * alliance.
   * 
   * @return the command to reset the robot heading
   */
  public Command resetHeading() {
    var cmd = runOnce(() -> {
      var currentPose = getPose();
      // always reset facing RED alliance
      var awayAngle = 0;
      resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromDegrees(awayAngle)));
    });
    return cmd.withName("ResetRobotHeading");
  }

  public Command setCenterOfRotation(Translation2d rotationPoint) {
    Runnable setNewRotation = () -> {
     centerOfRotation = rotationPoint;
    };
    Runnable setCenterRotation = () -> {
      centerOfRotation = Translation2d.kZero;
    };
    var cmd = Commands.startEnd(setNewRotation, setCenterRotation);
    return cmd.withName("SwerveSetCenterRotation");
  }

  /**
   * Gets the robot's current speed.
   * 
   * @return the robot's current speed
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  /**
   * Gets the robot's current speed.
   * 
   * @param centerPoint the center point of the robot
   * @return the robot's current speed
   */
  public ChassisSpeeds getChassisSpeeds(Translation2d centerPoint) {
    return kSwerveKinematics.toChassisSpeeds(getSwerveModuleStates(), centerPoint);
  }

  /**
   * Sets the module states based on the desired robot speed.
   * 
   * @param chassisSpeeds the desired robot speed
   */
  private void setModuleStates(final ChassisSpeeds chassisSpeeds) {
    var desiredStates = getOptimizedModuleStatesFromChassisSpeeds(chassisSpeeds);
    kFrontLeft.setDesiredState(desiredStates[0], true);
    kFrontRight.setDesiredState(desiredStates[1], true);
    kRearLeft.setDesiredState(desiredStates[2], true);
    kRearRight.setDesiredState(desiredStates[3], true);
  }

  /**
   * Optimizes the swerve module states given the desired robot speeds.
   * 
   * @param originalSpeeds the speeds to optimze
   * @return the optimized swerve module states
   */
  private SwerveModuleState[] getOptimizedModuleStatesFromChassisSpeeds(final ChassisSpeeds originalSpeeds) {
    // optimize chassis speeds
    var optimizedChassisSpeeds = correctForDynamics(originalSpeeds);

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#using-custom-centers-of-rotation
    var optimizedModuleStates = kSwerveKinematics.toSwerveModuleStates(optimizedChassisSpeeds, centerOfRotation);

    // normalize wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(optimizedModuleStates, maxTranslationalSpeedMetersPerSecond);

    return optimizedModuleStates;
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(final ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose = new Pose2d(
        originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
        originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = log(futureRobotPose);
    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
        twistForPose.dx / LOOP_TIME_S,
        twistForPose.dy / LOOP_TIME_S,
        twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /**
   * Logical inverse of the above. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
   */
  private static Twist2d log(final Pose2d transform) {
    final double kEps = 1E-9;
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta = -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part = transform
        .getTranslation()
        .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Returns the angle of the robot as a Rotation2d as read by the navx.
   * <p>
   * Note that {@link #getPose()} should be used when trying to access the robot's
   * actual heading (rotation)
   * relative to the field. This method is only used internally to update the
   * odometry.
   *
   * @return The angle of the robot.
   * @see #getPose()
   */
  private Rotation2d getAngleForOdometry() {
    return RobotContainer.kNavx.getRotation2d();
  }

  /**
   * Command to set the swerve drive to go slow until interrupted.
   * 
   * @return the go slow comamnd
   */
  public Command goSlow() {
    Runnable setSlowSpeed = () -> {
      maxTranslationalSpeedMetersPerSecond = 0.5;
      maxAngularSpeedRadiansPerSecond = Math.toRadians(90);
    };
    Runnable setNormalSpeed = () -> {
      maxTranslationalSpeedMetersPerSecond = 4.6;
      maxAngularSpeedRadiansPerSecond = Math.toRadians(540);
    };
    var cmd = Commands.startEnd(setSlowSpeed, setNormalSpeed);
    return cmd.withName("SwerveGoSlow");
  }

  /**
   * Command to hold the current heading of the robot while driving.
   * 
   * @return command to hold the current heading of the robot while driving
   */
  public Command holdCurrentHeadingWhileDriving(CommandXboxController hid, BooleanSupplier fieldRelative, double joystickDeadband, double joystickSensitivity) {
    var setHeading = runOnce(() -> headingToHold = getPose().getRotation());
    var holdHeading = runEnd(() -> {
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      var xSpeed = 0.0;
      var leftYInput = -hid.getLeftY();
      if (Math.abs(leftYInput) > joystickDeadband) {
        xSpeed = HIDUtils.smoothInputWithCubic(leftYInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
      }

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      var ySpeed = 0.0;
      var leftXInput = -hid.getLeftX();
      if (Math.abs(leftXInput) > joystickDeadband) {
        ySpeed = HIDUtils.smoothInputWithCubic(leftXInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
      }

      // flip drive input based on alliance since robot's movement is always
      // relative to the blue alliance (AKA facing towards red alliance)
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var sign = fieldRelative.getAsBoolean() && alliance == Alliance.Red ? -1 : 1;
      holdAngleWhileDriving(sign * xSpeed, sign * ySpeed, headingToHold, 1, fieldRelative.getAsBoolean());
    }, this::stop);
    var cmd = Commands.sequence(setHeading, holdHeading);
    return cmd.withName("SwerveHoldCurrentHeadingWhileDriving");
  }

  /**
   * Command to drive the robot with an Xbox Controller.
   * 
   * @param hid                 the xbox controller
   * @param fieldRelative       whether or not to use field relative drive
   * @param joystickDeadband    the deadband for the joysticks to consider as
   *                            non-zero
   * @param joystickSensitivity ranges from [0, 1] where 0 is full linear and 1 is
   *                            full cubic for smoother inputs
   * @return the command to drive the robot with an Xbox Controller
   */
  public Command driveWithXboxController(CommandXboxController hid, BooleanSupplier fieldRelative,
      double joystickDeadband, double joystickSensitivity) {
    var cmd = runEnd(() -> {
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      var xSpeed = 0.0;
      var leftYInput = -hid.getLeftY();
      if (Math.abs(leftYInput) > joystickDeadband) {
        xSpeed = HIDUtils.smoothInputWithCubic(leftYInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
      }

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      var ySpeed = 0.0;
      var leftXInput = -hid.getLeftX();
      if (Math.abs(leftXInput) > joystickDeadband) {
        ySpeed = HIDUtils.smoothInputWithCubic(leftXInput, joystickSensitivity) * maxTranslationalSpeedMetersPerSecond;
      }

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      var rotationSpeed = 0.0;
      var rightXInput = -hid.getRightX();
      if (Math.abs(rightXInput) > joystickDeadband) {
        rotationSpeed = HIDUtils.smoothInputWithCubic(rightXInput, joystickSensitivity)
            * maxAngularSpeedRadiansPerSecond;
      }
      // flip drive input based on alliance since robot's movement is always
      // relative to the blue alliance (AKA facing towards red alliance)
      var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
      var sign = fieldRelative.getAsBoolean() && alliance == Alliance.Red ? -1 : 1;
      drive(sign * xSpeed, sign * ySpeed, rotationSpeed, fieldRelative.getAsBoolean());
    }, this::stop);
    return cmd.withName("DriveWithXboxController");
  }

  private Command alignToTarget(Supplier<Optional<AprilTagTarget>> targetSupp, double XDist, double YDist, boolean wantsFinalAdjustment) {
    var alignCmd = runEnd(() -> {
      targetOpt = targetSupp.get();
      if (targetOpt.isEmpty()) {
        this.stop();
        return;
      }

      var target = targetOpt.get();

      errorX = XDist - target.getOffetX();
      var outputX = -(SwerveDriveConfig.VISION_X_P * errorX);

      errorY = YDist - target.getOffsetY();
      var outputY = -(SwerveDriveConfig.VISION_Y_P * errorY);

      var targetAngle = target.getTargetAngle();

      this.holdAngleWhileDriving(outputX, outputY, Rotation2d.fromDegrees(targetAngle), false);
    }, this::stop)
    .until(() -> targetOpt.isEmpty() || (Math.abs(errorX) <= 0.01 && Math.abs(errorY) <= 0.01 && atAngleSetpoint()));
    var finalAdjustment = Commands.none();
    if (wantsFinalAdjustment){
      finalAdjustment = runEnd(() -> this.drive(0.2, 0, 0, false), this::stop)
      .withTimeout(0.5);
    }

    var cmd = Commands.sequence(alignCmd, finalAdjustment);

    return cmd.withName("SwerveDriveAlignToTarget");
  }

  /**
   * Method to drive the robot using joystick info. (used for teleop)
   *
   * @param vxMetersPerSecond     Speed of the robot in the x direction (forward)
   *                              in m/s.
   * @param vyMetersPerSecond     Speed of the robot in the y direction (sideways)
   *                              in m/s.
   * @param omegaRadiansPerSecond Angular rate of the robot in rad/sec.
   * @param fieldRelative         Whether the provided x and y speeds are relative
   *                              to the
   *                              field.
   */
  public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond,
      boolean fieldRelative) {
    if (Math.abs(omegaRadiansPerSecond) < kDriveOmegaDeadband && Math.abs(vxMetersPerSecond) < kDriveVelocityDeadband
        && Math.abs(vyMetersPerSecond) < kDriveVelocityDeadband) {
      kFrontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontLeft.getAngle())));
      kFrontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kFrontRight.getAngle())));
      kRearLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearLeft.getAngle())));
      kRearRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(kRearRight.getAngle())));
    } else {
      // get desired chassis speeds
      ChassisSpeeds desiredChassisSpeeds = fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
              getPose().getRotation())
          : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

      // set desired chassis speeds
      setModuleStates(desiredChassisSpeeds);
    }
  }

  /**
   * Gets the current drive speeds and pivot angles of all swerve modules in the
   * following order: FL, FR, RL, RR.
   * 
   * @return the states of the module (speeds and angles) in the following order:
   *         FL, FR, RL, RR.
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        kFrontLeft.getState(),
        kFrontRight.getState(),
        kRearLeft.getState(),
        kRearRight.getState()
    };
  }

  /**
   * Gets the current drive distance traveled and pivot angles of all swerve
   * modules in the following order: FL, FR, RL, RR.
   * 
   * @return the states of the module (speeds and angles) in the following order:
   *         FL, FR, RL, RR.
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        kFrontLeft.getPosition(),
        kFrontRight.getPosition(),
        kRearLeft.getPosition(),
        kRearRight.getPosition()
    };
  }

  /**
   * Command to stop all modules.
   * 
   * @return a command to stop all modules
   */
  public Command stopCommand() {
    var cmd = runOnce(this::stop);
    return cmd.withName("StopSwerve");
  }

  /**
   * Stops all modules and resets the internal PID controllers.
   */
  public void stop() {
    kFrontRight.stop();
    kFrontLeft.stop();
    kRearRight.stop();
    kRearLeft.stop();
    resetPID();
  }

  /**
   * Sets swerve modules to be all angled in an "X" pattern - useful when trying to stay still on an
   * incline or to help prevent getting pushed by defense.
   */
  public void lock() {
    kFrontRight.setDesiredState(0, -45);
    kFrontLeft.setDesiredState(0, 45);
    kRearRight.setDesiredState(0, 45);
    kRearLeft.setDesiredState(0, -45);
  }
  
  /**
   * Command for lock.
   * @see #lock()
   * @return command to lock the drive train for defense
   */
  public Command lockCmd() {
    var cmd = runEnd(this::lock, this::stop);
    return cmd.withName("SwerveDriveLock");
  }

  /**
   * Holds the desired angle while translating.
   * 
   * @param xSpeed        x speed in m/s
   * @param ySpeed        y speed in m/s
   * @param setpointAngle the desired robot angle to hold about the robot's center
   * @param maxSpeed      desired percent of max speed (0.0 to 1.0)
   * @param fieldOriented field relative speeds (true) or robot relative speeds
   *                      (false)
   */
  public void holdAngleWhileDriving(double xSpeed, double ySpeed, Rotation2d setpointAngle, double maxSpeed, boolean fieldOriented) {
    var rotateOutput = MathUtil
        .clamp(thetaPid.calculate(getPose().getRotation().getDegrees(), normalizeAngle(setpointAngle.getDegrees())), -maxSpeed,
          maxSpeed)
        * maxAngularSpeedRadiansPerSecond;
    drive(xSpeed, ySpeed, rotateOutput, fieldOriented);
  }
  
  public void holdAngleWhileDriving(double xSpeed, double ySpeed, Rotation2d setpointAngle, boolean fieldOriented) {
    holdAngleWhileDriving(xSpeed, ySpeed, setpointAngle, 0.5, fieldOriented);
  }

  /**
   * Rotate to a desired angle about the robot's center.
   */
  public void rotateToAngleInPlace(Rotation2d setpointAngle, double maxSpeed) {
    holdAngleWhileDriving(0, 0, setpointAngle, maxSpeed, false);
  }

  public void rotateToAngleInPlace(Rotation2d setpointAngle) {
    holdAngleWhileDriving(0, 0, setpointAngle, false);
  }

  /**
   * Rotate to a desired angle about the robot's center.
   * 
   * @param setpointAngle
   */
  public Command rotateToAngleInPlaceCmd(Rotation2d setpointAngle) {
    var cmd = runEnd(() -> rotateToAngleInPlace(setpointAngle), this::stop)
      .until(() -> this.atAngleSetpoint());
    return cmd.withName("SwerveRotateToAngleInPlace");
  }

  /**
   * Sets the drive to go to the desired pose given the robot's current pose vs
   * target
   * 
   * @param target        the desired pose the robot should go to
   * @param fieldOriented field relative speeds (true) or robot relative speeds
   *                      (false)
   */
  public void goToPose(Pose2d target, boolean fieldOriented) {
    Pose2d pose = getPose();
    double xSpeed = MathUtil.clamp(xPid.calculate(pose.getX(), target.getX()), -1, 1)
        * maxTranslationalSpeedMetersPerSecond;
    double ySpeed = MathUtil.clamp(yPid.calculate(pose.getY(), target.getY()), -1, 1)
        * maxTranslationalSpeedMetersPerSecond;
    double vTheta = MathUtil.clamp(thetaPid.calculate(normalizeAngle(pose.getRotation().getDegrees()),
        normalizeAngle(target.getRotation().getDegrees())), -1, 1) * maxAngularSpeedRadiansPerSecond;
    drive(xSpeed, ySpeed, vTheta, fieldOriented);
  }

  /**
   * Gets if the robot's heading is at the desired angle.
   * 
   * @return true if the robot's heading is at the desired angle, false otherwise
   */
  public boolean atAngleSetpoint() {
    return thetaPid.atSetpoint();
  }

  /**
   * Resets the internal X, Y and Theta PID controllers.
   */
  public void resetPID() {
    xPid.reset();
    yPid.reset();
    thetaPid.reset();
  }

  /**
   * Clamps an angle to be from [-180, 180]
   * 
   * @param angle the angle to clamp
   * @return the newly clamped angle from [-180, 180]
   */
  private static double normalizeAngle(double angle) {
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
}
