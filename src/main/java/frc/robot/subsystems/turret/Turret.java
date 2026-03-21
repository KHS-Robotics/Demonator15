package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
    private final Belt belt = new Belt();
    private final Hood hood = new Hood();
    private final Waist waist = new Waist();
    private final Kicker kicker = new Kicker();
    private final Spitter spitter = new Spitter();
    private Supplier<Boolean> hoodCanMakeShot = () -> false;
    private Supplier<Boolean> waistCanMakeShot = () -> false;

    private double overrideSetpointDegrees;
    private boolean useOverride;

    public Turret() {
        SmartDashboard.putData(this);

        
        waist.setDefaultCommand(waist.setDegreesCommand(getDesiredWaistAngle(currentShootingTarget(), false)));
        hood.setDefaultCommand(hood.setAngleCommand(getDesiredHoodAngle(currentShootingTarget(), false)));
        spitter.setDefaultCommand(spitter.startCommand());
        kicker.setDefaultCommand(kicker.startCommand());
    }

    public void stop() {
        hood.stop();
        waist.stop();
        spitter.stop();
        kicker.stop();
        belt.stop();
    }

    public Command stopCommand() {
        var cmd = runOnce(this::stop);
        cmd.addRequirements(hood, waist, kicker, spitter, belt);
        return cmd.withName("StopTurret");
    }

    public Command shoot() {
        var cmd = spitter.startEnd(spitter::start, spitter::stop);
        cmd.addRequirements(spitter);
        return cmd.withName("ShootFuel");
    }

    public Command kick() {
        var cmd = kicker.startEnd(kicker::start, kicker::stop);
        cmd.addRequirements(kicker);
        return cmd.withName("KickFuel");
    }

    public Command feed() {
        var cmd = belt.runEnd(belt::start, belt::stop);
        cmd.addRequirements(belt);
        return cmd.withName("FeedFuel");
    }

    public Command reload() {
        var startKicker = kicker.startEnd(kicker::start, kicker::stop);
        var startBelt = belt.startEnd(belt::start, belt::stop);
        var cmd = startKicker.alongWith(startBelt);
        cmd.addRequirements(kicker, belt);
        return cmd.withName("ReloadFuel");
    }

    public Command shootContinuously() {
        var startSpitter = spitter.startEnd(spitter::start, spitter::stop);
        var startKicker = kicker.startEnd(kicker::start, kicker::stop);
        var startBelt = belt.startEnd(belt::start, belt::stop);
        var cmd = startSpitter.alongWith(startKicker).alongWith(startBelt);
        cmd.addRequirements(spitter, kicker, belt);
        return cmd.withName("ShootFuel");
    }

    private Supplier<Translation2d> currentHubTranslation() {
        return () -> {
            Translation2d hubPosition;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                hubPosition = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField;
                // if the alliance is null, it is auto set to blue variables
            } else {
                hubPosition = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField;
            }
            return hubPosition;
        };
    }

    private Supplier<Translation2d> currentPassingTranslation() {
        return () -> {
            Translation2d passingTranslation;
            var alliance = DriverStation.getAlliance();
            if (RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getY() > TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY &&
                    (alliance.isPresent() && alliance.get() == Alliance.Red)) {
                passingTranslation = TurretConfig.TurretFieldAndRobotInfo.kPassingPositionRedLeft;

            } else if (RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getY() > TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY
                    && (alliance.isPresent() && alliance.get() == Alliance.Blue)) {
                passingTranslation = TurretConfig.TurretFieldAndRobotInfo.kPassingPositionBlueLeft;

            } else if (RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getY() <= TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY &&
                    (alliance.isPresent() && alliance.get() == Alliance.Red)) {
                passingTranslation = TurretConfig.TurretFieldAndRobotInfo.kPassingPositionRedRight;

            } else {
                passingTranslation = TurretConfig.TurretFieldAndRobotInfo.kPassingPositionBlueRight;
            }
            return passingTranslation;
        };
    }

    private Supplier<Translation2d> currentShootingTarget() {
        return () -> {
            var alliance = DriverStation.getAlliance();
            Translation2d target;
            if ((alliance.isPresent() && alliance.get() == Alliance.Red)
                    && RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getX() <= TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX) {
                target = currentPassingTranslation().get();

            }else if ((alliance.isPresent() && alliance.get() == Alliance.Red)
                    && RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getX() > TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX) {
                target = currentHubTranslation().get();

            }else if ((alliance.isPresent() && alliance.get() == Alliance.Blue)
                    && RobotContainer.kSwerveDrive.getPose().getTranslation()
                    .getX() >= TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX) {
                target = currentPassingTranslation().get();

            }else{
                target = currentHubTranslation().get();
            }
            return target;
        };
    }

    private double getAngleToPosition() {
        return getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(),
                currentHubTranslation().get());
    }

    private double getAngleToPosition(Translation2d fromPose, Translation2d toPose) {
        double yDistanceTo = toPose.getY() - fromPose.getY();
        double xDistanceTo = toPose.getX() - fromPose.getX();

        var angleToHub = Math.atan2(yDistanceTo, xDistanceTo);
        return angleToHub;
    }

    private double getDistanceToHub(Translation2d pose) {
        double distanceToHub = pose.getDistance(currentHubTranslation().get());
        return distanceToHub;
    }

    private double getDistanceToHub() {
        return getDistanceToHub(RobotContainer.kSwerveDrive.getPose().getTranslation());
    }

    /**
     * @return an angle in radians
     */
    private double solvePitch(double distance) {
    // uses newtons method to itterate until it finds a zero (or something close
    // enough) of the original physics function
    // the original function shows where the angle needs to be to hit a set point,
    // in this case the hub
    // the function has two answers (where x = 0) that give us an angle that hits
    // the hub, we take the higher one by calculating using a higher guess for the
    // angle
    double theta = 1.5;
    double g = TurretConfig.TurretFieldAndRobotInfo.kGravity;
    double d = distance;
    double v = TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
    double h = TurretConfig.TurretFieldAndRobotInfo.kHeightBetweenShooterAndHub;
    for (int i = 0; i < 25; i++) {
       double func = d * Math.tan(theta) - (g * d * d) / (2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) - h;
      if (Math.abs(func) < 0.001) {
        return theta;
      }
      double derivative = (d * (1 / Math.cos(theta)) * (1 / Math.cos(theta))
          - (4 * g * d * d * (v * Math.cos(theta)) * v * Math.sin(theta))
              / ((2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))
                  * (2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))));
      if (Math.abs(derivative) < 0.5) {
        return theta;
      }
      double step = func / derivative;
      theta = theta - step;
    }
    return theta;
  }

    // var velocityX = RobotContainer.kNavx.getVelocityX();
    // var velocityY = RobotContainer.kNavx.getVelocityY();
    // use these variable when getting the robots radial

    // use different things for the turret radial
    private double getRadialVelocityForPhantom(double velocityX, double velocityY, Translation2d position) {
        // getting the radial velocity from a phantom position we got from tangential
        // velocity calcs to account for different radial distances
        // radial velocity is in reference to the velocity of the robot towards the hub
        // all velocity is in meters per second
        Translation2d robotPosition = position;

        var alliance = DriverStation.getAlliance();
        Translation2d radialVector;
        double rX;
        double rY;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            rX = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY - robotPosition.getY();
            // if the alliance is null, it is auto set to blue variables
        } else {
            rX = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY - robotPosition.getY();
        }
        radialVector = new Translation2d(rX, rY);
        // distance to hub
        var radialVectorNorm = radialVector.getNorm();
        Translation2d radialUnitVector;
        if (radialVectorNorm != 0) {
            radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm,
                    radialVector.getY() / radialVectorNorm);
        } else {
            radialUnitVector = new Translation2d(0, 0);
        }
        double radVelocity = (velocityX * radialUnitVector.getX() + velocityY * radialUnitVector.getY());
        return radVelocity;
    }

    private double getTangentialVelocity(double velocityX, double velocityY) {
        // tangential velocity is in reference to velocity of the robot around the hub
        // in le circle
        // we just use the radial velocity code and then rotate the vector
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        Translation2d radialVector;
        double rX;
        double rY;
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            rX = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY - robotPosition.getY();
            // if the alliance is null, it is auto set to blue variables
        } else {
            rX = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY - robotPosition.getY();
        }
        radialVector = new Translation2d(rX, rY);
        // distance to hub
        var radialVectorNorm = radialVector.getNorm();
        Translation2d radialUnitVector;
        if (radialVectorNorm != 0) {
            radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm,
                    radialVector.getY() / radialVectorNorm);
        } else {
            radialUnitVector = new Translation2d(0, 0);
        }
        Translation2d tangentialUnitVector = radialUnitVector.rotateBy(Rotation2d.kCCW_90deg);
        double tagVelocity = (velocityX * tangentialUnitVector.getX() + velocityY * tangentialUnitVector.getY());
        return tagVelocity;
    }

    /**
     * @return the linear velocity of the turret based on the rotation of the robot
     */
    private Translation2d getTurretRotationalToLinearVelocity() {
        Rotation2d theta = new Rotation2d(
                Math.toRadians(RobotContainer.kSwerveDrive.getPose().getRotation().getDegrees() + 90
                        + TurretConfig.TurretFieldAndRobotInfo.kDegreeOffsetOfTurret));
        var magnitude = new Rotation2d(RobotContainer.kSwerveDrive.getChassisSpeeds().omegaRadiansPerSecond)
                .getDegrees() * TurretConfig.TurretFieldAndRobotInfo.kTurretDistanceToNavX;
        Translation2d RotationalVelocity = new Translation2d(magnitude, theta);
        return RotationalVelocity;
    }

    /**
     * @return the fake robot position for position calcs based on tangential
     *         velocity
     */
    private Translation2d getPhantomRobotPosition() {
        // just using tangential velocity calcs
        double tangVelocity = getTangentialVelocity(RobotContainer.kSwerveDrive.getChassisSpeeds().vxMetersPerSecond,
                RobotContainer.kSwerveDrive.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d angleOfTang = new Rotation2d(getAngleToPosition() + (Math.PI / 2));
        var pitchAngle = solvePitch(getDistanceToHub());
        double T = getDistanceToHub() / (TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity * Math.cos(pitchAngle));
        var rotationalVelocity = getTurretRotationalToLinearVelocity();
        double robotRotationalTangVelocity = getTangentialVelocity(rotationalVelocity.getX(),
                rotationalVelocity.getY());
        double phantomYawDistance = T * (tangVelocity + robotRotationalTangVelocity);
        Translation2d phantomYawVector = new Translation2d(phantomYawDistance, angleOfTang);
        // translation 2ds just happen to be a canvinient way to use a two variable
        // variable, some translation2ds actually represent velocities
        Translation2d phantomYawPosition = phantomYawVector
                .plus(RobotContainer.kSwerveDrive.getPose().getTranslation());
        if (RobotContainer.kSwerveDrive.getChassisSpeeds().vxMetersPerSecond == 0
                && RobotContainer.kSwerveDrive.getChassisSpeeds().vyMetersPerSecond == 0) {
            phantomYawPosition = new Translation2d(RobotContainer.kSwerveDrive.getPose().getX(),
                    RobotContainer.kSwerveDrive.getPose().getY());
        }

        return phantomYawPosition;
    }

    /**
     * 
     * @return the angle that the waist should be to hit the position towards based
     *         on the current position and velocity of the robot
     */
    private Supplier<Double> getDesiredWaistAngle(Supplier<Translation2d> towards, boolean useVelocity) {
        return () -> {
            if (useOverride){
                var angle = overrideSetpointDegrees;
                return angle;
            }
            else{
            var angle = getWaistAimFinalRotation(towards, useVelocity);
            return angle;
            }
        };
    }

    public Command setOverride(boolean override, double setpoint){
        var cmd = Commands.runOnce(() -> {
            setOverrideDirect(override, setpoint);
        });
        return cmd.withName("SetOverride");
    }

    public void setOverrideDirect(boolean override, double setpoint){
        this.overrideSetpointDegrees = setpoint;
        this.useOverride = override;
    }

    public void calibrateRelativeEncoders() {
        waist.calibrateRelativeEncoder();
        hood.calibrateRelativeEncoder();
    }

    /**
     * 
     * @return the angle that the waist should be to score in the hub based on the
     *         current position and velocity of the robot
     */
    private Supplier<Double> getDesiredWaistAngle(boolean useVelocity) {
        return getDesiredWaistAngle(currentHubTranslation(), useVelocity);
    }

    private double getWaistAimFinalRotation(Supplier<Translation2d> towards, boolean useVelocity) {
        Translation2d phantomRobotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        if (useVelocity) {
            phantomRobotPosition = getPhantomRobotPosition();
        }
        // the adding of the yaw is to account for the rotation of the robot
        double angle = getAngleToPosition(phantomRobotPosition, towards.get())
                - RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
        // clamping the value because our turret only goes 240(?) degrees
        angle = Units.radiansToDegrees(MathUtil.angleModulus(angle + TurretConfig.WaistConfig.kWaistRadiansOffset));
        if (angle > TurretConfig.WaistConfig.kMaxSoftLimit || angle < TurretConfig.WaistConfig.kMinSoftLimit) {
                waistCanMakeShot = () -> false;
        }else {waistCanMakeShot = () -> true;}
        angle = MathUtil.clamp(angle, TurretConfig.WaistConfig.kMinSoftLimit, TurretConfig.WaistConfig.kMaxSoftLimit);
        // + TurretConfig.WaistConfig.kWaistDegreesOffset);
        return angle;
    }

    private Supplier<Double> getHoodAimFinalAngle(Supplier<Translation2d> towards, boolean useVelocity) {
        return () -> {
            Translation2d phantomPitchPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
            if (useVelocity == true) {
                // linear calcs
                Translation2d phantomRobotPosition = getPhantomRobotPosition();
                double radialVelocity = getRadialVelocityForPhantom(
                        RobotContainer.kSwerveDrive.getChassisSpeeds().vxMetersPerSecond,
                        RobotContainer.kSwerveDrive.getChassisSpeeds().vyMetersPerSecond, phantomRobotPosition);
                Rotation2d angleOfRad = new Rotation2d(getAngleToPosition(phantomRobotPosition, towards.get()));
                double T = getDistanceToHub(phantomRobotPosition) / (TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity
                        * Math.cos(Math.toRadians(90 - Math.toDegrees(solvePitch(phantomRobotPosition.getDistance(towards.get()))))));
                // rotational calcs
                var rotationalVelocity = getTurretRotationalToLinearVelocity();
                double robotRotationalRadVelocity = getTangentialVelocity(rotationalVelocity.getX(),
                        rotationalVelocity.getY());

                double phantomPitchDistance = T * (radialVelocity + robotRotationalRadVelocity);
                Translation2d phantomPitchVector = new Translation2d(phantomPitchDistance, angleOfRad);
                phantomPitchPosition = phantomPitchVector.plus(phantomRobotPosition);
            }

            double distanceToPoint = phantomPitchPosition.getDistance(towards.get());
            double pitchAngle = Math.toDegrees(solvePitch(distanceToPoint));
            var angle = 90 - pitchAngle + TurretConfig.TurretFieldAndRobotInfo.kHoodFudgeFactor;
            if ((pitchAngle > 90) || (distanceToPoint > TurretConfig.TurretFieldAndRobotInfo.kMaxDistance)) {
                angle = TurretConfig.HoodConfig.kMaxSoftLimit + 1;
            }
            // add radial velocity calcs
            // clamp to the physical limits of our hood
            if (angle > TurretConfig.HoodConfig.kMaxSoftLimit || angle < TurretConfig.HoodConfig.kMinSoftLimit) {
                hoodCanMakeShot = () -> false;
            }else {hoodCanMakeShot = () -> true;}
            angle = MathUtil.clamp(angle, TurretConfig.HoodConfig.kMinSoftLimit, TurretConfig.HoodConfig.kMaxSoftLimit);
            // this will be part of the relative / absolute hybrid incorporation +
            // TurretConfig.HoodConfig.kHoodDegreesOffset;
            return angle;
        };
    }

    /**
     * 
     * @return the angle that the hood should be to hit the position towards based
     *         on the current position and velocity of the robot
     */
    private Supplier<Double> getDesiredHoodAngle(Supplier<Translation2d> towards, boolean useVelocity) {
        return () -> {
            double angle = getHoodAimFinalAngle(towards, useVelocity).get();
            return angle;
        };
    }

    /**
     * 
     * @return the angle that the hood should be to score in the hub based on the
     *         current position and velocity of the robot
     */
    private Supplier<Double> getDesiredHoodAngle(boolean useVelocity) {
        return getDesiredHoodAngle(currentHubTranslation(), useVelocity);
    }

    public Command aimTowardsHubWithVelocity() {
        var aimHood = hood.runEnd(() -> hood.setSetpointAngle(getDesiredHoodAngle(true).get()), hood::stop);
        var aimWaist = waist.runEnd(() -> waist.setSetpointDegrees(getDesiredWaistAngle(true).get()), waist::stop);
        var cmd = aimWaist.alongWith(aimHood);
        cmd.addRequirements(hood, waist);
        return cmd.withName("TurretAimTowardsHubWithVelocity");
    }

    public Command aimTowardsHub() {
        var aimHood = hood.runEnd(() -> hood.setSetpointAngle(getDesiredHoodAngle(false).get()), hood::stop);
        var aimWaist = waist.runEnd(() -> waist.setSetpointDegrees(getDesiredWaistAngle(false).get()), waist::stop);
        var cmd = aimWaist.alongWith(aimHood);
        cmd.addRequirements(hood, waist);
        return cmd.withName("TurretAimTowardsHub");
    }

    // not sure if this will work because of command shenanagins
    public Command aimAndShootTowardsHub() {
        var aim = aimTowardsHub();
        var shoot = shootContinuously();
        var cmd = aim.alongWith(shoot);
        cmd.addRequirements(this);
        return cmd.withName("Shoot At Hub");
    }

    // not sure if this will work because of command shenanagins
    public Command aimAndShootTowardsHubWithVelocity() {
        var aim = aimTowardsHubWithVelocity();
        var shoot = shootContinuously();
        var cmd = aim.alongWith(shoot);
        cmd.addRequirements(this);
        return cmd.withName("Shoot At Hub With Velocity");
    }

    public Command aimForPassing() {
        var aimHood = hood.runEnd(
                () -> hood.setSetpointAngle(getDesiredHoodAngle(currentPassingTranslation(), false).get()), hood::stop);
        var aimWaist = waist.runEnd(
                () -> waist.setSetpointDegrees(getDesiredWaistAngle(currentPassingTranslation(), false).get()),
                waist::stop);
        var cmd = aimWaist.alongWith(aimHood);
        cmd.addRequirements(this);
        return cmd.withName("Aim For Passing");
    }

    // not sure if this will work because of command shenanagins
    public Command aimAndShootForPassing() {
        var aim = aimForPassing();
        var shoot = shootContinuously();
        var cmd = aim.alongWith(shoot);
        cmd.addRequirements(this);
        return cmd.withName("pass");
    }

    public Command goToSetHoodAngle() {
        var cmd = hood.setAngleCommand(() -> 0.0);
        cmd.addRequirements(hood);
        return cmd.withName("GoToHoodAngle");
    }

    public Command goToSetHoodAngle2() {
        var cmd = hood.setAngleCommand(() -> 25.0);
        cmd.addRequirements(hood);
        return cmd.withName("GoToHoodAngle");
    }

    public Command goToSetWaistAngle() {
        var cmd = waist.setDegreesCommand(() -> 0.0);
        cmd.addRequirements(waist);
        return cmd.withName("GoToWaistAngle");
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.addBooleanProperty("Is Turret At Setpoint?", () -> hood.isAtSetpoint() && waist.isAtSetpoint(), null);
        builder.addBooleanProperty("Hood Can Hit", () -> hoodCanMakeShot.get(), null);
        builder.addBooleanProperty("Waist Can Hit", () -> waistCanMakeShot.get(), null);
        builder.addDoubleProperty("hood insurace angle", () -> getDesiredHoodAngle(currentShootingTarget(), false).get(), null);
        builder.addDoubleProperty("waist insurace angle", () -> getDesiredWaistAngle(currentShootingTarget(), false).get(), null);
        builder.addDoubleProperty("current shooting target x",() -> currentShootingTarget().get().getX(), null);
        builder.addDoubleProperty("current shooting target y",() -> currentShootingTarget().get().getY(), null);
        // hood.aimHoodSimpleAngle(getCurrentHubPosition()), null);
    }
}
// for getting the position and rotation (which we will probably want to do in
// hood and waist), use our humble kNavx
//
// var navXRotation = RobotContainer.kNavx.getYaw();
// var navXVelocityX = RobotContainer.kNavx.getVelocityX();
// var navXVelocityY = RobotContainer.kNavx.getVelocityY();
// var swerveDrivePosition = RobotContainer.kSwerveDrive.getPose();
// T = distanceToHub / v;
// phantomPitchPosition = T * getRadialVelocity();