package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveDrive;

public class Turret extends SubsystemBase{
    private final Hood hood = new Hood();
    private final Waist waist = new Waist();
    private final Kicker kicker = new Kicker();
    private final Spitter spitter = new Spitter();

    public Turret() {
        SmartDashboard.putData(this);
    }

    public void periodic() {
        RobotContainer.kField.getObject("waistAiming").setPose(
            RobotContainer.kSwerveDrive.getPose().getX(),
            RobotContainer.kSwerveDrive.getPose().getY(),
            getTestAimWaistCalcs(getCurrentHubPosition())
            );
    }

    public Command stopCommand() {
    var cmd = runOnce(this::stop);
    cmd.addRequirements(hood, waist, spitter, kicker);
    return cmd.withName("StopTurret");
  }

    public void stop() {
        hood.stop();
        waist.stop();
        spitter.stop();
        kicker.stop();
    }

    public Command shootMomentarily() {
        var cmd = startEnd(spitter::start, spitter::stop);
        cmd.addRequirements(spitter);
        return cmd;
    }

    public Command reload() {
        var cmd = startEnd(kicker::start, kicker::stop);
        cmd.addRequirements(kicker);
        return cmd;
    }

    public Command shootContinuously() {
        var cmd = Commands.parallel(
                shootMomentarily(),
                reload()
            );
        cmd.addRequirements(this);
        return cmd;
    }

    private Translation2d getCurrentHubPosition() {
        Translation2d hubPosition;
        var alliance = DriverStation.getAlliance();
        if ( alliance.get() == Alliance.Red) {
            hubPosition = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField;
        //if the alliance is null, it is auto set to blue variables
        }else {
            hubPosition = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField;
        }
        return hubPosition;
    }

    private double getAngleFromRobotToPosition(Translation2d toPose) {
        return getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), toPose);
    }

    private double getAngleToHub(Translation2d fromPose) {
        return getAngleToPosition(fromPose, getCurrentHubPosition());
    }

    private double getAngleToPosition() {
        return getAngleToPosition(RobotContainer.kSwerveDrive.getPose().getTranslation(), getCurrentHubPosition());
    }

    private double getAngleToPosition(Translation2d fromPose, Translation2d toPose) {
        double yDistanceTo = toPose.getY() - fromPose.getY();
        double xDistanceTo = toPose.getX() - fromPose.getX();
        
        var angleToHub =  Math.atan2(yDistanceTo, xDistanceTo);
        return angleToHub;
    }


    private double getDistanceToHub(Translation2d pose) {
        double distanceToHub = pose.getDistance(getCurrentHubPosition());
        return distanceToHub;
    }

    private double getDistanceToHub() {
        return getDistanceToHub(RobotContainer.kSwerveDrive.getPose().getTranslation());
    }



    private double solvePitch(double distance) {
        //uses newtons method to itterate until it finds a zero (or something close enough) of the original physics function
        //the original function shows where the angle needs to be to hit a set point, in this case the hub
        //the function has two answers (where x = 0) that give us an angle that hits the hub, we take the higher one by calculating using a higher guess for the angle
        double theta = Math.toRadians(88); 
        double g = TurretConfig.TurretFieldAndRobotInfo.kGravity;
        double d = distance;
        double v = TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        double h = TurretConfig.TurretFieldAndRobotInfo.kHeightBetweenShooterAndHub;
        for (int i = 0; i < 20; i++) {
            double func = d * Math.tan(theta) - (g * d * d ) / (2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) - h;
            if (Math.abs(func) < 0.001) {
                return theta;
            }
            double derivative = (d * (1 / Math.cos(theta)) * (1 / Math.cos(theta)) - (  4 * g * d * d * (v * Math.cos(theta)) * v * Math.sin(theta)) / ((2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) * (2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))) );
            double step = func / derivative;
            theta = theta - step;
        }
        return theta;
    }

    private Translation2d getRadialUnitVector() {
        //radial velocity is in reference to the velocity of the robot towards the hub
        // all velocity is in meters per second
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        Translation2d radialVector;
        double rX;
        double rY;
        if ( alliance.get() == Alliance.Red) {
            rX = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY - robotPosition.getY();
        //if the alliance is null, it is auto set to blue variables
        }else {
            rX = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionX - robotPosition.getX();
            rY = TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY - robotPosition.getY();
        }
        radialVector = new Translation2d(rX, rY);
        //distance to hub
        var radialVectorNorm = radialVector.getNorm();
        Translation2d radialUnitVector;
        if (radialVectorNorm != 0) {
            radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm, radialVector.getY() / radialVectorNorm);
        }else {
            radialUnitVector = new Translation2d(0,0);
        }
        return radialUnitVector;
    }

        //var velocityX = RobotContainer.kNavx.getVelocityX();
        //var velocityY = RobotContainer.kNavx.getVelocityY();
        //use these variable when getting the robots radial

        //use different things for the turret radial
    private double getRadialVelocity(double velocityX, double velocityY) {
        Translation2d radialUnitVector = getRadialUnitVector();
        double radialVelocity = (velocityX * radialUnitVector.getX() + velocityY * radialUnitVector.getY());
        return radialVelocity;
    }

    //private double getRadialVelocityForPhantom(double velocityX, double velocityY, Translation2d position) {
    //getting the radial velocity from a phantom position we got from tangential velocity calcs to account for different radial distances
    //}

    private double getTangentialVelocity(double velocityX, double velocityY) {
        //tangential velocity is in reference to velocity of the robot around the hub in le circle
        // we just use the radial velocity code and then rotate the vector
        Translation2d radialUnitVector = getRadialUnitVector();
        Translation2d tangentialUnitVector = radialUnitVector.rotateBy(Rotation2d.kCCW_90deg);
        double tagVelocity = (velocityX * tangentialUnitVector.getX() + velocityY * tangentialUnitVector.getY());
        return tagVelocity;
    }
    
    private Translation2d getPhantomRobotPosition() {
        //just using tangential velocity calcs
        double tangVelocity = getTangentialVelocity(RobotContainer.kNavx.getVelocityX(), RobotContainer.kNavx.getVelocityY());
        Rotation2d angleOfTang = new Rotation2d(getAngleToPosition() + (Math.PI / 2));
        double T = getDistanceToHub() / TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        double phantomYawDistance = T * tangVelocity;// + robotRotationalTangVelocity;
        Translation2d phantomYaw = new Translation2d(phantomYawDistance, angleOfTang);
        phantomYaw = phantomYaw.plus(RobotContainer.kSwerveDrive.getPose().getTranslation());

        return phantomYaw;
    }

    public Command aimWaist(Translation2d towards) {
        Translation2d phantomRobotPosition = getPhantomRobotPosition();
        //the adding of the yaw is to account for the rotation of the robot
        double angle =  getAngleToPosition(phantomRobotPosition,towards) + RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
        //clamping the value because our turret only goes 240(?) degrees
        angle = MathUtil.clamp(Math.toDegrees(angle) % 360, 0, 360) - 180;
        var cmd = waist.setDegreesCommand(angle + TurretConfig.WaistConfig.kWaistDegreesOffset);
        return cmd;
    }

    public Rotation2d getTestAimWaistCalcs(Translation2d towards) {
        Translation2d phantomRobotPosition = getPhantomRobotPosition();

        //the adding of the yaw is to account for the rotation of the robot
        double angle =  getAngleToPosition(phantomRobotPosition,towards) + RobotContainer.kSwerveDrive.getPose().getRotation().getRadians();
        //clamping the value because our turret only goes 240(?) degrees
        angle = MathUtil.clamp(Math.toDegrees(angle) % 360, 0, 360) - 180;
        Rotation2d angleRotation = new Rotation2d(angle + TurretConfig.WaistConfig.kWaistDegreesOffset);
        return angleRotation;
    }

    public Command aimWaist() {
        return aimWaist(getCurrentHubPosition());
    }

    public Command aimHood(Translation2d towards) {
        double radialVelocity = getRadialVelocity(RobotContainer.kNavx.getVelocityX(), RobotContainer.kNavx.getVelocityY());
        Rotation2d angleOfRad = new Rotation2d(getAngleToPosition());
        double T = getDistanceToHub() / TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        double phantomPitchDistance = T * radialVelocity;// + robotRotationalRadVelocity;
        Translation2d phantomPitch = new Translation2d(phantomPitchDistance, angleOfRad);
        phantomPitch = phantomPitch.plus(getPhantomRobotPosition());

        double distanceToPoint = phantomPitch.getDistance(towards);
        var angle = 90 - Math.toDegrees(solvePitch(distanceToPoint));
        //add radial velocity calcs
        //clamp to the physical limits of our hood
        angle = MathUtil.clamp(angle, 0.0, 45.0);
        var cmd = hood.setAngleCommand(angle + TurretConfig.HoodConfig.kHoodDegreesOffset);
        return cmd;
    }

    public double getTestAimHoodCalcs(Translation2d towards) {
        double distanceToPoint = RobotContainer.kSwerveDrive.getPose().getTranslation().getDistance(towards);
        var angle = 90 - Math.toDegrees(solvePitch(distanceToPoint));
        //add radial velocity calcs
        //clamp to the physical limits of our hood
        angle = MathUtil.clamp(angle, 0.0, 45.0);
        angle += TurretConfig.HoodConfig.kHoodDegreesOffset;
        return angle;
    }

    public Command aimHood() {
        return aimHood(getCurrentHubPosition());
    }

    public Command aimTowardsHub() {
        var cmd = Commands.parallel(
            aimHood(), 
            aimWaist()
            );
        cmd.addRequirements(hood, waist);
        return cmd;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.setSafeState(this::stop);
        builder.addDoubleProperty("testHoodCalcs",() -> getTestAimHoodCalcs(getCurrentHubPosition()), null);
    }
}
    //for getting the position and rotation (which we will probably want to do in hood and waist), use our humble kNavx
    //
    //    var navXRotation = RobotContainer.kNavx.getYaw();
    //    var navXVelocityX = RobotContainer.kNavx.getVelocityX();
    //    var navXVelocityY = RobotContainer.kNavx.getVelocityY();
    //    var swerveDrivePosition = RobotContainer.kSwerveDrive.getPose();
    // T = distanceToHub / v;
    // phantomPitchPosition = T * getRadialVelocity();