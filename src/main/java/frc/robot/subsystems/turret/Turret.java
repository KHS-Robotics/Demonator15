package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Turret extends SubsystemBase{
    private final Hood hood = new Hood();
    private final Waist waist = new Waist();
    private final Kicker kicker = new Kicker();
    private final Spitter spitter = new Spitter();

    public Turret() {
        SmartDashboard.putData(this);
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

    private double getAngleToHub() {
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        double yDistanceToHub;
        if ( alliance.get() == Alliance.Red) {
            yDistanceToHub = robotPosition.getY() - TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY;
        //if the alliance is null, it is auto set to blue variables
        }else {
            yDistanceToHub = robotPosition.getY() - TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY;
        }
        var angleToHub =  Math.asin(yDistanceToHub / getDistanceToHub());
        return angleToHub;
    }

    private double getDistanceToHub(Translation2d pose) {
        Translation2d robotPosition = pose;
        var alliance = DriverStation.getAlliance();
        double distanceToHub;
        if (alliance.get() == Alliance.Red) {
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField);
        //if the alliance is null, it is auto set to blue variables
        }else {
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField);
        }
        return distanceToHub;
    }

    private double getDistanceToHub() {
        return getDistanceToHub(RobotContainer.kSwerveDrive.getPose().getTranslation());
    }


    private double solvePitch(double distance) {
        //uses newtons method to itterate until it finds a zero (or something close enough) of the original function
        //the original function shows where the angle needs to be to hit a set point, in this case the hub
        //the function has two answers (where x = 0) that give us an angle that hits the hub, we take the higher one by calculating using a higher guess for the angle
        double theta = Math.toRadians(88); 
        double g = TurretConfig.TurretFieldAndRobotInfo.kGravity;
        double d = distance;
        double v = TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        double h = TurretConfig.TurretFieldAndRobotInfo.kHeightBetweenShooterAndHub;
        for (int i = 0; i < 100; i++) {
            double func = d * Math.tan(theta) - (g * d * d ) / (2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) - h;
            System.out.println(theta);
            if (Math.abs(func) < 0.001) {
                return theta;
            }
            double derivative = (d * (1 / Math.cos(theta)) * (1 / Math.cos(theta)) - (  4 * g * d * d * (v * Math.cos(theta)) * v * Math.sin(theta)) / ((2 * (v * Math.cos(theta)) * (v * Math.cos(theta))) * (2 * (v * Math.cos(theta)) * (v * Math.cos(theta)))) );
            double step = func / derivative;
            theta = theta - step;
        }
        return theta;
    }

        //var velocityX = RobotContainer.kNavx.getVelocityX();
        //var velocityY = RobotContainer.kNavx.getVelocityY();
        //use these variable when getting the robots radial

        //use different things for the turret radial
    private double getRadialVelocity(double velocityX, double velocityY) {
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
        Translation2d radialUnitVector = new Translation2d(radialVector.getX() / radialVectorNorm, radialVector.getY() / radialVectorNorm);
        double radialVelocity = (velocityX * radialUnitVector.getX() + velocityY * radialUnitVector.getY());
        return radialVelocity;
    }

    private double getTangentialVelocity(double velocityX, double velocityY) {
        //tangential velocity is in reference to the velocity of the robot in a circle around the hub
        Translation2d targetsVelocity = new Translation2d(velocityX, velocityY);
        var angleToHub =  getAngleToHub();
        double radialVelocity = getRadialVelocity(velocityX, velocityY);

        Translation2d radialXYVelocity = new Translation2d( radialVelocity * Math.sin(angleToHub) ,radialVelocity * Math.cos(angleToHub) );
        Translation2d tagXYVelocity = new Translation2d(targetsVelocity.getX() - radialXYVelocity.getX(), targetsVelocity.getY() - radialXYVelocity.getY());
        double tagVelocity = tagXYVelocity.getNorm();

        return tagVelocity;
    }

    public Command aimWaistTowardsHub() {
        //the adding of the yaw is to account for the rotation of the robot
        var angleToHub =  getAngleToHub()  + RobotContainer.kNavx.getYaw();
        //add tangential velocity calcs
        var tangVelocity = getTangentialVelocity(RobotContainer.kNavx.getVelocityX(), RobotContainer.kNavx.getVelocityY());
        var robotRotationalTangVelocity = getTangentialVelocity(0, 0);
        var T = getDistanceToHub() / TurretConfig.TurretFieldAndRobotInfo.kShooterVelocity;
        var phantomPitchPosition = T * (tangVelocity + robotRotationalTangVelocity) * Math.cos(90 - getAngleToHub());
        //clamping the value because our turret only goes 360 degrees exactly
        angleToHub = MathUtil.clamp(angleToHub % 360, 0, 360);
        var cmd = waist.setDegreesCommand(angleToHub + TurretConfig.WaistConfig.kWaistDegreesOffset);
        return cmd;
    }

    public Command aimHoodTowardsHub() {
        double distanceToHub = getDistanceToHub();
        var angle = 90 - solvePitch(distanceToHub);
        //add radial velocity calcs
        //clamp to the physical limits of our hood
        angle = MathUtil.clamp(angle, 0.0, 45.0);
        var cmd = hood.setAngleCommand(angle + TurretConfig.HoodConfig.kHoodDegreesOffset);
        return cmd;
    }

    public Command aimTowardsHub() {
        var cmd = Commands.parallel(
            aimHoodTowardsHub(), 
            aimWaistTowardsHub()
            );
        cmd.addRequirements(hood, waist);
        return cmd;
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