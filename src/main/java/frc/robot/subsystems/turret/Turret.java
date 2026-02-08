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
        //SmartDashboard.putData(this);
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

    public Command aimWaistTowardsHub() {
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        double yDistanceToHub;
        double distanceToHub;
        if ( alliance.get() == Alliance.Red) {
            yDistanceToHub = robotPosition.getY() - TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionY;
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField);
        //if the alliance is null, it is auto set to blue variables
        }else {
            yDistanceToHub = robotPosition.getY() - TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionY;
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField);
        }
        //the adding of the yaw is to account for the rotation of the robot
        var angleToHub =  Math.asin(yDistanceToHub / distanceToHub)  + RobotContainer.kNavx.getYaw();
        //clamping the value because our turret only goes 360 degrees exactly
        angleToHub = MathUtil.clamp(angleToHub, -180.0, 180.0);
        //Theta = arcsin(yDistanceToHub / distanceToHub)
        var cmd = waist.setDegreesCommand(angleToHub + TurretConfig.WaistConfig.kWaistDegreesOffset);
        return cmd;
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

    public Command aimHoodTowardsHub() {
        Translation2d robotPosition = RobotContainer.kSwerveDrive.getPose().getTranslation();
        var alliance = DriverStation.getAlliance();
        double distanceToHub;
        if ( alliance.get() == Alliance.Red) {
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kRedHubPositionOnField);
        //if the alliance is null, it is auto set to blue variables
        }else {
            distanceToHub = robotPosition.getDistance(TurretConfig.TurretFieldAndRobotInfo.kBlueHubPositionOnField);
        }
        var angle = 90 - solvePitch(distanceToHub) ;
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
    // aimPitch = T * 