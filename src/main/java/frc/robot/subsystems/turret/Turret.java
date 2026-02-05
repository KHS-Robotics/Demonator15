package frc.robot.subsystems.turret;

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
            yDistanceToHub = robotPosition.getY() - TurretConfig.AbsoluteShooterConfig.kRedHubPositionY;
            distanceToHub = robotPosition.getDistance(TurretConfig.AbsoluteShooterConfig.kRedHubPositionOnField);
        //if the alliance is null, it is auto set to blue variables
        }else {
            yDistanceToHub = robotPosition.getY() - TurretConfig.AbsoluteShooterConfig.kBlueHubPositionY;
            distanceToHub = robotPosition.getDistance(TurretConfig.AbsoluteShooterConfig.kBlueHubPositionOnField);
        }
        var angleToHub = Math.asin(yDistanceToHub / distanceToHub);
        //Theta = arcsin(yDistanceToHub / distanceToHub)
        var cmd = waist.setDegreesCommand(angleToHub);
        return cmd;
    }
}
    //for getting the position and rotation (which we will probably want to do in hood and waist), use our humble kNavx
    //
    //    var navXRotation = RobotContainer.kNavx.getRotation2d();
    //    var navXVelocityX = RobotContainer.kNavx.getVelocityX();
    //    var navXVelocityY = RobotContainer.kNavx.getVelocityY();
    //    var swerveDrivePosition = RobotContainer.kSwerveDrive.getPose();
